import os
from typing import Optional

import mujoco
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from px4_msgs.msg import (
    VehicleAngularVelocity,
    VehicleAttitude,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleThrustSetpoint,
    VehicleTorqueSetpoint,
)
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from uav_simulator.frame_transforms import (
    enu_to_ned,
    flu_to_frd,
    mujoco_quat_to_px4_quat,
    ned_to_enu,
    yaw_from_px4_quat,
)
from uav_simulator.mixer import Px4QuadMixer, QuadrotorParams


def positive_rate(value, fallback):
    rate = float(value)
    if rate <= 0.0:
        return float(fallback)
    return rate


def nonnegative_std(value):
    return max(0.0, float(value))


def quat_multiply_wxyz(lhs, rhs):
    lw, lx, ly, lz = np.asarray(lhs, dtype=float)
    rw, rx, ry, rz = np.asarray(rhs, dtype=float)
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=float,
    )


class MujocoPx4Bridge(Node):
    def __init__(self):
        super().__init__("uav_simulator")
        self._declare_parameters()

        self.params = QuadrotorParams()
        self.mixer = Px4QuadMixer(self.params)

        self._control_rate_hz = positive_rate(self.get_parameter("control_rate_hz").value, 800.0)
        self._local_position_rate_hz = positive_rate(self.get_parameter("local_position_rate_hz").value, 100.0)
        self._attitude_rate_hz = positive_rate(self.get_parameter("attitude_rate_hz").value, 250.0)
        self._angular_velocity_rate_hz = positive_rate(self.get_parameter("angular_velocity_rate_hz").value, 800.0)
        self._mujoco_substeps = max(1, int(self.get_parameter("mujoco_substeps").value))
        self._load_noise_parameters()

        self.model = mujoco.MjModel.from_xml_path(self._model_path())
        self.model.opt.timestep = 1.0 / (self._control_rate_hz * self._mujoco_substeps)
        self.data = mujoco.MjData(self.model)
        self._reset_model_pose()

        self._gyro_sensor_adr = self._sensor_address("body_gyro")
        self._viewer = self._launch_viewer() if self.get_parameter("use_viewer").value else None
        self._viewer_period_s = 1.0 / 60.0
        self._next_viewer_sync_time = 0.0

        self._collective_thrust_n = 0.0
        self._torque_frd = np.zeros(3)
        self._has_thrust_setpoint = False
        self._has_torque_setpoint = False
        self._previous_velocity_ned = None
        self._previous_velocity_time = None
        self._previous_angular_velocity_frd = None
        self._previous_angular_velocity_time = None
        self._next_angular_velocity_pub_time = 0.0

        sensor_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        command_qos = QoSProfile(depth=10)

        self.local_position_pub = self.create_publisher(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", sensor_qos
        )
        self.attitude_pub = self.create_publisher(VehicleAttitude, "/fmu/out/vehicle_attitude", sensor_qos)
        self.angular_velocity_pub = self.create_publisher(
            VehicleAngularVelocity, "/fmu/out/vehicle_angular_velocity", sensor_qos
        )
        self.vehicle_status_pub = self.create_publisher(VehicleStatus, "/fmu/out/vehicle_status_v1", sensor_qos)

        self.create_subscription(
            VehicleThrustSetpoint, "/fmu/in/vehicle_thrust_setpoint", self._thrust_callback, command_qos
        )
        self.create_subscription(
            VehicleTorqueSetpoint, "/fmu/in/vehicle_torque_setpoint", self._torque_callback, command_qos
        )

        self.control_timer = self.create_timer(1.0 / self._control_rate_hz, self._step)
        self.local_position_timer = self.create_timer(
            1.0 / self._local_position_rate_hz, self._publish_local_position_and_status
        )
        self.attitude_timer = self.create_timer(1.0 / self._attitude_rate_hz, self._publish_attitude)
        self.get_logger().info(
            "Loaded MuJoCo model: "
            f"{self._model_path()} | control={self._control_rate_hz:.1f}Hz, "
            f"substeps={self._mujoco_substeps}, timestep={self.model.opt.timestep:.6f}s"
        )

    def _declare_parameters(self):
        self.declare_parameter("model_path", "")
        self.declare_parameter("use_viewer", True)
        self.declare_parameter("initial_position_ned", [0.0, 0.0, -0.3])
        self.declare_parameter("control_rate_hz", 800.0)
        self.declare_parameter("mujoco_substeps", 2)
        self.declare_parameter("local_position_rate_hz", 100.0)
        self.declare_parameter("attitude_rate_hz", 250.0)
        self.declare_parameter("angular_velocity_rate_hz", 800.0)
        self.declare_parameter("noise.enabled", True)
        self.declare_parameter("noise.seed", 7)
        self.declare_parameter("noise.position_std_m", 0.002)
        self.declare_parameter("noise.velocity_std_mps", 0.01)
        self.declare_parameter("noise.linear_acceleration_std_mps2", 0.05)
        self.declare_parameter("noise.attitude_std_rad", 0.0015)
        self.declare_parameter("noise.angular_velocity_std_radps", 0.005)
        self.declare_parameter("noise.angular_acceleration_std_radps2", 0.05)

    def _load_noise_parameters(self):
        self._noise_enabled = bool(self.get_parameter("noise.enabled").value)
        self._noise_rng = np.random.default_rng(int(self.get_parameter("noise.seed").value))
        self._position_noise_std_m = nonnegative_std(self.get_parameter("noise.position_std_m").value)
        self._velocity_noise_std_mps = nonnegative_std(self.get_parameter("noise.velocity_std_mps").value)
        self._linear_acceleration_noise_std_mps2 = nonnegative_std(
            self.get_parameter("noise.linear_acceleration_std_mps2").value
        )
        self._attitude_noise_std_rad = nonnegative_std(self.get_parameter("noise.attitude_std_rad").value)
        self._angular_velocity_noise_std_radps = nonnegative_std(
            self.get_parameter("noise.angular_velocity_std_radps").value
        )
        self._angular_acceleration_noise_std_radps2 = nonnegative_std(
            self.get_parameter("noise.angular_acceleration_std_radps2").value
        )

    def _model_path(self):
        configured = self.get_parameter("model_path").value
        if configured:
            return os.path.expanduser(str(configured))
        share_dir = get_package_share_directory("uav_simulator")
        return os.path.join(share_dir, "models", "real_x2", "scene.xml")

    def _reset_model_pose(self):
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "hover")
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        initial_position_ned = np.asarray(self.get_parameter("initial_position_ned").value, dtype=float)
        self.data.qpos[0:3] = ned_to_enu(initial_position_ned)
        mujoco.mj_forward(self.model, self.data)

    def _sensor_address(self, name):
        sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sensor_id < 0:
            return None
        return int(self.model.sensor_adr[sensor_id])

    def _launch_viewer(self):
        try:
            import mujoco.viewer

            return mujoco.viewer.launch_passive(self.model, self.data)
        except Exception as exc:
            self.get_logger().warn(f"MuJoCo viewer disabled: {exc}")
            return None

    def _now_us(self):
        return int(self.data.time * 1_000_000)

    # ************ noise tools ***************

    def _noise(self, std, size=3):
        if not self._noise_enabled or std <= 0.0:
            return np.zeros(size)
        return self._noise_rng.normal(0.0, std, size)

    def _noisy_attitude_quat(self, quat_px4):
        if not self._noise_enabled or self._attitude_noise_std_rad <= 0.0:
            return np.asarray(quat_px4, dtype=float)

        delta_angle = self._noise(self._attitude_noise_std_rad)
        angle = float(np.linalg.norm(delta_angle))
        if angle <= 1.0e-12:
            delta_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        else:
            axis = delta_angle / angle
            half_angle = 0.5 * angle
            delta_quat = np.concatenate(([np.cos(half_angle)], axis * np.sin(half_angle)))

        noisy_quat = quat_multiply_wxyz(delta_quat, quat_px4)
        noisy_quat /= max(np.linalg.norm(noisy_quat), 1.0e-12)
        return noisy_quat

    # ***************************************

    def _step(self):
        self._apply_control()
        for _ in range(self._mujoco_substeps):
            mujoco.mj_step(self.model, self.data)

        self._maybe_publish_angular_velocity()
        self._maybe_sync_viewer()

    def _apply_control(self):
        if self._has_thrust_setpoint and self._has_torque_setpoint:
            motor_forces = self.mixer.mix(self._collective_thrust_n, self._torque_frd)
        else:
            motor_forces = np.zeros(4)
        self.data.ctrl[:4] = motor_forces

    def _maybe_publish_angular_velocity(self):
        period_s = 1.0 / self._angular_velocity_rate_hz
        if self.data.time + 1.0e-12 < self._next_angular_velocity_pub_time:
            return
        self._publish_angular_velocity()
        while self._next_angular_velocity_pub_time <= self.data.time + 1.0e-12:
            self._next_angular_velocity_pub_time += period_s

    def _maybe_sync_viewer(self):
        if self._viewer is None:
            return
        if not self._viewer.is_running():
            rclpy.shutdown()
            return
        if self.data.time + 1.0e-12 >= self._next_viewer_sync_time:
            self._viewer.sync()
            self._next_viewer_sync_time += self._viewer_period_s

    def _publish_local_position_and_status(self):
        timestamp_us = self._now_us()
        position_enu = np.array(self.data.qpos[0:3], dtype=float)
        velocity_enu = np.array(self.data.qvel[0:3], dtype=float)
        position_ned = enu_to_ned(position_enu)
        velocity_ned = enu_to_ned(velocity_enu)
        acceleration_ned = self._linear_acceleration_ned(velocity_ned)
        position_ned = position_ned + self._noise(self._position_noise_std_m)
        velocity_ned = velocity_ned + self._noise(self._velocity_noise_std_mps)
        acceleration_ned = acceleration_ned + self._noise(self._linear_acceleration_noise_std_mps2)
        quat_px4 = self._noisy_attitude_quat(mujoco_quat_to_px4_quat(self.data.qpos[3:7]))

        local_position = VehicleLocalPosition()
        local_position.timestamp = timestamp_us
        local_position.timestamp_sample = timestamp_us
        local_position.xy_valid = True
        local_position.z_valid = True
        local_position.v_xy_valid = True
        local_position.v_z_valid = True
        local_position.x = float(position_ned[0])
        local_position.y = float(position_ned[1])
        local_position.z = float(position_ned[2])
        local_position.vx = float(velocity_ned[0])
        local_position.vy = float(velocity_ned[1])
        local_position.vz = float(velocity_ned[2])
        local_position.z_deriv = float(velocity_ned[2])
        local_position.ax = float(acceleration_ned[0])
        local_position.ay = float(acceleration_ned[1])
        local_position.az = float(acceleration_ned[2])
        local_position.heading = float(yaw_from_px4_quat(quat_px4))
        local_position.heading_good_for_control = True

        self._publish_vehicle_status(timestamp_us)
        self.local_position_pub.publish(local_position)

    def _linear_acceleration_ned(self, velocity_ned):
        if self._previous_velocity_ned is None or self._previous_velocity_time is None:
            acceleration_ned = np.zeros(3)
        else:
            dt = max(self.data.time - self._previous_velocity_time, 1.0e-6)
            acceleration_ned = (velocity_ned - self._previous_velocity_ned) / dt
        self._previous_velocity_ned = np.array(velocity_ned, dtype=float)
        self._previous_velocity_time = self.data.time
        return acceleration_ned

    def _publish_attitude(self):
        timestamp_us = self._now_us()
        attitude = VehicleAttitude()
        attitude.timestamp = timestamp_us
        attitude.timestamp_sample = timestamp_us
        quat_px4 = self._noisy_attitude_quat(mujoco_quat_to_px4_quat(self.data.qpos[3:7]))
        attitude.q = [float(v) for v in quat_px4]
        self.attitude_pub.publish(attitude)

    def _publish_angular_velocity(self):
        timestamp_us = self._now_us()
        angular_velocity_frd = flu_to_frd(self._angular_velocity_flu())
        angular_acceleration_frd = self._angular_acceleration_frd(angular_velocity_frd)
        angular_velocity_frd = angular_velocity_frd + self._noise(self._angular_velocity_noise_std_radps)
        angular_acceleration_frd = angular_acceleration_frd + self._noise(
            self._angular_acceleration_noise_std_radps2
        )

        angular_velocity = VehicleAngularVelocity()
        angular_velocity.timestamp = timestamp_us
        angular_velocity.timestamp_sample = timestamp_us
        angular_velocity.xyz = [float(v) for v in angular_velocity_frd]
        angular_velocity.xyz_derivative = [float(v) for v in angular_acceleration_frd]
        self.angular_velocity_pub.publish(angular_velocity)

    def _angular_acceleration_frd(self, angular_velocity_frd):
        if self._previous_angular_velocity_frd is None or self._previous_angular_velocity_time is None:
            angular_acceleration_frd = np.zeros(3)
        else:
            dt = max(self.data.time - self._previous_angular_velocity_time, 1.0e-6)
            angular_acceleration_frd = (angular_velocity_frd - self._previous_angular_velocity_frd) / dt
        self._previous_angular_velocity_frd = np.array(angular_velocity_frd, dtype=float)
        self._previous_angular_velocity_time = self.data.time
        return angular_acceleration_frd

    def _angular_velocity_flu(self):
        if self._gyro_sensor_adr is None:
            return np.array(self.data.qvel[3:6], dtype=float)
        return np.array(self.data.sensordata[self._gyro_sensor_adr : self._gyro_sensor_adr + 3], dtype=float)

    def _publish_vehicle_status(self, timestamp_us):
        msg = VehicleStatus()
        msg.timestamp = timestamp_us
        msg.armed_time = 0
        msg.nav_state_timestamp = 0
        msg.arming_state = VehicleStatus.ARMING_STATE_ARMED
        msg.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        msg.nav_state_user_intention = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        msg.nav_state_display = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        msg.accepts_offboard_setpoints = True
        msg.hil_state = VehicleStatus.HIL_STATE_ON
        msg.vehicle_type = VehicleStatus.VEHICLE_TYPE_ROTARY_WING
        msg.system_type = 2
        msg.system_id = 1
        msg.component_id = 1
        msg.safety_off = True
        msg.power_input_valid = True
        msg.pre_flight_checks_pass = True
        self.vehicle_status_pub.publish(msg)

    def _thrust_callback(self, msg):
        thrust_n = -float(msg.xyz[2]) * self.params.max_collective_thrust_n
        self._collective_thrust_n = float(np.clip(thrust_n, 0.0, self.params.max_collective_thrust_n))
        self._has_thrust_setpoint = True

    def _torque_callback(self, msg):
        normalized_torque = np.clip(np.asarray(msg.xyz, dtype=float), -1.0, 1.0)
        self._torque_frd = normalized_torque * self.mixer.torque_limits
        self._has_torque_setpoint = True


def main(args: Optional[list] = None):
    rclpy.init(args=args)
    node = MujocoPx4Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():
            raise
    finally:
        if node._viewer is not None:
            node._viewer.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
