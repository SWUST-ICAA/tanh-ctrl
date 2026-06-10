#include "tanh_ctrl/px4_bridge.hpp"

#include <algorithm>

namespace tanh_ctrl {

namespace {

float clampedAxis(double value) {
  return static_cast<float>(std::clamp(value, -1.0, 1.0));
}

}  // namespace

px4_msgs::msg::OffboardControlMode makeThrustTorqueOffboardMode(uint64_t timestamp_us) {
  px4_msgs::msg::OffboardControlMode mode{};
  mode.timestamp = timestamp_us;
  mode.position = false;
  mode.velocity = false;
  mode.acceleration = false;
  mode.attitude = false;
  mode.body_rate = false;
  mode.thrust_and_torque = true;
  mode.direct_actuator = false;
  return mode;
}

px4_msgs::msg::VehicleThrustSetpoint makeVehicleThrustSetpoint(
    const ControlOutput& output, const WrenchSetpointConfig& config, uint64_t timestamp_us, uint64_t timestamp_sample_us) {
  px4_msgs::msg::VehicleThrustSetpoint thrust_sp{};
  thrust_sp.timestamp = timestamp_us;
  thrust_sp.timestamp_sample = timestamp_sample_us;

  const double max_thrust = std::max(1.0e-6, config.max_collective_thrust_n);
  const double normalized_thrust = std::clamp(output.thrust_total / max_thrust, 0.0, 1.0);
  thrust_sp.xyz = {0.0f, 0.0f, static_cast<float>(-normalized_thrust)};
  return thrust_sp;
}

px4_msgs::msg::VehicleTorqueSetpoint makeVehicleTorqueSetpoint(
    const ControlOutput& output, uint64_t timestamp_us, uint64_t timestamp_sample_us) {
  px4_msgs::msg::VehicleTorqueSetpoint torque_sp{};
  torque_sp.timestamp = timestamp_us;
  torque_sp.timestamp_sample = timestamp_sample_us;
  torque_sp.xyz = {
      clampedAxis(output.torque_body.x()),
      clampedAxis(output.torque_body.y()),
      clampedAxis(output.torque_body.z()),
  };
  return torque_sp;
}

}  // namespace tanh_ctrl
