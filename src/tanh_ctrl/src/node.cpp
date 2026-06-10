#include "tanh_ctrl/tanh_node.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace tanh_ctrl {

namespace {

constexpr double kMinRequestIntervalS = 0.1;
constexpr bool kPublishOffboardControlMode = true;
constexpr bool kPublishWrenchSetpoint = true;
constexpr bool kAutoOffboard = true;
constexpr bool kAutoArm = false;
constexpr int kOffboardWarmup = 10;
constexpr double kMinLoopDt = 1e-4;
constexpr double kMaxLoopDt = 0.1;
constexpr double kDefaultDiagonalWheelbaseM = 0.25;
constexpr double kDefaultMomentToThrustRatioM = 0.3;

/************ time and math tools ***************/

uint64_t nowMicros(rclcpp::Clock &clock) {
  return static_cast<uint64_t>(clock.now().nanoseconds() / 1000ULL);
}

double elapsedSeconds(uint64_t now_us, uint64_t previous_us) {
  if (previous_us == 0 || now_us <= previous_us) {
    return 0.0;
  }
  return static_cast<double>(now_us - previous_us) * 1e-6;
}

Eigen::Vector3d planarAxisVec(double planar, double axial) {
  return Eigen::Vector3d(planar, planar, axial);
}

uint64_t selectMessageTimestampUs(uint64_t timestamp_sample_us,
                                  uint64_t timestamp_us, uint64_t fallback_us) {
  if (timestamp_sample_us != 0) {
    return timestamp_sample_us;
  }
  if (timestamp_us != 0) {
    return timestamp_us;
  }
  return fallback_us;
}

double computeLoopDtFromSample(uint64_t sample_timestamp_us,
                               uint64_t *last_sample_timestamp_us) {
  if (!last_sample_timestamp_us || sample_timestamp_us == 0) {
    return kMinLoopDt;
  }

  if (*last_sample_timestamp_us == 0 ||
      sample_timestamp_us <= *last_sample_timestamp_us) {
    *last_sample_timestamp_us = sample_timestamp_us;
    return kMinLoopDt;
  }

  const double dt =
      static_cast<double>(sample_timestamp_us - *last_sample_timestamp_us) *
      1e-6;
  *last_sample_timestamp_us = sample_timestamp_us;
  return std::clamp(dt, kMinLoopDt, kMaxLoopDt);
}

double quaternionToYaw(const Eigen::Quaterniond &q_body_to_ned) {
  const Eigen::Quaterniond q = q_body_to_ned.normalized();
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

bool requestDue(uint64_t now_us, uint64_t last_request_us, double interval_s) {
  return last_request_us == 0 ||
         elapsedSeconds(now_us, last_request_us) >= interval_s;
}

template <typename Array3T>
Eigen::Vector3d eigenFromArray3(const Array3T &values) {
  return Eigen::Vector3d(static_cast<double>(values[0]),
                         static_cast<double>(values[1]),
                         static_cast<double>(values[2]));
}

template <typename MsgVec3T>
Eigen::Vector3d eigenFromXyz(const MsgVec3T &values) {
  return Eigen::Vector3d(static_cast<double>(values.x),
                         static_cast<double>(values.y),
                         static_cast<double>(values.z));
}

/***************************************/

/************ parameter tools ***************/

void declareAxisPair(rclcpp::Node &node, const char *shared_name,
                     double shared_default, const char *axial_name,
                     double axial_default) {
  node.declare_parameter<double>(shared_name, shared_default);
  node.declare_parameter<double>(axial_name, axial_default);
}

Eigen::Vector3d loadAxisPairParam(rclcpp::Node &node, const char *shared_name,
                                  const char *axial_name) {
  return planarAxisVec(node.get_parameter(shared_name).as_double(),
                       node.get_parameter(axial_name).as_double());
}

/***************************************/

/************ px4 messages ***************/

float clampedAxis(double value) {
  return static_cast<float>(std::clamp(value, -1.0, 1.0));
}

float normalizedTorqueAxis(double torque_n_m, double torque_limit_n_m) {
  const double scale = std::max(1.0e-6, std::abs(torque_limit_n_m));
  return clampedAxis(torque_n_m / scale);
}

Eigen::Vector3d
computeTorqueLimitsFromCollectiveThrust(double max_collective_thrust_n,
                                        double diagonal_wheelbase_m,
                                        double moment_to_thrust_ratio_m) {
  const double max_collective_thrust =
      std::max(1.0e-6, max_collective_thrust_n);
  const double diagonal_wheelbase = std::max(1.0e-6, diagonal_wheelbase_m);
  const double moment_to_thrust_ratio =
      std::max(1.0e-6, moment_to_thrust_ratio_m);
  const double arm_xy_m = diagonal_wheelbase / (2.0 * std::sqrt(2.0));
  const double max_motor_thrust_n = max_collective_thrust / 4.0;
  const double roll_pitch_limit_n_m = 2.0 * arm_xy_m * max_motor_thrust_n;
  const double yaw_limit_n_m = moment_to_thrust_ratio * max_collective_thrust;
  return Eigen::Vector3d(roll_pitch_limit_n_m, roll_pitch_limit_n_m,
                         yaw_limit_n_m);
}

px4_msgs::msg::OffboardControlMode
makeThrustTorqueOffboardMode(uint64_t timestamp_us) {
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

/***************************************/

/************ mission tools ***************/

const char *missionStateName(MissionState state) {
  switch (state) {
  case WAIT_FOR_OFFBOARD:
    return "WAIT_FOR_OFFBOARD";
  case WAIT_FOR_ARMING:
    return "WAIT_FOR_ARMING";
  case TAKEOFF:
    return "TAKEOFF";
  case HOLD:
    return "HOLD";
  case TRACKING:
    return "TRACKING";
  }

  return "UNKNOWN";
}

void logMissionTransition(rclcpp::Logger logger, MissionState from,
                          MissionState to, const char *reason) {
  RCLCPP_INFO(logger, "Mission state: %s -> %s (%s)", missionStateName(from),
              missionStateName(to), reason);
}

/***************************************/

} // namespace

/************ reference tools ***************/

TrajectoryRef trajectoryReferenceFromMsg(
    const flat_trajectory_msgs::msg::FlatTrajectoryReference &msg) {
  TrajectoryRef ref{};
  ref.position_ned = eigenFromXyz(msg.position_ned);
  ref.velocity_ned = eigenFromXyz(msg.velocity_ned);
  ref.acceleration_ned = eigenFromXyz(msg.acceleration_ned);
  ref.jerk_ned = eigenFromXyz(msg.jerk_ned);
  ref.snap_ned = eigenFromXyz(msg.snap_ned);
  ref.yaw_rate = msg.yaw_rate;
  ref.yaw_acceleration = msg.yaw_acceleration;
  ref.has_flatness_feedforward = true;
  ref.yaw = msg.yaw;
  ref.valid = true;
  return ref;
}

TrajectoryRef makeHoldReference(const VehicleState &state, double target_z_ned,
                                double yaw) {
  TrajectoryRef hold_ref;
  hold_ref.position_ned = Eigen::Vector3d(state.position_ned.x(),
                                          state.position_ned.y(), target_z_ned);
  hold_ref.yaw = yaw;
  hold_ref.valid = true;
  return hold_ref;
}

bool hasFreshExternalReference(const TrajectoryRef &external_ref,
                               uint64_t now_us,
                               uint64_t last_reference_receive_us,
                               double timeout_s) {
  if (!external_ref.valid) {
    return false;
  }
  if (timeout_s <= 0.0) {
    return true;
  }
  if (last_reference_receive_us == 0 || now_us <= last_reference_receive_us) {
    return false;
  }
  return elapsedSeconds(now_us, last_reference_receive_us) <= timeout_s;
}

ControlOutput gateControlOutputForPublicationState(const ControlOutput &out,
                                                   bool is_armed,
                                                   bool is_offboard) {
  if (is_armed && is_offboard) {
    return out;
  }

  return ControlOutput{};
}

/***************************************/

/************ node lifecycle ***************/

TanhNode::TanhNode(const rclcpp::NodeOptions &options)
    : Node("tanh_ctrl", options) {
  declareParameters();
  loadStartupParams();
  loadRuntimeTuningParams();
  createRosInterfaces();
  parameter_reload_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TanhNode::loadRuntimeTuningParams, this));
}

void TanhNode::declareParameters() {
  this->declare_parameter<std::string>("topics.reference",
                                       "/tanh_ctrl/reference");
  this->declare_parameter<std::string>("topics.start_tracking",
                                       "/mission/start_tracking");

  this->declare_parameter<double>("mission.takeoff_target_z", -2.0);
  this->declare_parameter<double>("mission.takeoff_z_threshold", 0.2);
  this->declare_parameter<double>("mission.takeoff_hold_time_s", 2.0);
  this->declare_parameter<double>("mission.reference_timeout_s", 0.3);
  this->declare_parameter<double>("mission.request_interval_s", 1.0);

  this->declare_parameter<double>("model.mass", 0.813);
  this->declare_parameter<double>("model.gravity", 9.81);
  this->declare_parameter<double>("model.max_collective_thrust", 59.2);
  this->declare_parameter<double>("model.diagonal_wheelbase_m",
                                  kDefaultDiagonalWheelbaseM);
  this->declare_parameter<double>("model.moment_to_thrust_ratio_m",
                                  kDefaultMomentToThrustRatioM);
  this->declare_parameter<std::vector<double>>(
      "model.inertia_diag", {0.00191426, 0.002370211, 0.003600705});

  declareAxisPair(*this, "position.horizontal.M_P", 2.5,
                  "position.vertical.M_P", 2.0);
  declareAxisPair(*this, "position.horizontal.K_P", 1.0,
                  "position.vertical.K_P", 1.0);
  declareAxisPair(*this, "position.horizontal.M_V", 8.0,
                  "position.vertical.M_V", 6.5);
  declareAxisPair(*this, "position.horizontal.K_V", 0.5,
                  "position.vertical.K_V", 0.5);
  declareAxisPair(*this, "position.horizontal.K_Acceleration", 1.1,
                  "position.vertical.K_Acceleration", 1.0);
  declareAxisPair(*this, "position.horizontal.observer.P_V", 0.0,
                  "position.vertical.observer.P_V", 0.0);
  declareAxisPair(*this, "position.horizontal.observer.L_V", 5.0,
                  "position.vertical.observer.L_V", 5.0);
  this->declare_parameter<double>("position.max_tilt_deg", 35.0);

  declareAxisPair(*this, "attitude.tilt.M_Angle", 3.0, "attitude.yaw.M_Angle",
                  3.0);
  declareAxisPair(*this, "attitude.tilt.K_Angle", 4.0, "attitude.yaw.K_Angle",
                  4.0);
  declareAxisPair(*this, "attitude.tilt.M_AngularVelocity", 20.0,
                  "attitude.yaw.M_AngularVelocity", 15.0);
  declareAxisPair(*this, "attitude.tilt.K_AngularVelocity", 2.0,
                  "attitude.yaw.K_AngularVelocity", 2.0);
  declareAxisPair(*this, "attitude.tilt.K_AngularAcceleration", 0.0,
                  "attitude.yaw.K_AngularAcceleration", 0.0);
  declareAxisPair(*this, "attitude.tilt.observer.P_AngularVelocity", 0.0,
                  "attitude.yaw.observer.P_AngularVelocity", 0.0);
  declareAxisPair(*this, "attitude.tilt.observer.L_AngularVelocity", 5.0,
                  "attitude.yaw.observer.L_AngularVelocity", 5.0);

  this->declare_parameter<double>("filters.velocity_disturbance_cutoff_hz",
                                  0.0);
  this->declare_parameter<double>(
      "filters.angular_velocity_disturbance_cutoff_hz", 0.0);
}

void TanhNode::createRosInterfaces() {
  const auto qos_px4_out = rclcpp::SensorDataQoS();
  const auto qos_default = rclcpp::QoS(rclcpp::KeepLast(10));

  local_position_sub_ =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          "/fmu/out/vehicle_local_position", qos_px4_out,
          std::bind(&TanhNode::localPositionCallback, this,
                    std::placeholders::_1));
  attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos_px4_out,
      std::bind(&TanhNode::attitudeCallback, this, std::placeholders::_1));
  angular_velocity_sub_ =
      this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
          "/fmu/out/vehicle_angular_velocity", qos_px4_out,
          std::bind(&TanhNode::angularVelocityCallback, this,
                    std::placeholders::_1));
  vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", qos_px4_out,
      std::bind(&TanhNode::vehicleStatusCallback, this, std::placeholders::_1));
  reference_sub_ = this->create_subscription<
      flat_trajectory_msgs::msg::FlatTrajectoryReference>(
      this->get_parameter("topics.reference").as_string(), qos_default,
      std::bind(&TanhNode::referenceCallback, this, std::placeholders::_1));

  offboard_mode_pub_ =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>(
          "/fmu/in/offboard_control_mode", qos_default);
  vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", qos_default);
  thrust_sp_pub_ = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
      "/fmu/in/vehicle_thrust_setpoint", qos_default);
  torque_sp_pub_ = this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
      "/fmu/in/vehicle_torque_setpoint", qos_default);
  start_tracking_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      this->get_parameter("topics.start_tracking").as_string(),
      rclcpp::QoS(1).reliable().transient_local());
  publishStartTrackingSignal(false);
}

void TanhNode::loadStartupParams() {
  publish_offboard_control_mode_ = kPublishOffboardControlMode;
  publish_wrench_setpoint_ = kPublishWrenchSetpoint;
  enable_auto_offboard_ = kAutoOffboard;
  enable_auto_arm_ = kAutoArm;
  offboard_setpoint_warmup_ = kOffboardWarmup;

  mission_takeoff_target_z_ =
      this->get_parameter("mission.takeoff_target_z").as_double();
  mission_takeoff_z_threshold_ = std::max(
      0.0, this->get_parameter("mission.takeoff_z_threshold").as_double());
  mission_takeoff_hold_time_s_ = std::max(
      0.0, this->get_parameter("mission.takeoff_hold_time_s").as_double());
  mission_reference_timeout_s_ = std::max(
      0.0, this->get_parameter("mission.reference_timeout_s").as_double());
  mission_request_interval_s_ =
      std::max(kMinRequestIntervalS,
               this->get_parameter("mission.request_interval_s").as_double());
}

void TanhNode::loadRuntimeTuningParams() {
  controller_.setMass(this->get_parameter("model.mass").as_double());
  controller_.setGravity(this->get_parameter("model.gravity").as_double());

  const auto inertia_diag =
      this->get_parameter("model.inertia_diag").as_double_array();
  if (inertia_diag.size() != 3) {
    RCLCPP_WARN(
        this->get_logger(),
        "model.inertia_diag must have length 3; using identity inertia.");
    controller_.setInertia(Eigen::Matrix3d::Identity());
  } else {
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
    inertia(0, 0) = inertia_diag[0];
    inertia(1, 1) = inertia_diag[1];
    inertia(2, 2) = inertia_diag[2];
    controller_.setInertia(inertia);
  }

  PositionGains position_gains;
  position_gains.M_P = loadAxisPairParam(*this, "position.horizontal.M_P",
                                         "position.vertical.M_P");
  position_gains.K_P = loadAxisPairParam(*this, "position.horizontal.K_P",
                                         "position.vertical.K_P");
  position_gains.M_V = loadAxisPairParam(*this, "position.horizontal.M_V",
                                         "position.vertical.M_V");
  position_gains.K_V = loadAxisPairParam(*this, "position.horizontal.K_V",
                                         "position.vertical.K_V");
  position_gains.K_Acceleration =
      loadAxisPairParam(*this, "position.horizontal.K_Acceleration",
                        "position.vertical.K_Acceleration");
  position_gains.P_V =
      loadAxisPairParam(*this, "position.horizontal.observer.P_V",
                        "position.vertical.observer.P_V");
  position_gains.L_V =
      loadAxisPairParam(*this, "position.horizontal.observer.L_V",
                        "position.vertical.observer.L_V");
  controller_.setPositionGains(position_gains);

  const double max_tilt_deg =
      this->get_parameter("position.max_tilt_deg").as_double();
  controller_.setMaxTilt(max_tilt_deg * M_PI / 180.0);

  AttitudeGains attitude_gains;
  attitude_gains.M_Angle =
      loadAxisPairParam(*this, "attitude.tilt.M_Angle", "attitude.yaw.M_Angle");
  attitude_gains.K_Angle =
      loadAxisPairParam(*this, "attitude.tilt.K_Angle", "attitude.yaw.K_Angle");
  attitude_gains.M_AngularVelocity =
      loadAxisPairParam(*this, "attitude.tilt.M_AngularVelocity",
                        "attitude.yaw.M_AngularVelocity");
  attitude_gains.K_AngularVelocity =
      loadAxisPairParam(*this, "attitude.tilt.K_AngularVelocity",
                        "attitude.yaw.K_AngularVelocity");
  attitude_gains.K_AngularAcceleration =
      loadAxisPairParam(*this, "attitude.tilt.K_AngularAcceleration",
                        "attitude.yaw.K_AngularAcceleration");
  attitude_gains.P_AngularVelocity =
      loadAxisPairParam(*this, "attitude.tilt.observer.P_AngularVelocity",
                        "attitude.yaw.observer.P_AngularVelocity");
  attitude_gains.L_AngularVelocity =
      loadAxisPairParam(*this, "attitude.tilt.observer.L_AngularVelocity",
                        "attitude.yaw.observer.L_AngularVelocity");
  controller_.setAttitudeGains(attitude_gains);

  controller_.setVelocityDisturbanceLowPassHz(
      this->get_parameter("filters.velocity_disturbance_cutoff_hz")
          .as_double());
  controller_.setAngularVelocityDisturbanceLowPassHz(
      this->get_parameter("filters.angular_velocity_disturbance_cutoff_hz")
          .as_double());

  const double max_collective_thrust_n = std::max(
      1.0e-6, this->get_parameter("model.max_collective_thrust").as_double());
  const double diagonal_wheelbase_m = std::max(
      1.0e-6, this->get_parameter("model.diagonal_wheelbase_m").as_double());
  const double moment_to_thrust_ratio_m = std::max(
      1.0e-6,
      this->get_parameter("model.moment_to_thrust_ratio_m").as_double());
  wrench_setpoint_config_.max_collective_thrust_n = max_collective_thrust_n;
  wrench_setpoint_config_.torque_limit_body_n_m =
      computeTorqueLimitsFromCollectiveThrust(max_collective_thrust_n,
                                              diagonal_wheelbase_m,
                                              moment_to_thrust_ratio_m);
}

/***************************************/

/************ callbacks ***************/

void TanhNode::localPositionCallback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
  if (!msg) {
    return;
  }

  if (!msg->xy_valid || !msg->z_valid) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "vehicle_local_position is missing valid position "
                         "data; waiting for estimator readiness.");
    return;
  }

  if (!msg->v_xy_valid || !msg->v_z_valid) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "vehicle_local_position is missing valid velocity "
                         "data; waiting for estimator readiness.");
    return;
  }

  const uint64_t sample_us = selectMessageTimestampUs(
      msg->timestamp_sample, msg->timestamp, nowMicros(*this->get_clock()));
  state_.position_ned = Eigen::Vector3d(msg->x, msg->y, msg->z);
  state_.velocity_ned = Eigen::Vector3d(msg->vx, msg->vy, msg->vz);
  state_.linear_acceleration_ned = Eigen::Vector3d(msg->ax, msg->ay, msg->az);

  has_position_state_ = true;
  positionControlLoop(sample_us);
}

void TanhNode::attitudeCallback(
    const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
  if (!msg) {
    return;
  }

  Eigen::Quaterniond attitude(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
  attitude.normalize();
  state_.q_body_to_ned = attitude;
  current_yaw_ = quaternionToYaw(attitude);
  has_attitude_state_ = true;
}

void TanhNode::angularVelocityCallback(
    const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) {
  if (!msg) {
    return;
  }

  state_.angular_velocity_body = eigenFromArray3(msg->xyz);
  state_.angular_acceleration_body = eigenFromArray3(msg->xyz_derivative);
  has_angular_velocity_state_ = true;

  const uint64_t sample_us = selectMessageTimestampUs(
      msg->timestamp_sample, msg->timestamp, nowMicros(*this->get_clock()));
  attitudeControlLoop(sample_us);
}

void TanhNode::vehicleStatusCallback(
    const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
  if (!msg) {
    return;
  }

  const bool was_armed = is_armed_;
  const bool was_offboard = is_offboard_;
  const uint8_t previous_nav_state = last_nav_state_;

  last_nav_state_ = msg->nav_state;
  is_armed_ =
      (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
  is_offboard_ = (msg->nav_state ==
                  px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);

  if (!is_armed_) {
    saw_disarmed_status_ = true;
    manual_arm_detected_ = false;
  } else if (!was_armed && saw_disarmed_status_) {
    manual_arm_detected_ = true;
  }

  if (is_offboard_) {
    offboard_ever_engaged_ = true;
  }

  if (was_armed && !is_armed_) {
    resetControllerRuntimeState();
  }

  if (offboard_ever_engaged_ && was_offboard && !is_offboard_ &&
      !exit_requested_) {
    exit_requested_ = true;
    RCLCPP_ERROR(this->get_logger(),
                 "Detected offboard exit, nav_state changed from %u to %u. "
                 "Controller will shut down.",
                 static_cast<unsigned>(previous_nav_state),
                 static_cast<unsigned>(msg->nav_state));
  }
}

void TanhNode::referenceCallback(
    const flat_trajectory_msgs::msg::FlatTrajectoryReference::SharedPtr msg) {
  if (!msg) {
    return;
  }

  const uint64_t now_us = nowMicros(*this->get_clock());
  external_ref_ = trajectoryReferenceFromMsg(*msg);
  last_reference_receive_us_ = now_us;
}

/***************************************/

/************ control loops ***************/

void TanhNode::positionControlLoop(uint64_t sample_us) {
  if (!has_position_state_ || !has_attitude_state_) {
    return;
  }

  if (!has_hold_ref_) {
    updateHoldReference(state_.position_ned.z());
  }

  const uint64_t now_us = nowMicros(*this->get_clock());
  handleMissionPreconditions();
  updateMissionStateMachine(now_us);

  const TrajectoryRef *active_ref = selectActiveReference(now_us);
  if (!active_ref || !active_ref->valid) {
    position_loop_command_ = AttitudeReference{};
    has_position_loop_command_ = false;
    return;
  }

  AttitudeReference position_loop_command{};
  const double dt = computeLoopDtFromSample(sample_us, &last_position_loop_us_);
  const bool freeze_controller_state = !is_armed_;
  if (freeze_controller_state) {
    controller_.reset();
  }

  if (!controller_.computePositionLoop(state_, *active_ref, dt,
                                       &position_loop_command)) {
    if (freeze_controller_state) {
      controller_.reset();
    }
    position_loop_command_ = AttitudeReference{};
    has_position_loop_command_ = false;
    return;
  }

  if (freeze_controller_state) {
    controller_.reset();
  }

  position_loop_command_ = position_loop_command;
  has_position_loop_command_ = position_loop_command_.valid;
}

void TanhNode::attitudeControlLoop(uint64_t sample_us) {
  if (exit_requested_) {
    RCLCPP_ERROR(this->get_logger(),
                 "Shutting down controller because offboard mode was exited.");
    rclcpp::shutdown();
    return;
  }

  const uint64_t now_us = nowMicros(*this->get_clock());
  publishOffboardControlMode(now_us);

  if (!has_position_state_ || !has_attitude_state_ ||
      !has_angular_velocity_state_) {
    return;
  }

  maybeSendAutomaticRequests(now_us);

  if (!has_position_loop_command_ || !position_loop_command_.valid) {
    return;
  }

  ControlOutput out;
  const double dt = computeLoopDtFromSample(sample_us, &last_attitude_loop_us_);
  const bool freeze_controller_state = !is_armed_;
  if (freeze_controller_state) {
    controller_.reset();
  }

  if (!controller_.computeAttitudeLoop(state_, position_loop_command_, dt,
                                       &out)) {
    if (freeze_controller_state) {
      controller_.reset();
    }
    return;
  }

  if (freeze_controller_state) {
    controller_.reset();
  }

  const ControlOutput publish_out =
      gateControlOutputForPublicationState(out, is_armed_, is_offboard_);
  publishWrenchSetpoint(publish_out, now_us, sample_us);
}

/***************************************/

/************ publishers ***************/

void TanhNode::publishVehicleCommand(uint32_t command, float param1,
                                     float param2, float param3) {
  if (!vehicle_command_pub_) {
    return;
  }

  px4_msgs::msg::VehicleCommand cmd{};
  cmd.timestamp = nowMicros(*this->get_clock());
  cmd.param1 = param1;
  cmd.param2 = param2;
  cmd.param3 = param3;
  cmd.command = command;
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;
  vehicle_command_pub_->publish(cmd);
}

void TanhNode::publishStartTrackingSignal(bool enabled) {
  if (!start_tracking_pub_) {
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = enabled;
  start_tracking_pub_->publish(msg);
}

void TanhNode::publishOffboardControlMode(uint64_t now_us) {
  if (!publish_offboard_control_mode_ || !offboard_mode_pub_) {
    return;
  }

  offboard_mode_pub_->publish(makeThrustTorqueOffboardMode(now_us));
}

void TanhNode::publishWrenchSetpoint(const ControlOutput &out, uint64_t now_us,
                                     uint64_t sample_us) {
  if (!publish_wrench_setpoint_ || !thrust_sp_pub_ || !torque_sp_pub_) {
    return;
  }

  px4_msgs::msg::VehicleThrustSetpoint thrust_sp{};
  thrust_sp.timestamp = now_us;
  thrust_sp.timestamp_sample = sample_us;

  const double max_thrust =
      std::max(1.0e-6, wrench_setpoint_config_.max_collective_thrust_n);
  const double normalized_thrust =
      std::clamp(out.thrust_total / max_thrust, 0.0, 1.0);
  thrust_sp.xyz = {0.0f, 0.0f, static_cast<float>(-normalized_thrust)};

  px4_msgs::msg::VehicleTorqueSetpoint torque_sp{};
  torque_sp.timestamp = now_us;
  torque_sp.timestamp_sample = sample_us;
  torque_sp.xyz = {
      normalizedTorqueAxis(out.torque_body.x(),
                           wrench_setpoint_config_.torque_limit_body_n_m.x()),
      normalizedTorqueAxis(out.torque_body.y(),
                           wrench_setpoint_config_.torque_limit_body_n_m.y()),
      normalizedTorqueAxis(out.torque_body.z(),
                           wrench_setpoint_config_.torque_limit_body_n_m.z()),
  };

  thrust_sp_pub_->publish(thrust_sp);
  torque_sp_pub_->publish(torque_sp);
}

/***************************************/

/************ mission state ***************/

void TanhNode::resetControllerRuntimeState() { controller_.reset(); }

void TanhNode::updateHoldReference(double target_z_ned) {
  if (!has_position_state_) {
    return;
  }

  hold_ref_ = makeHoldReference(state_, target_z_ned, current_yaw_);
  has_hold_ref_ = true;
}

void TanhNode::updateCurrentHoldReference() {
  updateHoldReference(state_.position_ned.z());
}

void TanhNode::resetMissionProgress() {
  publishStartTrackingSignal(false);
  start_tracking_sent_ = false;
  resetTakeoffProgress();
}

void TanhNode::resetTakeoffProgress() {
  takeoff_reached_ = false;
  takeoff_reached_since_us_ = 0;
}

bool TanhNode::takeoffHoldComplete(uint64_t now_us) {
  const double altitude_error =
      std::abs(state_.position_ned.z() - mission_takeoff_target_z_);
  if (altitude_error > mission_takeoff_z_threshold_) {
    resetTakeoffProgress();
    return false;
  }

  if (!takeoff_reached_) {
    takeoff_reached_ = true;
    takeoff_reached_since_us_ = now_us;
  }

  return elapsedSeconds(now_us, takeoff_reached_since_us_) >=
         mission_takeoff_hold_time_s_;
}

void TanhNode::publishStartTrackingOnce() {
  if (start_tracking_sent_) {
    return;
  }

  publishStartTrackingSignal(true);
  start_tracking_sent_ = true;
}

void TanhNode::handleMissionPreconditions() {
  if (!is_armed_ && mission_state_ != WAIT_FOR_ARMING) {
    const MissionState previous_state = mission_state_;
    resetMissionProgress();
    updateCurrentHoldReference();
    mission_state_ = WAIT_FOR_ARMING;
    logMissionTransition(this->get_logger(), previous_state, mission_state_,
                         "vehicle disarmed");
  }

  if (manual_arm_detected_ && is_armed_ && !is_offboard_ &&
      mission_state_ != WAIT_FOR_OFFBOARD) {
    const MissionState previous_state = mission_state_;
    resetMissionProgress();
    updateCurrentHoldReference();
    mission_state_ = WAIT_FOR_OFFBOARD;
    logMissionTransition(this->get_logger(), previous_state, mission_state_,
                         "waiting for offboard");
  }
}

void TanhNode::maybeSendAutomaticRequests(uint64_t now_us) {
  if (has_position_loop_command_ &&
      offboard_counter_ < offboard_setpoint_warmup_) {
    ++offboard_counter_;
  }

  const bool warmup_done = offboard_counter_ >= offboard_setpoint_warmup_;
  if (!has_position_loop_command_ || !warmup_done) {
    return;
  }

  if (enable_auto_offboard_ && manual_arm_detected_ && is_armed_ &&
      !is_offboard_ &&
      requestDue(now_us, last_offboard_request_us_,
                 mission_request_interval_s_)) {
    publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f,
        0.0f);
    last_offboard_request_us_ = now_us;
  }

  if (enable_auto_arm_ && is_offboard_ && !is_armed_ &&
      requestDue(now_us, last_arm_request_us_, mission_request_interval_s_)) {
    publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f,
        0.0f, 0.0f);
    last_arm_request_us_ = now_us;
  }
}

void TanhNode::updateMissionStateMachine(uint64_t now_us) {
  switch (mission_state_) {
  case WAIT_FOR_OFFBOARD:
    updateCurrentHoldReference();
    if (is_offboard_) {
      last_offboard_request_us_ = 0;
      updateHoldReference(mission_takeoff_target_z_);
      resetControllerRuntimeState();
      resetTakeoffProgress();
      logMissionTransition(this->get_logger(), mission_state_, TAKEOFF,
                           "offboard enabled");
      mission_state_ = TAKEOFF;
    }
    break;

  case WAIT_FOR_ARMING:
    updateCurrentHoldReference();
    if (is_armed_ && !is_offboard_) {
      last_arm_request_us_ = 0;
      logMissionTransition(this->get_logger(), mission_state_,
                           WAIT_FOR_OFFBOARD, "vehicle armed");
      mission_state_ = WAIT_FOR_OFFBOARD;
    } else if (is_armed_ && is_offboard_) {
      last_arm_request_us_ = 0;
      updateHoldReference(mission_takeoff_target_z_);
      resetControllerRuntimeState();
      resetTakeoffProgress();
      logMissionTransition(this->get_logger(), mission_state_, TAKEOFF,
                           "vehicle armed in offboard");
      mission_state_ = TAKEOFF;
    }
    break;

  case TAKEOFF:
    updateHoldReference(mission_takeoff_target_z_);
    if (takeoffHoldComplete(now_us)) {
      if (!start_tracking_sent_) {
        publishStartTrackingOnce();
      }
      logMissionTransition(this->get_logger(), mission_state_, HOLD,
                           "takeoff complete");
      mission_state_ = HOLD;
    }
    break;

  case HOLD:
    if (hasFreshExternalReference(external_ref_, now_us,
                                  last_reference_receive_us_,
                                  mission_reference_timeout_s_)) {
      logMissionTransition(this->get_logger(), mission_state_, TRACKING,
                           "trajectory received");
      mission_state_ = TRACKING;
    }
    break;

  case TRACKING:
    if (!hasFreshExternalReference(external_ref_, now_us,
                                   last_reference_receive_us_,
                                   mission_reference_timeout_s_)) {
      updateCurrentHoldReference();
      logMissionTransition(this->get_logger(), mission_state_, HOLD,
                           "trajectory timeout");
      mission_state_ = HOLD;
    }
    break;
  }
}

const TrajectoryRef *TanhNode::selectActiveReference(uint64_t now_us) const {
  if (mission_state_ == TRACKING &&
      hasFreshExternalReference(external_ref_, now_us,
                                last_reference_receive_us_,
                                mission_reference_timeout_s_)) {
    return &external_ref_;
  }

  if (has_hold_ref_ && hold_ref_.valid) {
    return &hold_ref_;
  }

  return nullptr;
}

/***************************************/

} // namespace tanh_ctrl
