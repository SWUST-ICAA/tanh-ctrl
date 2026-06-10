#pragma once

#include <array>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <flat_trajectory_msgs/msg/flat_trajectory_reference.hpp>

#include "tanh_ctrl/px4_bridge.hpp"
#include "tanh_ctrl/tanh_controller.hpp"

namespace tanh_ctrl {

enum MissionState {
  WAIT_FOR_OFFBOARD,
  WAIT_FOR_ARMING,
  TAKEOFF,
  HOLD,
  TRACKING,
};

TrajectoryRef trajectoryReferenceFromMsg(const flat_trajectory_msgs::msg::FlatTrajectoryReference& msg);

TrajectoryRef makeHoldReference(const VehicleState& state, double target_z_ned, double yaw);

bool hasFreshExternalReference(const TrajectoryRef& external_ref, uint64_t now_us, uint64_t last_reference_receive_us, double timeout_s);

ControlOutput gateControlOutputForPublicationState(const ControlOutput& out, bool is_armed, bool is_offboard);

class TanhNode : public rclcpp::Node {
 public:
  explicit TanhNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void declareParameters();
  void loadParams();
  void createRosInterfaces();

  void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  void angularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg);
  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void referenceCallback(const flat_trajectory_msgs::msg::FlatTrajectoryReference::SharedPtr msg);
  void positionControlLoop(uint64_t sample_us);
  void attitudeControlLoop(uint64_t sample_us);

  void publishVehicleCommand(uint32_t command, float param1, float param2, float param3);
  void publishStartTrackingSignal(bool enabled);
  void publishOffboardControlMode(uint64_t now_us);
  void publishWrenchSetpoint(const ControlOutput& out, uint64_t now_us, uint64_t sample_us);
  void resetControllerRuntimeState();

  void updateHoldReference(double target_z_ned);
  void updateCurrentHoldReference();
  void resetMissionProgress();
  void publishStartTrackingOnce();
  void resetTakeoffProgress();
  bool takeoffHoldComplete(uint64_t now_us);
  void handleMissionPreconditions();
  void maybeSendAutomaticRequests(uint64_t now_us);
  void updateMissionStateMachine(uint64_t now_us);
  const TrajectoryRef* selectActiveReference(uint64_t now_us) const;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<flat_trajectory_msgs::msg::FlatTrajectoryReference>::SharedPtr reference_sub_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_sp_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_sp_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_tracking_pub_;

  VehicleState state_{};
  bool has_position_state_{false};
  bool has_attitude_state_{false};
  bool has_angular_velocity_state_{false};
  TrajectoryRef external_ref_{};
  TrajectoryRef hold_ref_{};
  AttitudeReference position_loop_command_{};
  bool has_hold_ref_{false};
  bool has_position_loop_command_{false};
  uint64_t last_position_loop_us_{0};
  uint64_t last_attitude_loop_us_{0};
  uint64_t last_reference_receive_us_{0};
  bool is_armed_{false};
  bool is_offboard_{false};
  bool saw_disarmed_status_{false};
  bool manual_arm_detected_{false};
  bool offboard_ever_engaged_{false};
  bool exit_requested_{false};
  uint8_t last_nav_state_{0};
  double current_yaw_{0.0};
  MissionState mission_state_{WAIT_FOR_ARMING};
  bool takeoff_reached_{false};
  uint64_t takeoff_reached_since_us_{0};
  bool start_tracking_sent_{false};
  uint64_t last_offboard_request_us_{0};
  uint64_t last_arm_request_us_{0};

  TanhController controller_{};

  bool publish_offboard_control_mode_{true};
  bool publish_wrench_setpoint_{true};
  WrenchSetpointConfig wrench_setpoint_config_{};
  bool enable_auto_offboard_{true};
  bool enable_auto_arm_{false};
  int offboard_setpoint_warmup_{10};
  int offboard_counter_{0};
  double mission_takeoff_target_z_{-2.0};
  double mission_takeoff_z_threshold_{0.2};
  double mission_takeoff_hold_time_s_{2.0};
  double mission_reference_timeout_s_{0.3};
  double mission_request_interval_s_{1.0};
};

}  // namespace tanh_ctrl
