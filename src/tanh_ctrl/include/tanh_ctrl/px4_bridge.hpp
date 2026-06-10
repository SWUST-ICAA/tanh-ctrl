#pragma once

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>

#include "tanh_ctrl/types.hpp"

namespace tanh_ctrl {

struct WrenchSetpointConfig {
  double max_collective_thrust_n{1.0};
};

px4_msgs::msg::OffboardControlMode makeThrustTorqueOffboardMode(uint64_t timestamp_us);

px4_msgs::msg::VehicleThrustSetpoint makeVehicleThrustSetpoint(
    const ControlOutput& output, const WrenchSetpointConfig& config, uint64_t timestamp_us, uint64_t timestamp_sample_us);

px4_msgs::msg::VehicleTorqueSetpoint makeVehicleTorqueSetpoint(
    const ControlOutput& output, uint64_t timestamp_us, uint64_t timestamp_sample_us);

}  // namespace tanh_ctrl
