#include "tanh_ctrl/px4_bridge.hpp"

#include <algorithm>
#include <cmath>

namespace tanh_ctrl {

namespace {

float clampedAxis(double value) {
  return static_cast<float>(std::clamp(value, -1.0, 1.0));
}

float normalizedTorqueAxis(double torque_n_m, double max_torque_n_m) {
  const double scale = std::max(1.0e-6, std::abs(max_torque_n_m));
  return clampedAxis(torque_n_m / scale);
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
    const ControlOutput& output, const WrenchSetpointConfig& config, uint64_t timestamp_us, uint64_t timestamp_sample_us) {
  px4_msgs::msg::VehicleTorqueSetpoint torque_sp{};
  torque_sp.timestamp = timestamp_us;
  torque_sp.timestamp_sample = timestamp_sample_us;
  torque_sp.xyz = {
      normalizedTorqueAxis(output.torque_body.x(), config.max_torque_body_n_m[0]),
      normalizedTorqueAxis(output.torque_body.y(), config.max_torque_body_n_m[1]),
      normalizedTorqueAxis(output.torque_body.z(), config.max_torque_body_n_m[2]),
  };
  return torque_sp;
}

}  // namespace tanh_ctrl
