#pragma once

#include <Eigen/Dense>

#include "tanh_ctrl/common.hpp"
#include "tanh_ctrl/tanh_blocks.hpp"

namespace tanh_ctrl {

/**
 * @brief Tanh feedback controller for quadrotor position and attitude stabilization.
 */
class TanhController {
 public:
  TanhController();

  void setMass(double mass);
  void setGravity(double gravity);
  void setPositionGains(const PositionGains& gains);
  void setAttitudeGains(const AttitudeGains& gains);
  void setInertia(const Eigen::Matrix3d& inertia);

  void setMaxTilt(double max_tilt_rad);
  void setVelocityDisturbanceLowPassHz(double cutoff_hz);
  void setAngularVelocityDisturbanceLowPassHz(double cutoff_hz);

  void reset();

  bool computePositionLoop(const VehicleState& state, const TrajectoryRef& ref, double dt, AttitudeReference* attitude_reference);
  bool computeAttitudeLoop(const VehicleState& state, const AttitudeReference& attitude_reference, double dt, ControlOutput* out);

 private:
  void computePosition(const VehicleState& state, const TrajectoryRef& ref, double dt, Eigen::Vector3d* thrust_vec_ned);
  void computeAttitude(const VehicleState& state, const AttitudeReference& attitude_reference, double dt, Eigen::Vector3d* torque_body);
  void initializeLoopState();

  double mass_{1.0};
  double gravity_{9.81};
  Eigen::Matrix3d inertia_{Eigen::Matrix3d::Identity()};
  PositionGains pos_gains_{};
  AttitudeGains att_gains_{};
  double max_tilt_rad_{0.0};

  Eigen::Vector3d velocity_error_hat_ned_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_error_hat_body_{Eigen::Vector3d::Zero()};
  bool first_run_{true};

  Vec3LowPass velocity_disturbance_lpf_{};
  Vec3LowPass angular_velocity_disturbance_lpf_{};
};

}  // namespace tanh_ctrl
