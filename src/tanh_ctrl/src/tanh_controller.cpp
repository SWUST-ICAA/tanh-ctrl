#include "tanh_ctrl/tanh_controller.hpp"

#include <algorithm>
#include <cmath>
#include <sophus/so3.hpp>

namespace tanh_ctrl {

namespace {

constexpr double kMinMass = 1e-3;
constexpr double kMinTiltRad = 1e-3;
constexpr double kMinDt = 1e-4;
constexpr double kMaxDt = 0.1;
constexpr double kMinThrustOverMass = 1e-3;

double clampPositive(double value, double min_value) {
  return std::max(min_value, value);
}

struct FlatnessAttitudeFeedforward {
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration_body{Eigen::Vector3d::Zero()};
};

FlatnessAttitudeFeedforward computeFlatnessAttitudeFeedforward(const AttitudeReference& attitude_reference, double mass) {
  FlatnessAttitudeFeedforward feedforward;

  const double thrust_over_mass = attitude_reference.collective_thrust / clampPositive(mass, kMinMass);
  if (!attitude_reference.has_flatness_feedforward || thrust_over_mass <= kMinThrustOverMass) {
    return feedforward;
  }

  const Sophus::SO3d desired_body_to_ned_so3(attitude_reference.attitude_body_to_ned.normalized());
  const Eigen::Matrix3d desired_body_to_ned = desired_body_to_ned_so3.matrix();
  const Eigen::Vector3d x_body_ned = desired_body_to_ned.col(0);
  const Eigen::Vector3d y_body_ned = desired_body_to_ned.col(1);
  const Eigen::Vector3d z_body_ned = desired_body_to_ned.col(2);
  const Eigen::Vector3d z_world_ned = Eigen::Vector3d::UnitZ();

  const double thrust_over_mass_dot = -attitude_reference.jerk_ned.dot(z_body_ned);
  const Eigen::Vector3d reference_h_omega_ned =
      -(attitude_reference.jerk_ned + thrust_over_mass_dot * z_body_ned) / thrust_over_mass;

  feedforward.angular_velocity_body.x() = -reference_h_omega_ned.dot(y_body_ned);
  feedforward.angular_velocity_body.y() = reference_h_omega_ned.dot(x_body_ned);
  feedforward.angular_velocity_body.z() = attitude_reference.yaw_rate * z_world_ned.dot(z_body_ned);

  const Eigen::Vector3d omega_ned = desired_body_to_ned * feedforward.angular_velocity_body;
  const double thrust_over_mass_ddot =
      -attitude_reference.snap_ned.dot(z_body_ned) - reference_h_omega_ned.dot(attitude_reference.jerk_ned);
  const Eigen::Vector3d omega_cross_h_omega_ned = omega_ned.cross(reference_h_omega_ned);
  const Eigen::Vector3d reference_h_alpha_ned =
      -attitude_reference.snap_ned / thrust_over_mass - omega_cross_h_omega_ned -
      2.0 * (thrust_over_mass_dot / thrust_over_mass) * reference_h_omega_ned -
      (thrust_over_mass_ddot / thrust_over_mass) * z_body_ned;

  feedforward.angular_acceleration_body.x() = -reference_h_alpha_ned.dot(y_body_ned);
  feedforward.angular_acceleration_body.y() = reference_h_alpha_ned.dot(x_body_ned);
  feedforward.angular_acceleration_body.z() = attitude_reference.yaw_acceleration * z_world_ned.dot(z_body_ned);

  return feedforward;
}

}  // namespace

TanhController::TanhController() = default;

void TanhController::setMass(double mass) {
  mass_ = clampPositive(mass, kMinMass);
}

void TanhController::setGravity(double gravity) {
  gravity_ = std::max(0.0, gravity);
}

void TanhController::setPositionGains(const PositionGains& gains) {
  pos_gains_ = gains;
}

void TanhController::setAttitudeGains(const AttitudeGains& gains) {
  att_gains_ = gains;
}

void TanhController::setInertia(const Eigen::Matrix3d& inertia) {
  inertia_ = inertia;
}

void TanhController::setMaxTilt(double max_tilt_rad) {
  max_tilt_rad_ = max_tilt_rad <= 0.0 ? 0.0 : std::clamp(max_tilt_rad, kMinTiltRad, M_PI_2 - kMinTiltRad);
}

void TanhController::setVelocityDisturbanceLowPassHz(double cutoff_hz) {
  velocity_disturbance_lpf_.cutoff_hz = Eigen::Vector3d::Constant(cutoff_hz);
  reset_low_pass(velocity_disturbance_lpf_);
}

void TanhController::setAngularVelocityDisturbanceLowPassHz(double cutoff_hz) {
  angular_velocity_disturbance_lpf_.cutoff_hz = Eigen::Vector3d::Constant(cutoff_hz);
  reset_low_pass(angular_velocity_disturbance_lpf_);
}

void TanhController::reset() {
  velocity_error_hat_ned_.setZero();
  angular_velocity_error_hat_body_.setZero();
  first_run_ = true;

  reset_low_pass(velocity_disturbance_lpf_);
  reset_low_pass(angular_velocity_disturbance_lpf_);
}

bool TanhController::computePositionLoop(const VehicleState& state, const TrajectoryRef& ref, double dt, AttitudeReference* attitude_reference) {
  if (!attitude_reference || !ref.valid) {
    return false;
  }

  initializeLoopState();
  dt = std::clamp(dt, kMinDt, kMaxDt);

  Eigen::Vector3d thrust_vec_ned = Eigen::Vector3d::Zero();
  computePosition(state, ref, dt, &thrust_vec_ned);

  *attitude_reference = computeAttitudeReference(thrust_vec_ned, ref);
  return attitude_reference->valid;
}

bool TanhController::computeAttitudeLoop(const VehicleState& state, const AttitudeReference& attitude_reference, double dt, ControlOutput* out) {
  if (!out || !attitude_reference.valid) {
    return false;
  }

  initializeLoopState();
  dt = std::clamp(dt, kMinDt, kMaxDt);

  Eigen::Vector3d torque_body = Eigen::Vector3d::Zero();
  computeAttitude(state, attitude_reference, dt, &torque_body);

  out->thrust_total = attitude_reference.collective_thrust;
  out->torque_body = torque_body;
  return true;
}

void TanhController::initializeLoopState() {
  if (!first_run_) {
    return;
  }

  velocity_error_hat_ned_.setZero();
  angular_velocity_error_hat_body_.setZero();
  first_run_ = false;
}

void TanhController::computePosition(const VehicleState& state, const TrajectoryRef& ref, double dt, Eigen::Vector3d* thrust_vec_ned) {
  const Eigen::Vector3d position_error_ned = state.position_ned - ref.position_ned;
  const Eigen::Vector3d tanh_position_error = tanh_feedback(position_error_ned, pos_gains_.K_P, Eigen::Vector3d::Ones());
  const Eigen::Vector3d velocity_error_ned = (state.velocity_ned - ref.velocity_ned) + pos_gains_.M_P.cwiseProduct(tanh_position_error);
  const Eigen::Vector3d velocity_estimation_error_ned = velocity_error_ned - velocity_error_hat_ned_;
  const Eigen::Vector3d velocity_disturbance_raw = tanh_feedback(velocity_estimation_error_ned, pos_gains_.L_V, pos_gains_.P_V);
  const Eigen::Vector3d velocity_disturbance_filtered = update_low_pass(velocity_disturbance_raw, dt, velocity_disturbance_lpf_);
  const Eigen::Vector3d gravity_ned(0.0, 0.0, gravity_);
  const Eigen::Vector3d tanh_velocity_error = tanh_feedback(velocity_error_ned, pos_gains_.K_V, Eigen::Vector3d::Ones());
  const Eigen::Vector3d velocity_feedback = pos_gains_.M_V.cwiseProduct(tanh_velocity_error);
  const Eigen::Vector3d acceleration_feedback =
      pos_gains_.K_Acceleration.cwiseProduct(state.linear_acceleration_ned - ref.acceleration_ned);

  const auto thrustOverMassFromDisturbance = [&](const Eigen::Vector3d& disturbance) {
    return disturbance + gravity_ned + velocity_feedback + acceleration_feedback - ref.acceleration_ned;
  };

  Eigen::Vector3d thrust_over_mass_control = thrustOverMassFromDisturbance(velocity_disturbance_filtered);
  Eigen::Vector3d thrust_over_mass_observer = thrustOverMassFromDisturbance(velocity_disturbance_raw);

  applyTiltLimit(&thrust_over_mass_control, max_tilt_rad_);
  applyTiltLimit(&thrust_over_mass_observer, max_tilt_rad_);

  const Eigen::Vector3d desired_thrust_control = mass_ * thrust_over_mass_control;
  const Eigen::Vector3d desired_thrust_observer = mass_ * thrust_over_mass_observer;

  const Eigen::Vector3d velocity_error_hat_dot_ned = (-desired_thrust_observer / mass_) + gravity_ned + velocity_disturbance_raw - ref.acceleration_ned;
  velocity_error_hat_ned_ += dt * velocity_error_hat_dot_ned;

  if (thrust_vec_ned) {
    *thrust_vec_ned = desired_thrust_control;
  }
}

void TanhController::computeAttitude(const VehicleState& state, const AttitudeReference& attitude_reference, double dt, Eigen::Vector3d* torque_body) {
  const Eigen::Quaterniond q = state.q_body_to_ned.normalized();
  const Eigen::Quaterniond q_d = attitude_reference.attitude_body_to_ned.normalized();
  const Sophus::SO3d current_so3(q);
  const Sophus::SO3d desired_so3(q_d);
  const Eigen::Matrix3d current_to_desired_body = current_so3.matrix().transpose() * desired_so3.matrix();

  const FlatnessAttitudeFeedforward feedforward = computeFlatnessAttitudeFeedforward(attitude_reference, mass_);
  const Eigen::Vector3d desired_angular_velocity_body = current_to_desired_body * feedforward.angular_velocity_body;
  const Eigen::Vector3d desired_angular_acceleration_body = current_to_desired_body * feedforward.angular_acceleration_body;

  const Eigen::Vector3d attitude_error = geometricAttitudeError(q, q_d);
  const Eigen::Vector3d tanh_attitude_error = tanh_feedback(attitude_error, att_gains_.K_Angle, Eigen::Vector3d::Ones());
  const Eigen::Vector3d angular_velocity_error_body = (state.angular_velocity_body - desired_angular_velocity_body) + att_gains_.M_Angle.cwiseProduct(tanh_attitude_error);
  const Eigen::Vector3d angular_velocity_estimation_error_body = angular_velocity_error_body - angular_velocity_error_hat_body_;
  const Eigen::Vector3d angular_velocity_disturbance_raw = tanh_feedback(angular_velocity_estimation_error_body, att_gains_.L_AngularVelocity, att_gains_.P_AngularVelocity);
  const Eigen::Vector3d angular_velocity_disturbance_filtered = update_low_pass(angular_velocity_disturbance_raw, dt, angular_velocity_disturbance_lpf_);
  const Eigen::Vector3d omega_body = state.angular_velocity_body;
  const Eigen::Vector3d omega_cross_inertia_omega = omega_body.cross(inertia_ * omega_body);
  const Eigen::Matrix3d inertia_inv = inertia_.inverse();
  const Eigen::Vector3d geometric_feedforward_body =
      omega_body.cross(desired_angular_velocity_body) - desired_angular_acceleration_body;
  const Eigen::Vector3d angular_velocity_error_derivative_hat_body =
      state.angular_acceleration_body + geometric_feedforward_body;
  const Eigen::Vector3d tanh_angular_velocity_error = tanh_feedback(angular_velocity_error_body, att_gains_.K_AngularVelocity, Eigen::Vector3d::Ones());
  const Eigen::Vector3d angular_velocity_control_term = att_gains_.M_AngularVelocity.cwiseProduct(tanh_angular_velocity_error);
  const Eigen::Vector3d angular_acceleration_feedback =
      inertia_ * att_gains_.K_AngularAcceleration.cwiseProduct(angular_velocity_error_derivative_hat_body);
  const auto desiredTorqueFromDisturbance = [&](const Eigen::Vector3d& angular_velocity_disturbance) {
    return omega_cross_inertia_omega - inertia_ * angular_velocity_disturbance -
           inertia_ * geometric_feedforward_body - inertia_ * angular_velocity_control_term -
           angular_acceleration_feedback;
  };

  const Eigen::Vector3d desired_torque_control = desiredTorqueFromDisturbance(angular_velocity_disturbance_filtered);
  const Eigen::Vector3d desired_torque_observer = desiredTorqueFromDisturbance(angular_velocity_disturbance_raw);

  const Eigen::Vector3d angular_velocity_error_hat_dot_body =
      (-inertia_inv * omega_cross_inertia_omega) + inertia_inv * desired_torque_observer +
      geometric_feedforward_body + angular_velocity_disturbance_raw;
  angular_velocity_error_hat_body_ += dt * angular_velocity_error_hat_dot_body;

  if (torque_body) {
    *torque_body = desired_torque_control;
  }
}

}  // namespace tanh_ctrl
