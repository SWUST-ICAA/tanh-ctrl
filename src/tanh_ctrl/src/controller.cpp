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
constexpr double kMinPositiveZ = 1e-3;
constexpr double kSmallNorm = 1e-9;
constexpr double kMinCollectiveThrust = 1e-6;
constexpr double kMinThrustOverMass = 1e-3;
constexpr double kCutoffChangeToleranceHz = 1e-9;
constexpr double kButterworthSqrt2 = 1.41421356237309504880;
constexpr double kMaxNormalizedCutoff = 0.45;

/************ math tools ***************/

double clampPositive(double value, double min_value) {
  return std::max(min_value, value);
}

Eigen::Vector3d tanhFeedback(const Eigen::Vector3d &error,
                             const Eigen::Vector3d &slope,
                             const Eigen::Vector3d &scale) {
  return scale.cwiseProduct(
      (slope.cwiseProduct(error)).array().tanh().matrix());
}

void applyTiltLimit(Eigen::Vector3d *thrust_vector_over_mass_ned,
                    double max_tilt_rad) {
  if (!thrust_vector_over_mass_ned || max_tilt_rad <= 0.0) {
    return;
  }

  thrust_vector_over_mass_ned->z() =
      std::max(thrust_vector_over_mass_ned->z(), kMinPositiveZ);
  const double horizontal_norm = std::hypot(thrust_vector_over_mass_ned->x(),
                                            thrust_vector_over_mass_ned->y());
  const double max_horizontal =
      thrust_vector_over_mass_ned->z() * std::tan(max_tilt_rad);

  if (horizontal_norm <= max_horizontal || horizontal_norm <= kSmallNorm) {
    return;
  }

  const double scale = max_horizontal / horizontal_norm;
  thrust_vector_over_mass_ned->x() *= scale;
  thrust_vector_over_mass_ned->y() *= scale;
}

Eigen::Quaterniond
computeDesiredAttitude(const Eigen::Vector3d &thrust_direction_ned,
                       double yaw) {
  Eigen::Vector3d z_body_ned = thrust_direction_ned;
  if (z_body_ned.norm() <= kMinCollectiveThrust) {
    z_body_ned = Eigen::Vector3d::UnitZ();
  } else {
    z_body_ned.normalize();
  }

  Eigen::Vector3d x_course_ned(std::cos(yaw), std::sin(yaw), 0.0);
  Eigen::Vector3d y_body_ned = z_body_ned.cross(x_course_ned);
  if (y_body_ned.norm() <= kMinCollectiveThrust) {
    x_course_ned = Eigen::Vector3d::UnitX();
    y_body_ned = z_body_ned.cross(x_course_ned);
  }
  y_body_ned.normalize();

  Eigen::Vector3d x_body_ned = y_body_ned.cross(z_body_ned);
  x_body_ned.normalize();

  Eigen::Matrix3d desired_rotation;
  desired_rotation.col(0) = x_body_ned;
  desired_rotation.col(1) = y_body_ned;
  desired_rotation.col(2) = z_body_ned;

  const Sophus::SO3d desired_so3(desired_rotation);
  Eigen::Quaterniond desired_attitude = desired_so3.unit_quaternion();
  desired_attitude.normalize();
  return desired_attitude;
}

Eigen::Vector3d
geometricAttitudeError(const Eigen::Quaterniond &current_body_to_ned,
                       const Eigen::Quaterniond &desired_body_to_ned) {
  const Sophus::SO3d current_so3(current_body_to_ned.normalized());
  const Sophus::SO3d desired_so3(desired_body_to_ned.normalized());
  const Eigen::Matrix3d current_rotation = current_so3.matrix();
  const Eigen::Matrix3d desired_rotation = desired_so3.matrix();
  const Eigen::Matrix3d error_matrix =
      desired_rotation.transpose() * current_rotation -
      current_rotation.transpose() * desired_rotation;
  return 0.5 * Sophus::SO3d::vee(error_matrix);
}

AttitudeReference
computeAttitudeReference(const Eigen::Vector3d &desired_thrust_vector_ned,
                         const TrajectoryRef &ref) {
  AttitudeReference attitude_reference;

  const Eigen::Vector3d thrust_vector_ned = desired_thrust_vector_ned;
  const double collective_thrust = thrust_vector_ned.norm();
  if (collective_thrust <= kMinCollectiveThrust) {
    attitude_reference.collective_thrust = 0.0;
    attitude_reference.valid = false;
    return attitude_reference;
  }

  attitude_reference.collective_thrust = collective_thrust;
  attitude_reference.thrust_direction_ned =
      thrust_vector_ned / collective_thrust;
  attitude_reference.attitude_body_to_ned =
      computeDesiredAttitude(attitude_reference.thrust_direction_ned, ref.yaw);
  attitude_reference.jerk_ned = ref.jerk_ned;
  attitude_reference.snap_ned = ref.snap_ned;
  attitude_reference.yaw_rate = ref.yaw_rate;
  attitude_reference.yaw_acceleration = ref.yaw_acceleration;
  attitude_reference.has_flatness_feedforward = ref.has_flatness_feedforward;
  attitude_reference.valid = true;

  return attitude_reference;
}

/***************************************/

/************ flatness feedforward ***************/

struct FlatnessAttitudeFeedforward {
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration_body{Eigen::Vector3d::Zero()};
};

FlatnessAttitudeFeedforward
computeFlatnessAttitudeFeedforward(const AttitudeReference &attitude_reference,
                                   double mass) {
  FlatnessAttitudeFeedforward feedforward;

  const double thrust_over_mass =
      attitude_reference.collective_thrust / clampPositive(mass, kMinMass);
  if (!attitude_reference.has_flatness_feedforward ||
      thrust_over_mass <= kMinThrustOverMass) {
    return feedforward;
  }

  const Sophus::SO3d desired_body_to_ned_so3(
      attitude_reference.attitude_body_to_ned.normalized());
  const Eigen::Matrix3d desired_body_to_ned = desired_body_to_ned_so3.matrix();
  const Eigen::Vector3d x_body_ned = desired_body_to_ned.col(0);
  const Eigen::Vector3d y_body_ned = desired_body_to_ned.col(1);
  const Eigen::Vector3d z_body_ned = desired_body_to_ned.col(2);
  const Eigen::Vector3d z_world_ned = Eigen::Vector3d::UnitZ();

  const double thrust_over_mass_dot =
      -attitude_reference.jerk_ned.dot(z_body_ned);
  const Eigen::Vector3d reference_h_omega_ned =
      -(attitude_reference.jerk_ned + thrust_over_mass_dot * z_body_ned) /
      thrust_over_mass;

  feedforward.angular_velocity_body.x() =
      -reference_h_omega_ned.dot(y_body_ned);
  feedforward.angular_velocity_body.y() = reference_h_omega_ned.dot(x_body_ned);
  feedforward.angular_velocity_body.z() =
      attitude_reference.yaw_rate * z_world_ned.dot(z_body_ned);

  const Eigen::Vector3d omega_ned =
      desired_body_to_ned * feedforward.angular_velocity_body;
  const double thrust_over_mass_ddot =
      -attitude_reference.snap_ned.dot(z_body_ned) -
      reference_h_omega_ned.dot(attitude_reference.jerk_ned);
  const Eigen::Vector3d omega_cross_h_omega_ned =
      omega_ned.cross(reference_h_omega_ned);
  const Eigen::Vector3d reference_h_alpha_ned =
      -attitude_reference.snap_ned / thrust_over_mass -
      omega_cross_h_omega_ned -
      2.0 * (thrust_over_mass_dot / thrust_over_mass) * reference_h_omega_ned -
      (thrust_over_mass_ddot / thrust_over_mass) * z_body_ned;

  feedforward.angular_acceleration_body.x() =
      -reference_h_alpha_ned.dot(y_body_ned);
  feedforward.angular_acceleration_body.y() =
      reference_h_alpha_ned.dot(x_body_ned);
  feedforward.angular_acceleration_body.z() =
      attitude_reference.yaw_acceleration * z_world_ned.dot(z_body_ned);

  return feedforward;
}

/***************************************/

} // namespace

/************ controller lifecycle ***************/

TanhController::TanhController() = default;

void TanhController::setMass(double mass) {
  mass_ = clampPositive(mass, kMinMass);
}

void TanhController::setGravity(double gravity) {
  gravity_ = std::max(0.0, gravity);
}

void TanhController::setPositionGains(const PositionGains &gains) {
  pos_gains_ = gains;
}

void TanhController::setAttitudeGains(const AttitudeGains &gains) {
  att_gains_ = gains;
}

void TanhController::setInertia(const Eigen::Matrix3d &inertia) {
  inertia_ = inertia;
}

void TanhController::setMaxTilt(double max_tilt_rad) {
  max_tilt_rad_ = max_tilt_rad <= 0.0 ? 0.0
                                      : std::clamp(max_tilt_rad, kMinTiltRad,
                                                   M_PI_2 - kMinTiltRad);
}

void TanhController::setVelocityDisturbanceLowPassHz(double cutoff_hz) {
  const Eigen::Vector3d cutoff = Eigen::Vector3d::Constant(cutoff_hz);
  if ((velocity_disturbance_lpf_.cutoff_hz - cutoff).cwiseAbs().maxCoeff() <=
      kCutoffChangeToleranceHz) {
    return;
  }

  velocity_disturbance_lpf_.cutoff_hz = cutoff;
  resetButterworthLowPass(velocity_disturbance_lpf_);
}

void TanhController::setAngularVelocityDisturbanceLowPassHz(double cutoff_hz) {
  const Eigen::Vector3d cutoff = Eigen::Vector3d::Constant(cutoff_hz);
  if ((angular_velocity_disturbance_lpf_.cutoff_hz - cutoff)
          .cwiseAbs()
          .maxCoeff() <= kCutoffChangeToleranceHz) {
    return;
  }

  angular_velocity_disturbance_lpf_.cutoff_hz = cutoff;
  resetButterworthLowPass(angular_velocity_disturbance_lpf_);
}

void TanhController::reset() {
  velocity_error_hat_ned_.setZero();
  angular_velocity_error_hat_body_.setZero();
  first_run_ = true;

  resetButterworthLowPass(velocity_disturbance_lpf_);
  resetButterworthLowPass(angular_velocity_disturbance_lpf_);
}

/***************************************/

/************ filters ***************/

void TanhController::resetButterworthLowPass(Vec3ButterworthLowPass &lpf) {
  lpf.input_1.setZero();
  lpf.input_2.setZero();
  lpf.output_1.setZero();
  lpf.output_2.setZero();
  lpf.initialized = {{false, false, false}};
}

Eigen::Vector3d TanhController::updateButterworthLowPass(
    const Eigen::Vector3d &input, double dt, Vec3ButterworthLowPass &lpf) {
  const auto updateScalarButterworth = [](double input_value, double cutoff_hz,
                                          double dt_s, double *input_1,
                                          double *input_2, double *output_1,
                                          double *output_2, bool *initialized) {
    if (!input_1 || !input_2 || !output_1 || !output_2 || !initialized ||
        dt_s <= 0.0 || cutoff_hz <= 0.0) {
      return input_value;
    }

    if (!(*initialized)) {
      *input_1 = input_value;
      *input_2 = input_value;
      *output_1 = input_value;
      *output_2 = input_value;
      *initialized = true;
      return input_value;
    }

    const double normalized_cutoff =
        std::clamp(cutoff_hz * dt_s, 1.0e-6, kMaxNormalizedCutoff);
    const double k = std::tan(M_PI * normalized_cutoff);
    const double k2 = k * k;
    const double norm = 1.0 / (1.0 + kButterworthSqrt2 * k + k2);
    const double b0 = k2 * norm;
    const double b1 = 2.0 * b0;
    const double b2 = b0;
    const double a1 = 2.0 * (k2 - 1.0) * norm;
    const double a2 = (1.0 - kButterworthSqrt2 * k + k2) * norm;

    const double output = b0 * input_value + b1 * (*input_1) + b2 * (*input_2) -
                          a1 * (*output_1) - a2 * (*output_2);
    *input_2 = *input_1;
    *input_1 = input_value;
    *output_2 = *output_1;
    *output_1 = output;
    return output;
  };

  Eigen::Vector3d output;
  output.x() = updateScalarButterworth(
      input.x(), lpf.cutoff_hz.x(), dt, &lpf.input_1.x(), &lpf.input_2.x(),
      &lpf.output_1.x(), &lpf.output_2.x(), &lpf.initialized[0]);
  output.y() = updateScalarButterworth(
      input.y(), lpf.cutoff_hz.y(), dt, &lpf.input_1.y(), &lpf.input_2.y(),
      &lpf.output_1.y(), &lpf.output_2.y(), &lpf.initialized[1]);
  output.z() = updateScalarButterworth(
      input.z(), lpf.cutoff_hz.z(), dt, &lpf.input_1.z(), &lpf.input_2.z(),
      &lpf.output_1.z(), &lpf.output_2.z(), &lpf.initialized[2]);
  return output;
}

/***************************************/

/************ control loops ***************/

bool TanhController::computePositionLoop(
    const VehicleState &state, const TrajectoryRef &ref, double dt,
    AttitudeReference *attitude_reference) {
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

bool TanhController::computeAttitudeLoop(
    const VehicleState &state, const AttitudeReference &attitude_reference,
    double dt, ControlOutput *out) {
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

void TanhController::computePosition(const VehicleState &state,
                                     const TrajectoryRef &ref, double dt,
                                     Eigen::Vector3d *thrust_vec_ned) {
  const Eigen::Vector3d position_error_ned =
      state.position_ned - ref.position_ned;
  const Eigen::Vector3d tanh_position_error =
      tanhFeedback(position_error_ned, pos_gains_.K_P, Eigen::Vector3d::Ones());
  const Eigen::Vector3d velocity_error_ned =
      (state.velocity_ned - ref.velocity_ned) +
      pos_gains_.M_P.cwiseProduct(tanh_position_error);
  const Eigen::Vector3d velocity_estimation_error_ned =
      velocity_error_ned - velocity_error_hat_ned_;
  const Eigen::Vector3d velocity_disturbance_raw = tanhFeedback(
      velocity_estimation_error_ned, pos_gains_.L_V, pos_gains_.P_V);
  const Eigen::Vector3d velocity_disturbance_filtered =
      updateButterworthLowPass(velocity_disturbance_raw, dt,
                               velocity_disturbance_lpf_);
  const Eigen::Vector3d gravity_ned(0.0, 0.0, gravity_);
  const Eigen::Vector3d tanh_velocity_error =
      tanhFeedback(velocity_error_ned, pos_gains_.K_V, Eigen::Vector3d::Ones());
  const Eigen::Vector3d velocity_feedback =
      pos_gains_.M_V.cwiseProduct(tanh_velocity_error);
  const Eigen::Vector3d acceleration_feedback =
      pos_gains_.K_Acceleration.cwiseProduct(state.linear_acceleration_ned -
                                             ref.acceleration_ned);

  const auto thrustOverMassFromDisturbance =
      [&](const Eigen::Vector3d &disturbance) {
        return disturbance + gravity_ned + velocity_feedback +
               acceleration_feedback - ref.acceleration_ned;
      };

  Eigen::Vector3d thrust_over_mass_control =
      thrustOverMassFromDisturbance(velocity_disturbance_filtered);
  Eigen::Vector3d thrust_over_mass_observer =
      thrustOverMassFromDisturbance(velocity_disturbance_raw);

  applyTiltLimit(&thrust_over_mass_control, max_tilt_rad_);
  applyTiltLimit(&thrust_over_mass_observer, max_tilt_rad_);

  const Eigen::Vector3d desired_thrust_control =
      mass_ * thrust_over_mass_control;
  const Eigen::Vector3d desired_thrust_observer =
      mass_ * thrust_over_mass_observer;

  const Eigen::Vector3d velocity_error_hat_dot_ned =
      (-desired_thrust_observer / mass_) + gravity_ned +
      velocity_disturbance_raw - ref.acceleration_ned;
  velocity_error_hat_ned_ += dt * velocity_error_hat_dot_ned;

  if (thrust_vec_ned) {
    *thrust_vec_ned = desired_thrust_control;
  }
}

void TanhController::computeAttitude(
    const VehicleState &state, const AttitudeReference &attitude_reference,
    double dt, Eigen::Vector3d *torque_body) {
  const Eigen::Quaterniond q = state.q_body_to_ned.normalized();
  const Eigen::Quaterniond q_d =
      attitude_reference.attitude_body_to_ned.normalized();
  const Sophus::SO3d current_so3(q);
  const Sophus::SO3d desired_so3(q_d);
  const Eigen::Matrix3d current_to_desired_body =
      current_so3.matrix().transpose() * desired_so3.matrix();

  const FlatnessAttitudeFeedforward feedforward =
      computeFlatnessAttitudeFeedforward(attitude_reference, mass_);
  const Eigen::Vector3d desired_angular_velocity_body =
      current_to_desired_body * feedforward.angular_velocity_body;
  const Eigen::Vector3d desired_angular_acceleration_body =
      current_to_desired_body * feedforward.angular_acceleration_body;

  const Eigen::Vector3d attitude_error = geometricAttitudeError(q, q_d);
  const Eigen::Vector3d tanh_attitude_error =
      tanhFeedback(attitude_error, att_gains_.K_Angle, Eigen::Vector3d::Ones());
  const Eigen::Vector3d angular_velocity_error_body =
      (state.angular_velocity_body - desired_angular_velocity_body) +
      att_gains_.M_Angle.cwiseProduct(tanh_attitude_error);
  const Eigen::Vector3d angular_velocity_estimation_error_body =
      angular_velocity_error_body - angular_velocity_error_hat_body_;
  const Eigen::Vector3d angular_velocity_disturbance_raw =
      tanhFeedback(angular_velocity_estimation_error_body,
                   att_gains_.L_AngularVelocity, att_gains_.P_AngularVelocity);
  const Eigen::Vector3d angular_velocity_disturbance_filtered =
      updateButterworthLowPass(angular_velocity_disturbance_raw, dt,
                               angular_velocity_disturbance_lpf_);
  const Eigen::Vector3d omega_body = state.angular_velocity_body;
  const Eigen::Vector3d omega_cross_inertia_omega =
      omega_body.cross(inertia_ * omega_body);
  const Eigen::Matrix3d inertia_inv = inertia_.inverse();
  const Eigen::Vector3d geometric_feedforward_body =
      omega_body.cross(desired_angular_velocity_body) -
      desired_angular_acceleration_body;
  const Eigen::Vector3d angular_velocity_error_derivative_hat_body =
      state.angular_acceleration_body + geometric_feedforward_body;
  const Eigen::Vector3d tanh_angular_velocity_error =
      tanhFeedback(angular_velocity_error_body, att_gains_.K_AngularVelocity,
                   Eigen::Vector3d::Ones());
  const Eigen::Vector3d angular_velocity_control_term =
      att_gains_.M_AngularVelocity.cwiseProduct(tanh_angular_velocity_error);
  const Eigen::Vector3d angular_acceleration_feedback =
      inertia_ * att_gains_.K_AngularAcceleration.cwiseProduct(
                     angular_velocity_error_derivative_hat_body);
  const auto desiredTorqueFromDisturbance =
      [&](const Eigen::Vector3d &angular_velocity_disturbance) {
        return omega_cross_inertia_omega -
               inertia_ * angular_velocity_disturbance -
               inertia_ * geometric_feedforward_body -
               inertia_ * angular_velocity_control_term -
               angular_acceleration_feedback;
      };

  const Eigen::Vector3d desired_torque_control =
      desiredTorqueFromDisturbance(angular_velocity_disturbance_filtered);
  const Eigen::Vector3d desired_torque_observer =
      desiredTorqueFromDisturbance(angular_velocity_disturbance_raw);

  const Eigen::Vector3d angular_velocity_error_hat_dot_body =
      (-inertia_inv * omega_cross_inertia_omega) +
      inertia_inv * desired_torque_observer + geometric_feedforward_body +
      angular_velocity_disturbance_raw;
  angular_velocity_error_hat_body_ += dt * angular_velocity_error_hat_dot_body;

  if (torque_body) {
    *torque_body = desired_torque_control;
  }
}

/***************************************/

} // namespace tanh_ctrl
