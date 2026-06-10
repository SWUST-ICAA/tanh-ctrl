#include "tanh_ctrl/common.hpp"

#include <algorithm>
#include <cmath>
#include <sophus/so3.hpp>

namespace tanh_ctrl {

namespace {

constexpr double kMinPositiveZ = 1e-3;
constexpr double kSmallNorm = 1e-9;
constexpr double kMinCollectiveThrust = 1e-6;
constexpr double kMinLoopDt = 1e-4;
constexpr double kMaxLoopDt = 0.1;

}  // namespace

Eigen::Vector3d planarAxisVec(double planar, double axial) {
  return Eigen::Vector3d(planar, planar, axial);
}

uint64_t selectMessageTimestampUs(uint64_t timestamp_sample_us, uint64_t timestamp_us, uint64_t fallback_us) {
  if (timestamp_sample_us != 0) {
    return timestamp_sample_us;
  }
  if (timestamp_us != 0) {
    return timestamp_us;
  }
  return fallback_us;
}

double computeLoopDtFromSample(uint64_t sample_timestamp_us, uint64_t* last_sample_timestamp_us) {
  if (!last_sample_timestamp_us || sample_timestamp_us == 0) {
    return kMinLoopDt;
  }

  if (*last_sample_timestamp_us == 0 || sample_timestamp_us <= *last_sample_timestamp_us) {
    *last_sample_timestamp_us = sample_timestamp_us;
    return kMinLoopDt;
  }

  const double dt = static_cast<double>(sample_timestamp_us - *last_sample_timestamp_us) * 1e-6;
  *last_sample_timestamp_us = sample_timestamp_us;
  return std::clamp(dt, kMinLoopDt, kMaxLoopDt);
}

void applyTiltLimit(Eigen::Vector3d* thrust_vector_over_mass_ned, double max_tilt_rad) {
  if (!thrust_vector_over_mass_ned || max_tilt_rad <= 0.0) {
    return;
  }

  thrust_vector_over_mass_ned->z() = std::max(thrust_vector_over_mass_ned->z(), kMinPositiveZ);
  const double horizontal_norm = std::hypot(thrust_vector_over_mass_ned->x(), thrust_vector_over_mass_ned->y());
  const double max_horizontal = thrust_vector_over_mass_ned->z() * std::tan(max_tilt_rad);

  if (horizontal_norm <= max_horizontal || horizontal_norm <= kSmallNorm) {
    return;
  }

  const double scale = max_horizontal / horizontal_norm;
  thrust_vector_over_mass_ned->x() *= scale;
  thrust_vector_over_mass_ned->y() *= scale;
}

Eigen::Quaterniond computeDesiredAttitude(const Eigen::Vector3d& thrust_direction_ned, double yaw) {
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

Eigen::Vector3d geometricAttitudeError(const Eigen::Quaterniond& current_body_to_ned, const Eigen::Quaterniond& desired_body_to_ned) {
  const Sophus::SO3d current_so3(current_body_to_ned.normalized());
  const Sophus::SO3d desired_so3(desired_body_to_ned.normalized());
  const Eigen::Matrix3d current_rotation = current_so3.matrix();
  const Eigen::Matrix3d desired_rotation = desired_so3.matrix();
  const Eigen::Matrix3d error_matrix =
      desired_rotation.transpose() * current_rotation - current_rotation.transpose() * desired_rotation;
  return 0.5 * Sophus::SO3d::vee(error_matrix);
}

AttitudeReference computeAttitudeReference(const Eigen::Vector3d& desired_thrust_vector_ned, const TrajectoryRef& ref) {
  AttitudeReference attitude_reference;

  const Eigen::Vector3d thrust_vector_ned = desired_thrust_vector_ned;
  const double collective_thrust = thrust_vector_ned.norm();
  if (collective_thrust <= kMinCollectiveThrust) {
    attitude_reference.collective_thrust = 0.0;
    attitude_reference.valid = false;
    return attitude_reference;
  }

  attitude_reference.collective_thrust = collective_thrust;
  attitude_reference.thrust_direction_ned = thrust_vector_ned / collective_thrust;
  attitude_reference.attitude_body_to_ned = computeDesiredAttitude(attitude_reference.thrust_direction_ned, ref.yaw);
  attitude_reference.jerk_ned = ref.jerk_ned;
  attitude_reference.snap_ned = ref.snap_ned;
  attitude_reference.yaw_rate = ref.yaw_rate;
  attitude_reference.yaw_acceleration = ref.yaw_acceleration;
  attitude_reference.has_flatness_feedforward = ref.has_flatness_feedforward;
  attitude_reference.valid = true;

  return attitude_reference;
}

}  // namespace tanh_ctrl
