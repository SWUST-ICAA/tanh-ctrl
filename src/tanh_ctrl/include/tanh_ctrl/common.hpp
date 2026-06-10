#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "tanh_ctrl/types.hpp"

namespace tanh_ctrl {

/**
 * @brief Build a 3-axis vector with shared planar gains and an independent yaw/vertical axis.
 */
Eigen::Vector3d planarAxisVec(double planar, double axial);

/**
 * @brief Select the best available PX4 timestamp in microseconds.
 */
uint64_t selectMessageTimestampUs(uint64_t timestamp_sample_us, uint64_t timestamp_us, uint64_t fallback_us);

/**
 * @brief Compute a loop dt from successive PX4 sample timestamps.
 */
double computeLoopDtFromSample(uint64_t sample_timestamp_us, uint64_t* last_sample_timestamp_us);

/**
 * @brief Limit horizontal thrust so the thrust vector respects a tilt bound.
 */
void applyTiltLimit(Eigen::Vector3d* thrust_vector_over_mass_ned, double max_tilt_rad);

/**
 * @brief Reconstruct the desired body attitude from thrust direction and yaw.
 */
Eigen::Quaterniond computeDesiredAttitude(const Eigen::Vector3d& thrust_direction_ned, double yaw);

/**
 * @brief Paper SO(3) attitude error: 0.5 * (R_d^T R - R^T R_d)^vee.
 */
Eigen::Vector3d geometricAttitudeError(const Eigen::Quaterniond& current_body_to_ned, const Eigen::Quaterniond& desired_body_to_ned);

/**
 * @brief Build the inner-loop attitude reference from thrust and trajectory feedforward.
 */
AttitudeReference computeAttitudeReference(const Eigen::Vector3d& desired_thrust_vector_ned, const TrajectoryRef& ref);

}  // namespace tanh_ctrl
