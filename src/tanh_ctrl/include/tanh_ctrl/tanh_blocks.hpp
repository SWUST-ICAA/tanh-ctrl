#pragma once

#include <Eigen/Dense>
#include <array>

namespace tanh_ctrl {

struct Vec3LowPass {
  Eigen::Vector3d cutoff_hz{Eigen::Vector3d::Zero()};
  Eigen::Vector3d state{Eigen::Vector3d::Zero()};
  std::array<bool, 3> initialized{{false, false, false}};
};

Eigen::Vector3d tanh_feedback(const Eigen::Vector3d& error, const Eigen::Vector3d& slope, const Eigen::Vector3d& scale);

void reset_low_pass(Vec3LowPass& lpf);
Eigen::Vector3d update_low_pass(const Eigen::Vector3d& input, double dt, Vec3LowPass& lpf);

}  // namespace tanh_ctrl
