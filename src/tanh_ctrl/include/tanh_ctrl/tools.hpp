#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>

#include "tanh_ctrl/common.hpp"

namespace tanh_ctrl {

inline uint64_t nowMicros(rclcpp::Clock& clock) {
  return static_cast<uint64_t>(clock.now().nanoseconds() / 1000ULL);
}

inline double elapsedSeconds(uint64_t now_us, uint64_t previous_us) {
  if (previous_us == 0 || now_us <= previous_us) {
    return 0.0;
  }
  return static_cast<double>(now_us - previous_us) * 1e-6;
}

inline double quaternionToYaw(const Eigen::Quaterniond& q_body_to_ned) {
  const Eigen::Quaterniond q = q_body_to_ned.normalized();
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

template <typename Array3T>
Eigen::Vector3d eigenFromArray3(const Array3T& values) {
  return Eigen::Vector3d(static_cast<double>(values[0]), static_cast<double>(values[1]), static_cast<double>(values[2]));
}

template <typename MsgVec3T>
Eigen::Vector3d eigenFromXyz(const MsgVec3T& values) {
  return Eigen::Vector3d(static_cast<double>(values.x), static_cast<double>(values.y), static_cast<double>(values.z));
}

inline bool requestDue(uint64_t now_us, uint64_t last_request_us, double interval_s) {
  return last_request_us == 0 || elapsedSeconds(now_us, last_request_us) >= interval_s;
}

inline void declareAxisPair(rclcpp::Node& node, const char* shared_name, double shared_default, const char* axial_name, double axial_default) {
  node.declare_parameter<double>(shared_name, shared_default);
  node.declare_parameter<double>(axial_name, axial_default);
}

inline Eigen::Vector3d loadAxisPairParam(rclcpp::Node& node, const char* shared_name, const char* axial_name) {
  return planarAxisVec(node.get_parameter(shared_name).as_double(), node.get_parameter(axial_name).as_double());
}

}  // namespace tanh_ctrl
