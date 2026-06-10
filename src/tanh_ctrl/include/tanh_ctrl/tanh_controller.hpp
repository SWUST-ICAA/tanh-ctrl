#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>

namespace tanh_ctrl {

struct VehicleState {
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration_ned{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_body_to_ned{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration_body{Eigen::Vector3d::Zero()};
};

struct TrajectoryRef {
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d snap_ned{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double yaw_rate{0.0};
  double yaw_acceleration{0.0};
  bool has_flatness_feedforward{false};
  bool valid{false};
};

struct PositionGains {
  Eigen::Vector3d M_P{Eigen::Vector3d::Ones()};
  Eigen::Vector3d K_P{Eigen::Vector3d::Ones()};
  Eigen::Vector3d M_V{Eigen::Vector3d::Ones()};
  Eigen::Vector3d K_V{Eigen::Vector3d::Ones()};
  Eigen::Vector3d K_Acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d P_V{Eigen::Vector3d::Zero()};
  Eigen::Vector3d L_V{Eigen::Vector3d::Ones()};
};

struct AttitudeGains {
  Eigen::Vector3d M_Angle{Eigen::Vector3d::Ones()};
  Eigen::Vector3d K_Angle{Eigen::Vector3d::Ones()};
  Eigen::Vector3d M_AngularVelocity{Eigen::Vector3d::Ones()};
  Eigen::Vector3d K_AngularVelocity{Eigen::Vector3d::Ones()};
  Eigen::Vector3d K_AngularAcceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d P_AngularVelocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d L_AngularVelocity{Eigen::Vector3d::Ones()};
};

struct AttitudeReference {
  Eigen::Quaterniond attitude_body_to_ned{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d jerk_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d snap_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d thrust_direction_ned{Eigen::Vector3d::UnitZ()};
  double collective_thrust{0.0};
  double yaw_rate{0.0};
  double yaw_acceleration{0.0};
  bool has_flatness_feedforward{false};
  bool valid{false};
};

struct ControlOutput {
  double thrust_total{0.0};
  Eigen::Vector3d torque_body{Eigen::Vector3d::Zero()};
};

/**
 * @brief Tanh feedback controller for quadrotor position and attitude
 * stabilization.
 */
class TanhController {
public:
  TanhController();

  void setMass(double mass);
  void setGravity(double gravity);
  void setPositionGains(const PositionGains &gains);
  void setAttitudeGains(const AttitudeGains &gains);
  void setInertia(const Eigen::Matrix3d &inertia);

  void setMaxTilt(double max_tilt_rad);
  void setVelocityDisturbanceLowPassHz(double cutoff_hz);
  void setAngularVelocityDisturbanceLowPassHz(double cutoff_hz);

  void reset();

  bool computePositionLoop(const VehicleState &state, const TrajectoryRef &ref,
                           double dt, AttitudeReference *attitude_reference);
  bool computeAttitudeLoop(const VehicleState &state,
                           const AttitudeReference &attitude_reference,
                           double dt, ControlOutput *out);

private:
  struct Vec3ButterworthLowPass {
    Eigen::Vector3d cutoff_hz{Eigen::Vector3d::Zero()};
    Eigen::Vector3d input_1{Eigen::Vector3d::Zero()};
    Eigen::Vector3d input_2{Eigen::Vector3d::Zero()};
    Eigen::Vector3d output_1{Eigen::Vector3d::Zero()};
    Eigen::Vector3d output_2{Eigen::Vector3d::Zero()};
    std::array<bool, 3> initialized{{false, false, false}};
  };

  static void resetButterworthLowPass(Vec3ButterworthLowPass &lpf);
  static Eigen::Vector3d updateButterworthLowPass(const Eigen::Vector3d &input,
                                                  double dt,
                                                  Vec3ButterworthLowPass &lpf);

  void computePosition(const VehicleState &state, const TrajectoryRef &ref,
                       double dt, Eigen::Vector3d *thrust_vec_ned);
  void computeAttitude(const VehicleState &state,
                       const AttitudeReference &attitude_reference, double dt,
                       Eigen::Vector3d *torque_body);
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

  Vec3ButterworthLowPass velocity_disturbance_lpf_{};
  Vec3ButterworthLowPass angular_velocity_disturbance_lpf_{};
};

} // namespace tanh_ctrl
