#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <flat_trajectory_msgs/msg/flat_trajectory_reference.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::array<double, 3> kFigure8Harmonics{1.0, 2.0, 1.0};
constexpr std::array<double, 3> kFigure8PhasesRad{0.0, 0.0, 0.0};

std::array<double, 3> loadVector3(rclcpp::Node &node, const std::string &name,
                                  const std::array<double, 3> &fallback) {
  const auto values = node.get_parameter(name).as_double_array();
  if (values.size() != 3) {
    RCLCPP_WARN(node.get_logger(), "%s must have length 3; using fallback.",
                name.c_str());
    return fallback;
  }
  return {values[0], values[1], values[2]};
}

double clampPositive(double value, double fallback) {
  return value > 0.0 ? value : fallback;
}

struct StartupBlend {
  double value{1.0};
  double dot{0.0};
  double ddot{0.0};
  double dddot{0.0};
  double ddddot{0.0};
};

StartupBlend smoothStartupBlend(double t, double smoothing_s) {
  if (smoothing_s <= 0.0 || t >= smoothing_s) {
    return {1.0, 0.0, 0.0, 0.0, 0.0};
  }

  const double r = smoothing_s;
  const double u = std::clamp(t / r, 0.0, 1.0);
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  const double u6 = u5 * u;
  const double u7 = u6 * u;
  const double u8 = u7 * u;
  const double u9 = u8 * u;

  const double s = 70.0 * u5 - 224.0 * u6 + 280.0 * u7 - 160.0 * u8 + 35.0 * u9;
  const double ds =
      350.0 * u4 - 1344.0 * u5 + 1960.0 * u6 - 1280.0 * u7 + 315.0 * u8;
  const double dds =
      1400.0 * u3 - 6720.0 * u4 + 11760.0 * u5 - 8960.0 * u6 + 2520.0 * u7;
  const double ddds =
      4200.0 * u2 - 26880.0 * u3 + 58800.0 * u4 - 53760.0 * u5 + 17640.0 * u6;
  const double dddds =
      8400.0 * u - 80640.0 * u2 + 235200.0 * u3 - 268800.0 * u4 + 105840.0 * u5;

  return {s, ds / r, dds / (r * r), ddds / (r * r * r),
          dddds / (r * r * r * r)};
}

} // namespace

class LissajousTrajectoryPublisher : public rclcpp::Node {
public:
  LissajousTrajectoryPublisher() : Node("lissajous_traj_pub") {
    declareParameters();
    loadParameters();
    createRosInterfaces();
  }

private:
  void declareParameters() {
    this->declare_parameter<std::string>("topics.reference",
                                         "/tanh_ctrl/reference");
    this->declare_parameter<std::string>("topics.start_tracking",
                                         "/mission/start_tracking");
    this->declare_parameter<std::string>("frame_id", "ned");
    this->declare_parameter<bool>("require_start_signal", true);
    this->declare_parameter<bool>("start_enabled", false);
    this->declare_parameter<double>("rate_hz", 100.0);
    this->declare_parameter<std::vector<double>>("origin_ned",
                                                 {0.0, 0.0, -1.0});
    this->declare_parameter<double>("amplitude_x_m", 0.8);
    this->declare_parameter<double>("amplitude_y_m", 0.5);
    this->declare_parameter<double>("period_s", 12.0);
    this->declare_parameter<double>("startup_smoothing_s", 2.0);
    this->declare_parameter<double>("yaw_rad", 0.0);
  }

  void loadParameters() {
    reference_topic_ = this->get_parameter("topics.reference").as_string();
    start_tracking_topic_ =
        this->get_parameter("topics.start_tracking").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    require_start_signal_ =
        this->get_parameter("require_start_signal").as_bool();
    enabled_ = this->get_parameter("start_enabled").as_bool();
    rate_hz_ = clampPositive(this->get_parameter("rate_hz").as_double(), 100.0);
    origin_ned_ = loadVector3(*this, "origin_ned", {0.0, 0.0, -1.0});
    amplitude_x_m_ =
        std::max(0.0, this->get_parameter("amplitude_x_m").as_double());
    amplitude_y_m_ =
        std::max(0.0, this->get_parameter("amplitude_y_m").as_double());
    period_s_ =
        clampPositive(this->get_parameter("period_s").as_double(), 12.0);
    startup_smoothing_s_ =
        std::max(0.0, this->get_parameter("startup_smoothing_s").as_double());
    yaw_rad_ = this->get_parameter("yaw_rad").as_double();
  }

  void createRosInterfaces() {
    reference_pub_ = this->create_publisher<
        flat_trajectory_msgs::msg::FlatTrajectoryReference>(
        reference_topic_, rclcpp::QoS(rclcpp::KeepLast(10)));

    const auto start_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    start_tracking_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        start_tracking_topic_, start_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (!msg) {
            return;
          }
          if (msg->data && !enabled_) {
            start_time_ = this->now();
          }
          enabled_ = msg->data;
        });

    if (!require_start_signal_ && enabled_) {
      start_time_ = this->now();
    }

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz_),
        std::bind(&LissajousTrajectoryPublisher::publishReference, this));
  }

  void publishReference() {
    if (require_start_signal_ && !enabled_) {
      return;
    }
    if (!enabled_) {
      enabled_ = true;
      start_time_ = this->now();
    }

    const double t = std::max(0.0, (this->now() - start_time_).seconds());
    const StartupBlend startup_blend =
        smoothStartupBlend(t, startup_smoothing_s_);
    const double base_omega = 2.0 * kPi / period_s_;

    flat_trajectory_msgs::msg::FlatTrajectoryReference ref;
    ref.header.stamp = this->now();
    ref.header.frame_id = frame_id_;
    ref.yaw = static_cast<float>(yaw_rad_);
    ref.yaw_rate = 0.0f;
    ref.yaw_acceleration = 0.0f;

    std::array<double, 3> position{};
    std::array<double, 3> velocity{};
    std::array<double, 3> acceleration{};
    std::array<double, 3> jerk{};
    std::array<double, 3> snap{};
    const std::array<double, 3> amplitudes{amplitude_x_m_, amplitude_y_m_, 0.0};

    for (int axis = 0; axis < 3; ++axis) {
      const double omega = base_omega * kFigure8Harmonics[axis];
      const double angle = omega * t + kFigure8PhasesRad[axis];
      const double amplitude = amplitudes[axis];
      const double sin_angle = std::sin(angle);
      const double cos_angle = std::cos(angle);
      const double raw_position = amplitude * sin_angle;
      const double raw_velocity = amplitude * omega * cos_angle;
      const double raw_acceleration = -amplitude * omega * omega * sin_angle;
      const double raw_jerk = -amplitude * omega * omega * omega * cos_angle;
      const double raw_snap =
          amplitude * omega * omega * omega * omega * sin_angle;

      position[axis] = origin_ned_[axis] + startup_blend.value * raw_position;
      velocity[axis] =
          startup_blend.dot * raw_position + startup_blend.value * raw_velocity;
      acceleration[axis] = startup_blend.ddot * raw_position +
                           2.0 * startup_blend.dot * raw_velocity +
                           startup_blend.value * raw_acceleration;
      jerk[axis] = startup_blend.dddot * raw_position +
                   3.0 * startup_blend.ddot * raw_velocity +
                   3.0 * startup_blend.dot * raw_acceleration +
                   startup_blend.value * raw_jerk;
      snap[axis] = startup_blend.ddddot * raw_position +
                   4.0 * startup_blend.dddot * raw_velocity +
                   6.0 * startup_blend.ddot * raw_acceleration +
                   4.0 * startup_blend.dot * raw_jerk +
                   startup_blend.value * raw_snap;
    }

    ref.position_ned.x = position[0];
    ref.position_ned.y = position[1];
    ref.position_ned.z = position[2];
    ref.velocity_ned.x = velocity[0];
    ref.velocity_ned.y = velocity[1];
    ref.velocity_ned.z = velocity[2];
    ref.acceleration_ned.x = acceleration[0];
    ref.acceleration_ned.y = acceleration[1];
    ref.acceleration_ned.z = acceleration[2];
    ref.jerk_ned.x = jerk[0];
    ref.jerk_ned.y = jerk[1];
    ref.jerk_ned.z = jerk[2];
    ref.snap_ned.x = snap[0];
    ref.snap_ned.y = snap[1];
    ref.snap_ned.z = snap[2];

    reference_pub_->publish(ref);
  }

  std::string reference_topic_;
  std::string start_tracking_topic_;
  std::string frame_id_;
  bool require_start_signal_{true};
  bool enabled_{false};
  double rate_hz_{100.0};
  double period_s_{12.0};
  double startup_smoothing_s_{2.0};
  double yaw_rad_{0.0};
  std::array<double, 3> origin_ned_{0.0, 0.0, -1.0};
  double amplitude_x_m_{0.8};
  double amplitude_y_m_{0.5};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<flat_trajectory_msgs::msg::FlatTrajectoryReference>::
      SharedPtr reference_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_tracking_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LissajousTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
