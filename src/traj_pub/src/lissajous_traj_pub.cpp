#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <flat_trajectory_msgs/msg/flat_trajectory_reference.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;

std::array<double, 3> loadVector3(
    rclcpp::Node& node, const std::string& name, const std::array<double, 3>& fallback) {
  const auto values = node.get_parameter(name).as_double_array();
  if (values.size() != 3) {
    RCLCPP_WARN(node.get_logger(), "%s must have length 3; using fallback.", name.c_str());
    return fallback;
  }
  return {values[0], values[1], values[2]};
}

double clampPositive(double value, double fallback) {
  return value > 0.0 ? value : fallback;
}

}  // namespace

class LissajousTrajectoryPublisher : public rclcpp::Node {
 public:
  LissajousTrajectoryPublisher() : Node("lissajous_traj_pub") {
    declareParameters();
    loadParameters();
    createRosInterfaces();
  }

 private:
  void declareParameters() {
    this->declare_parameter<std::string>("topics.reference", "/tanh_ctrl/reference");
    this->declare_parameter<std::string>("topics.start_tracking", "/mission/start_tracking");
    this->declare_parameter<std::string>("frame_id", "ned");
    this->declare_parameter<bool>("require_start_signal", true);
    this->declare_parameter<bool>("start_enabled", false);
    this->declare_parameter<double>("rate_hz", 100.0);
    this->declare_parameter<std::vector<double>>("origin_ned", {0.0, 0.0, -1.0});
    this->declare_parameter<std::vector<double>>("amplitude_ned", {0.8, 0.5, 0.0});
    this->declare_parameter<double>("period_s", 12.0);
    this->declare_parameter<double>("x_harmonic", 1.0);
    this->declare_parameter<double>("y_harmonic", 2.0);
    this->declare_parameter<double>("z_harmonic", 1.0);
    this->declare_parameter<double>("phase_x_rad", 0.0);
    this->declare_parameter<double>("phase_y_rad", 0.0);
    this->declare_parameter<double>("phase_z_rad", 0.0);
    this->declare_parameter<double>("yaw_rad", 0.0);
  }

  void loadParameters() {
    reference_topic_ = this->get_parameter("topics.reference").as_string();
    start_tracking_topic_ = this->get_parameter("topics.start_tracking").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    require_start_signal_ = this->get_parameter("require_start_signal").as_bool();
    enabled_ = this->get_parameter("start_enabled").as_bool();
    rate_hz_ = clampPositive(this->get_parameter("rate_hz").as_double(), 100.0);
    origin_ned_ = loadVector3(*this, "origin_ned", {0.0, 0.0, -1.0});
    amplitude_ned_ = loadVector3(*this, "amplitude_ned", {0.8, 0.5, 0.0});
    period_s_ = clampPositive(this->get_parameter("period_s").as_double(), 12.0);
    harmonics_ = {
        this->get_parameter("x_harmonic").as_double(),
        this->get_parameter("y_harmonic").as_double(),
        this->get_parameter("z_harmonic").as_double(),
    };
    phases_ = {
        this->get_parameter("phase_x_rad").as_double(),
        this->get_parameter("phase_y_rad").as_double(),
        this->get_parameter("phase_z_rad").as_double(),
    };
    yaw_rad_ = this->get_parameter("yaw_rad").as_double();
  }

  void createRosInterfaces() {
    reference_pub_ = this->create_publisher<flat_trajectory_msgs::msg::FlatTrajectoryReference>(
        reference_topic_, rclcpp::QoS(rclcpp::KeepLast(10)));

    const auto start_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
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

    for (int axis = 0; axis < 3; ++axis) {
      const double omega = base_omega * harmonics_[axis];
      const double angle = omega * t + phases_[axis];
      const double amplitude = amplitude_ned_[axis];
      position[axis] = origin_ned_[axis] + amplitude * std::sin(angle);
      velocity[axis] = amplitude * omega * std::cos(angle);
      acceleration[axis] = -amplitude * omega * omega * std::sin(angle);
      jerk[axis] = -amplitude * omega * omega * omega * std::cos(angle);
      snap[axis] = amplitude * omega * omega * omega * omega * std::sin(angle);
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
  double yaw_rad_{0.0};
  std::array<double, 3> origin_ned_{0.0, 0.0, -1.0};
  std::array<double, 3> amplitude_ned_{0.8, 0.5, 0.0};
  std::array<double, 3> harmonics_{1.0, 2.0, 1.0};
  std::array<double, 3> phases_{0.0, 0.0, 0.0};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<flat_trajectory_msgs::msg::FlatTrajectoryReference>::SharedPtr reference_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_tracking_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LissajousTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
