#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "tanh_ctrl/tanh_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tanh_ctrl::TanhNode>());
  rclcpp::shutdown();
  return 0;
}
