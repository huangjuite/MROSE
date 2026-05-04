#include <rclcpp/rclcpp.hpp>

#include "DIORosNode.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DIORosNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
