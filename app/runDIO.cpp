
#include "RosNode/DIORosNode.h"
#include <rclcpp/rclcpp.hpp>

#include "RadarPointType.h"

// typedef pcl::PoinXYZSD PointType;
typedef pcl::PointHugin PointType;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DIORosNode<PointType>>());
  rclcpp::shutdown();
  return 0;
}