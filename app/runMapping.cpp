
#include "RadarPointType.h"
#include "RosNode/MapRosNode.h"
#include <rclcpp/rclcpp.hpp>

// typedef pcl::PointXYZSD PointType;
typedef pcl::PointHugin PointType;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto rpg = std::make_shared<MapOptimize<PointType>>();
  std::thread map_thread(&MapOptimize<PointType>::render_map, rpg);
  rclcpp::spin(rpg);
  rclcpp::shutdown();
  map_thread.join();
  return 0;
}