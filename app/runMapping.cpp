
#include "MapRosNode.h"
#include "RadarPointType.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto rpg = std::make_shared<MapOptimize<pcl::PointXYZSD>>();
  std::thread map_thread(&MapOptimize<pcl::PointXYZSD>::render_map, rpg);
  rclcpp::spin(rpg);
  rclcpp::shutdown();
  map_thread.join();
  return 0;
}