
#include "RosNode/DIORosNode.h"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

#include "RadarPointType.h"

template<typename PointType>
void runNode() {
  auto node = std::make_shared<DIORosNode<PointType>>();
  rclcpp::spin(node);
}

std::string getPointTypeFromConfig(const std::string& config_path) {
  try {
    YAML::Node config = YAML::LoadFile(config_path);
    if (config["point_type"]) {
      return config["point_type"].as<std::string>();
    }
  } catch (const YAML::Exception& e) {
    std::cerr << "Error reading config file: " << e.what() << std::endl;
  }
  return "";
}

int main(int argc, char *argv[]) {
  // Initialize ROS first to get parameters
  rclcpp::init(argc, argv);

  // Create temporary node to read config parameter
  auto temp_node = rclcpp::Node::make_shared(
    "temp_node",
    rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true)
  );

  std::string config_path;
  if (temp_node->has_parameter("config")) {
    config_path = temp_node->get_parameter("config").as_string();
  } else {
    std::cerr << "Error: config parameter not specified" << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  temp_node.reset();

  std::string point_type = getPointTypeFromConfig(config_path);

  if (point_type.empty()) {
    std::cerr << "Error: point_type not found in config file: " << config_path << std::endl;
    std::cerr << "Available types: PointXYZD, PointXYZSD, PointHugin" << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  if (point_type == "PointXYZD") {
    runNode<pcl::PointXYZD>();
  } else if (point_type == "PointXYZSD") {
    runNode<pcl::PointXYZSD>();
  } else if (point_type == "PointHugin") {
    runNode<pcl::PointHugin>();
  } else {
    std::cerr << "Error: Unknown point type '" << point_type << "'" << std::endl;
    std::cerr << "Available types: PointXYZD, PointXYZSD, PointHugin" << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}