#include "ros_params.hpp"

#include <yaml-cpp/yaml.h>

RosParams get_ros_params(const std::string &cfg_path) {
  RosParams rp;
  rp.core = rplrio::get_params(cfg_path);

  YAML::Node config = YAML::LoadFile(cfg_path);
  const auto &ros = config["ros"];
  if (!ros)
    return rp;

  if (ros["imu_topic"])
    rp.imu_topic = ros["imu_topic"].as<std::string>();

  if (ros["radars"]) {
    for (const auto &entry : ros["radars"]) {
      std::string name = entry.first.as<std::string>();
      RosRadarParams rrp;
      if (entry.second["topic"])
        rrp.topic = entry.second["topic"].as<std::string>();
      if (entry.second["type"])
        rrp.type = entry.second["type"].as<std::string>();
      rp.ros_radars[name] = rrp;
    }
  }

  return rp;
}
