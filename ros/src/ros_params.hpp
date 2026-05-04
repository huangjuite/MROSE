#pragma once

#include <string>
#include <unordered_map>

#include "core/params.hpp"

struct RosRadarParams {
  std::string topic;
  std::string type =
      "PointCloud2"; // "PointCloud2" or "TwistWithCovarianceStamped"
};

struct RosParams {
  rplrio::Params core;
  std::string imu_topic;
  std::unordered_map<std::string, RosRadarParams> ros_radars;
};

RosParams get_ros_params(const std::string &cfg_path);
