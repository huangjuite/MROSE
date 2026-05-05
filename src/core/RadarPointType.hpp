#pragma once

#include <memory>
#include <vector>

namespace rplrio {

struct PointXYZD {
  float x, y, z, doppler;
};

using PointCloud = std::vector<PointXYZD>;
using PointCloudPtr = std::shared_ptr<PointCloud>;

} // namespace rplrio
