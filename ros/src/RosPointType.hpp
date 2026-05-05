#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

namespace pcl {

struct EIGEN_ALIGN16 PointXYZDoppler {
  PCL_ADD_POINT4D;
  float doppler;
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZDoppler,
                                  (float, x, x)(float, y, y)(float, z,
                                                             z)(float, doppler,
                                                                doppler))
