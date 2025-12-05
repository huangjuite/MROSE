#pragma once

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pcl {

struct EIGEN_ALIGN16 PointXYZD {
  PCL_ADD_POINT4D;
  float doppler;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointXYZSD {
  PCL_ADD_POINT4D;
  float snr;
  float doppler;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointXYZSDW {
  PCL_ADD_POINT4D;
  float snr;
  float doppler;
  float weight;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointXYZSNV {
  PCL_ADD_POINT4D;
  float snr_db;
  float noise_db;
  float v_doppler_mps;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointHugin {
  PCL_ADD_POINT4D;
  float range;
  float elevation;
  float azimuth;
  float power;
  float doppler;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZD,
                                  (float, x, x)(float, y, y)(float, z,
                                                             z)(float, doppler,
                                                                doppler))

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZSD,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, snr, snr)(float, doppler, doppler))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointXYZSDW, (float, x, x)(float, y, y)(float, z, z)(float, snr, snr)(
                          float, doppler, doppler)(float, weight, weight))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointXYZSNV,
    (float, x, x)(float, y, y)(float, z, z)(float, snr_db, snr_db)(
        float, noise_db, noise_db)(float, v_doppler_mps, v_doppler_mps))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointHugin,
    (float, x, x)(float, y, y)(float, z, z)(float, range,
                                            range)(float, elevation, elevation)(
        float, azimuth, azimuth)(float, power, power)(float, doppler, doppler))
