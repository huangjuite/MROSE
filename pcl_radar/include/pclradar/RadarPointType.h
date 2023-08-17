#ifndef RADAR_POINT_TYPE_H
#define RADAR_POINT_TYPE_H

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

namespace pcl
{
    struct PointXYZSD
    {
        PCL_ADD_POINT4D;
        float snr;
        float doppler;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
    } EIGEN_ALIGN16;

    struct PointXYZSDW
    {
        PCL_ADD_POINT4D;
        float snr;
        float doppler;
        float weight;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
    } EIGEN_ALIGN16;

    struct PointXYZSNV
    {
        PCL_ADD_POINT4D;
        float snr_db;
        float noise_db;
        float v_doppler_mps;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
    } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointXYZSD,
    (float, x, x)(float, y, y)(float, z, z)(float, snr, snr)(float, doppler, doppler))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointXYZSDW,
    (float, x, x)(float, y, y)(float, z, z)(float, snr, snr)(float, doppler, doppler)(float, weight, weight))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointXYZSNV,
    (float, x, x)(float, y, y)(float, z, z)(float, snr_db, snr_db)(float, noise_db, noise_db)(float, v_doppler_mps, v_doppler_mps))

#endif