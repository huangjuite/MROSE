#pragma once

#include "radar/RadarVelocityEstimatorBase.hpp"

template <typename PointType>
class RadarVelocityEstimatorEigen
    : public RadarVelocityEstimatorBase<PointType> {
  typedef typename pcl::PointCloud<PointType>::ConstPtr PclConstPtr;

  float cauchy_scale_;
  int max_iterations_;

public:
  RadarVelocityEstimatorEigen(float cauchy_scale = 1.0f,
                              int max_iterations = 10);
  Eigen::Vector3d optimize(PclConstPtr radar_pc) override;
  Eigen::Matrix3d get_covariance(PclConstPtr radar_pc,
                                 Eigen::Vector3d velocity) override;
};
