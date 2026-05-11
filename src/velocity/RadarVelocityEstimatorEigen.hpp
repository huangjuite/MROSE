#pragma once

#include "velocity/RadarVelocityEstimatorBase.hpp"

class RadarVelocityEstimatorEigen : public RadarVelocityEstimatorBase {
  int max_iterations_;
  float min_pt_dist_;
  double convergence_tol_;

public:
  RadarVelocityEstimatorEigen(int max_iterations = 10,
                              float min_pt_dist = 0.1f,
                              double convergence_tol = 1e-7);
  Eigen::Vector3d optimize(const rplrio::PointCloudPtr &radar_pc) override;
  Eigen::Matrix3d get_covariance(const rplrio::PointCloudPtr &radar_pc,
                                 Eigen::Vector3d velocity) override;
};
