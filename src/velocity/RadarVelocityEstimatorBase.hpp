#pragma once

#include <Eigen/Dense>
#include <cstdlib>

#include "core/RadarPointType.hpp"

class RadarVelocityEstimatorBase {
protected:
  Eigen::Vector3d initial_v;
  Eigen::Matrix3d covariance_matrix;
  float inlier_thresh = 0.1;
  float doppler_sigma = 0.1;
  float cauchy_scale = 1.0;

  float doppler_projection(const rplrio::PointXYZD &p, Eigen::Vector3d v) {
    Eigen::Vector3d unit_r(p.x, p.y, p.z);
    unit_r = -unit_r / unit_r.norm();
    return unit_r.dot(v);
  }

public:
  RadarVelocityEstimatorBase(){};
  ~RadarVelocityEstimatorBase(){};

  void set_initial_velocity(Eigen::Vector3d init_v) {
    this->initial_v = init_v;
  };

  void set_inlier_thresh(float thresh) { this->inlier_thresh = thresh; };

  void set_doppler_sigma(float sigma) { this->doppler_sigma = sigma; };
  void set_cauchy_scale(float scale) { this->cauchy_scale = scale; };

  rplrio::PointCloudPtr get_inlier(const rplrio::PointCloudPtr &radar_pc,
                                   Eigen::Vector3d velocity) {
    auto inlier_pc = std::make_shared<rplrio::PointCloud>();
    for (const auto &p : *radar_pc) {
      float err = std::abs(doppler_projection(p, velocity) - p.doppler);
      if (err < inlier_thresh) {
        inlier_pc->push_back(p);
      }
    }
    return inlier_pc;
  };

  virtual Eigen::Vector3d optimize(const rplrio::PointCloudPtr &radar_pc) = 0;

  virtual Eigen::Matrix3d get_covariance(const rplrio::PointCloudPtr &radar_pc,
                                         Eigen::Vector3d velocity) = 0;
};
