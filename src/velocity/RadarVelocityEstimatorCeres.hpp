#pragma once

#include "velocity/RadarVelocityEstimatorBase.hpp"
#include <ceres/ceres.h>
#include <ceres/solver.h>
#include <ceres/types.h>
#include <optional>

class DopplerCostFunction : public ceres::SizedCostFunction<1, 3> {
private:
  double d_;
  Eigen::Vector3d p_;
  double s_ = 1;

public:
  DopplerCostFunction(Eigen::Vector3d pos, float doppler, double sigma) {
    p_ = -pos / pos.norm();
    d_ = double(doppler);
    s_ = sigma;
  };

  ~DopplerCostFunction() {};
  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const {
    const double *v = parameters[0];

    residuals[0] = (p_[0] * v[0] + p_[1] * v[1] + p_[2] * v[2] - d_) / s_;

    if (jacobians != nullptr && jacobians[0] != nullptr) {
      jacobians[0][0] = p_[0] / s_;
      jacobians[0][1] = p_[1] / s_;
      jacobians[0][2] = p_[2] / s_;
    }

    return true;
  };
};

class RadarVelocityEstimatorCeres : public RadarVelocityEstimatorBase {
  ceres::Solver::Options options;
  float cauchy_scale;

public:
  RadarVelocityEstimatorCeres(
      std::optional<ceres::Solver::Options> options = std::nullopt,
      std::optional<float> doppler_sigma = 0.124,
      std::optional<float> cauchy_scale = 1.0);
  Eigen::Vector3d optimize(const rplrio::PointCloudPtr &radar_pc);
  Eigen::Matrix3d get_covariance(const rplrio::PointCloudPtr &radar_pc,
                                 Eigen::Vector3d velocity);
};
