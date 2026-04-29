#pragma once

// PCL includes
#include "radar/RadarVelocityEstimatorBase.hpp"
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
  DopplerCostFunction() {}

  DopplerCostFunction(Eigen::Vector3d pos, float doppler, double sigma) {
    p_ = -pos / pos.norm();
    d_ = double(doppler);
    s_ = sigma;
  };

  ~DopplerCostFunction() {};

  void setValues(Eigen::Vector3d pos, float doppler, double sigma) {
    p_ = -pos / pos.norm();
    d_ = double(doppler);
    s_ = sigma;
  }

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

template <typename PointType>
class RadarVelocityEstimatorCeres
    : public RadarVelocityEstimatorBase<PointType> {
  typedef typename pcl::PointCloud<PointType>::ConstPtr PclConstPtr;
  typedef typename pcl::PointCloud<PointType>::Ptr PclPtr;
  typedef typename pcl::PointCloud<PointType> Pcl;
  ceres::Solver::Options options;
  float cauchy_scale_;

public:
  RadarVelocityEstimatorCeres(
      std::optional<ceres::Solver::Options> options = std::nullopt,
      float doppler_sigma = 0.124f,
      float cauchy_scale = 1.0f,
      float min_pt_dist = 0.1f
    );
  Eigen::Vector3d optimize(PclConstPtr radar_pc);
  Eigen::Matrix3d get_covariance(PclConstPtr radar_pc,
                                 Eigen::Vector3d velocity);
};
