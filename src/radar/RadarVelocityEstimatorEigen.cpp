#include "radar/RadarVelocityEstimatorEigen.hpp"
#include "core/RadarPointType.hpp"
#include "core/ScopedTimer.hpp"
#include <spdlog/spdlog.h>

template <typename PointType>
RadarVelocityEstimatorEigen<PointType>::RadarVelocityEstimatorEigen(
    float cauchy_scale, int max_iterations, float min_pt_dist, double convergence_tol)
    : cauchy_scale_(cauchy_scale), max_iterations_(max_iterations), convergence_tol_(convergence_tol) {
        this->min_pt_dist_ = min_pt_dist;
      }

template <typename PointType>
Eigen::Vector3d
RadarVelocityEstimatorEigen<PointType>::optimize(PclConstPtr radar_pc) {
  const int n = radar_pc->points.size();
  const double sigma = this->doppler_sigma_;
  const double c2 = static_cast<double>(cauchy_scale_) * cauchy_scale_;

  // No Auto here since Eigen returns lazy expression
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, 3);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(n);

  double min_pt_dist_double = static_cast<double>(this->min_pt_dist_);

  auto timer_logger = spdlog::get("rplrio");
  {
    rplrio::ScopedTimer t(timer_logger, "eigen_populate", this->timing_csv_);
    for (int i = 0; i < n; ++i) {
      const auto &p = radar_pc->points[i];
      Eigen::Vector3d pos(p.x, p.y, p.z);
      double norm = pos.norm();

      // We can skip a row because it's set to zeros above
      if (norm < min_pt_dist_double) continue;
      A(i, 0) = -p.x / (norm * sigma);
      A(i, 1) = -p.y / (norm * sigma);
      A(i, 2) = -p.z / (norm * sigma);
      b(i) = p.doppler / sigma;
    }
  }

  Eigen::Vector3d v = this->initial_v;

  {
    rplrio::ScopedTimer t(timer_logger, "eigen_solve", this->timing_csv_);

    for (int iter = 0; iter < max_iterations_; ++iter) {
      Eigen::VectorXd r = A * v - b;

      Eigen::VectorXd w(n);
      for (int i = 0; i < n; ++i) {
        w(i) = 1.0 / (1.0 + r(i) * r(i) / c2);
      }

      Eigen::Matrix3d AtWA = A.transpose() * w.asDiagonal() * A;
      Eigen::Vector3d AtWb = A.transpose() * (w.asDiagonal() * b);

      Eigen::Vector3d v_new = AtWA.ldlt().solve(AtWb);

      if ((v_new - v).squaredNorm() < convergence_tol_) {
        v = v_new;
        break;
      }
      v = v_new;
    }
  }

  {
    rplrio::ScopedTimer t(timer_logger, "eigen_covariance", this->timing_csv_);
    Eigen::VectorXd r = A * v - b;
    Eigen::VectorXd w(n);
    for (int i = 0; i < n; ++i) {
      w(i) = 1.0 / (1.0 + r(i) * r(i) / c2);
    }
    Eigen::Matrix3d AtWA = A.transpose() * w.asDiagonal() * A;
    this->covariance_matrix = AtWA.inverse();
  }

  return v;
}

template <typename PointType>
Eigen::Matrix3d RadarVelocityEstimatorEigen<PointType>::get_covariance(
    PclConstPtr radar_pc, Eigen::Vector3d velocity) {
  return this->covariance_matrix;
}

template class RadarVelocityEstimatorEigen<pcl::PointHugin>;
template class RadarVelocityEstimatorEigen<pcl::PointXYZSD>;
template class RadarVelocityEstimatorEigen<pcl::PointXYZD>;
