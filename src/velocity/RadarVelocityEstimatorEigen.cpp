#include "velocity/RadarVelocityEstimatorEigen.hpp"
#include "core/RadarPointType.hpp"
#include <spdlog/spdlog.h>

RadarVelocityEstimatorEigen::RadarVelocityEstimatorEigen(
    int max_iterations, float min_pt_dist, double convergence_tol)
    : max_iterations_(max_iterations),
      min_pt_dist_(min_pt_dist), convergence_tol_(convergence_tol) {}

Eigen::Vector3d
RadarVelocityEstimatorEigen::optimize(const rplrio::PointCloudPtr &radar_pc) {
  const int n = radar_pc->size();
  const double sigma = this->doppler_sigma;
  const double c2 = static_cast<double>(cauchy_scale) * cauchy_scale;

  // No Auto here since Eigen returns lazy expression
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, 3);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(n);

  double min_pt_dist_double = static_cast<double>(min_pt_dist_);

  for (int i = 0; i < n; ++i) {
    const auto &p = (*radar_pc)[i];
    Eigen::Vector3d pos(p.x, p.y, p.z);
    double norm = pos.norm();

    // We can skip a row because it's set to zeros above
    if (norm < min_pt_dist_double) continue;
    A(i, 0) = -p.x / (norm * sigma);
    A(i, 1) = -p.y / (norm * sigma);
    A(i, 2) = -p.z / (norm * sigma);
    b(i) = p.doppler / sigma;
  }

  Eigen::Vector3d v = this->initial_v;

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

  Eigen::VectorXd r = A * v - b;
  Eigen::VectorXd w(n);
  for (int i = 0; i < n; ++i) {
    w(i) = 1.0 / (1.0 + r(i) * r(i) / c2);
  }
  Eigen::Matrix3d AtWA = A.transpose() * w.asDiagonal() * A;
  this->covariance_matrix = AtWA.inverse();

  return v;
}

Eigen::Matrix3d RadarVelocityEstimatorEigen::get_covariance(
    const rplrio::PointCloudPtr &radar_pc, Eigen::Vector3d velocity) {
  return this->covariance_matrix;
}
