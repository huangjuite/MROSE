#include "velocity/RadarVelocityEstimatorCeres.hpp"

#include <optional>

RadarVelocityEstimatorCeres::RadarVelocityEstimatorCeres(
    std::optional<ceres::Solver::Options> options,
    std::optional<float> doppler_sigma, std::optional<float> cauchy_scale) {
  if (options.has_value())
    this->options = options.value();
  else
    this->options.linear_solver_type = ceres::DENSE_QR;

  if (doppler_sigma.has_value())
    this->doppler_sigma = doppler_sigma.value();

  if (cauchy_scale.has_value())
    this->cauchy_scale = cauchy_scale.value();
}

Eigen::Vector3d
RadarVelocityEstimatorCeres::optimize(const rplrio::PointCloudPtr &radar_pc) {
  double v_est[3] = {this->initial_v[0], this->initial_v[1],
                     this->initial_v[2]};

  ceres::Problem problem;
  std::vector<ceres::ResidualBlockId> residual_ids;

  problem.AddParameterBlock(v_est, 3);

  for (const auto &p : *radar_pc) {
    Eigen::Vector3d pos = (Eigen::Vector3d() << p.x, p.y, p.z).finished();
    DopplerCostFunction *cost_function =
        new DopplerCostFunction(pos, p.doppler, this->doppler_sigma);
    ceres::ResidualBlockId id = problem.AddResidualBlock(
        cost_function, new ceres::CauchyLoss(cauchy_scale), v_est);
    residual_ids.push_back(id);
  }

  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);

  ceres::Covariance::Options cov_opt;
  ceres::Covariance covariance(cov_opt);

  std::vector<std::pair<const double *, const double *>> covariance_blocks;
  covariance_blocks.push_back(std::make_pair(v_est, v_est));
  covariance.Compute(covariance_blocks, &problem);
  double cov[3 * 3];
  covariance.GetCovarianceBlock(v_est, v_est, cov);
  this->covariance_matrix = Eigen::Matrix3d(cov);

  return Eigen::Vector3d(v_est);
}

Eigen::Matrix3d RadarVelocityEstimatorCeres::get_covariance(
    const rplrio::PointCloudPtr &radar_pc, Eigen::Vector3d velocity) {
  return this->covariance_matrix;
}
