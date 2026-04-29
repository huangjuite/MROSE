#include "radar/RadarVelocityEstimatorCeres.hpp"
#include "core/RadarPointType.hpp"
#include "core/ScopedTimer.hpp"
#include <optional>
#include <spdlog/spdlog.h>

template <typename PointType>
RadarVelocityEstimatorCeres<PointType>::RadarVelocityEstimatorCeres(
    std::optional<ceres::Solver::Options> options,
    float doppler_sigma, float cauchy_scale, float min_pt_dist) {

  if (options.has_value())
    this->options = options.value();
  else
    this->options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;

  this->cauchy_scale_ = cauchy_scale;
  this->doppler_sigma_ = doppler_sigma;
  this->min_pt_dist_ = min_pt_dist;
}

template <typename PointType>
Eigen::Vector3d
RadarVelocityEstimatorCeres<PointType>::optimize(PclConstPtr radar_pc) {
  double v_est[3] = {this->initial_v[0], this->initial_v[1],
                     this->initial_v[2]};

  // By Default ceres takes ownership and will deallocate at destructor
  ceres::Problem::Options problem_opts_;
  problem_opts_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_opts_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problem_opts_);
  problem.AddParameterBlock(v_est, 3);

  double min_pt_dist_double = static_cast<double>(this->min_pt_dist_);

  auto timer_logger = spdlog::get("rplrio");
  std::vector<DopplerCostFunction> cost_fn_pool_(radar_pc->points.size());
  ceres::CauchyLoss cauchy_loss(cauchy_scale_);
  {
    rplrio::ScopedTimer t(timer_logger, "ceres_populate", this->timing_csv_);
    for (size_t idx = 0; idx < radar_pc->points.size(); ++idx) {
      const auto & p = radar_pc->points[idx];
      Eigen::Vector3d pos(p.x, p.y, p.z);
      if (pos.norm() < min_pt_dist_double) continue;

      cost_fn_pool_[idx].setValues(pos, p.doppler, this->doppler_sigma_);
      ceres::ResidualBlockId id = problem.AddResidualBlock(
          &cost_fn_pool_[idx], &cauchy_loss, v_est);
    }
  }

  ceres::Solver::Summary summary;
  {
    rplrio::ScopedTimer t(timer_logger, "ceres_solve", this->timing_csv_);
    Solve(options, &problem, &summary);
  }

  // std::cout << summary.FullReport() << std::endl;

  {
    rplrio::ScopedTimer t(timer_logger, "ceres_covariance", this->timing_csv_);
    ceres::Covariance::Options cov_opt;
    ceres::Covariance covariance(cov_opt);

    std::vector<std::pair<const double *, const double *>> covariance_blocks;
    covariance_blocks.push_back(std::make_pair(v_est, v_est));
    covariance.Compute(covariance_blocks, &problem);
    double cov[3 * 3];
    covariance.GetCovarianceBlock(v_est, v_est, cov);
    this->covariance_matrix = Eigen::Matrix3d(cov);
  }

  // ceres::Problem::EvaluateOptions eval_opt;
  // eval_opt.residual_blocks = residual_ids;
  // double total_cost = 0.0;
  // std::vector<double> residuals;
  // problem.Evaluate(eval_opt, &total_cost, &residuals, nullptr, nullptr);

  // std::cout << residuals.size() << std::endl;

  return Eigen::Vector3d(v_est);
}

template <typename PointType>
Eigen::Matrix3d RadarVelocityEstimatorCeres<PointType>::get_covariance(
    PclConstPtr radar_pc, Eigen::Vector3d velocity) {
  return this->covariance_matrix;
}

// Template instantiation
template class RadarVelocityEstimatorCeres<pcl::PointHugin>;
template class RadarVelocityEstimatorCeres<pcl::PointXYZSD>;
template class RadarVelocityEstimatorCeres<pcl::PointXYZD>;
