#include "RadarVelocityEstimatorCeres.h"

Eigen::Vector3d RadarVelocityEstimatorCeres::optimize(pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc)
{
    double v_est[3] = {initial_v[0], initial_v[1], initial_v[2]};

    ceres::Problem problem;
    std::vector<ceres::ResidualBlockId> residual_ids;

    problem.AddParameterBlock(v_est, 3);

    for (auto p : radar_pc->points)
    {
        Eigen::Vector3d pos = (Eigen::Vector3d() << p.x, p.y, p.z).finished();
        DopplerCostFunction *cost_function = new DopplerCostFunction(pos, p.speed);
        ceres::ResidualBlockId id = problem.AddResidualBlock(
            cost_function, new ceres::CauchyLoss(0.05), v_est);
        residual_ids.push_back(id);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    // options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);

    // std::cout << summary.FullReport() << std::endl;

    ceres::Covariance::Options cov_opt;
    ceres::Covariance covariance(cov_opt);

    std::vector<std::pair<const double *, const double *>> covariance_blocks;
    covariance_blocks.push_back(std::make_pair(v_est, v_est));
    covariance.Compute(covariance_blocks, &problem);
    double cov[3 * 3];
    covariance.GetCovarianceBlock(v_est, v_est, cov);
    covariance_matrix = Eigen::Matrix3d(cov);

    // ceres::Problem::EvaluateOptions eval_opt;
    // eval_opt.residual_blocks = residual_ids;
    // double total_cost = 0.0;
    // std::vector<double> residuals;
    // problem.Evaluate(eval_opt, &total_cost, &residuals, nullptr, nullptr);

    // std::cout << residuals.size() << std::endl;

    return Eigen::Vector3d(v_est);
}

Eigen::Matrix3d RadarVelocityEstimatorCeres::get_covariance(
    pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc, Eigen::Vector3d velocity)
{
    return covariance_matrix;
}
