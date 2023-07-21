#include "RadarVelocityEstimatorCeresNoise.h"

RadarVelocityEstimatorCeresNoise::RadarVelocityEstimatorCeresNoise()
{
    this->inlier_thresh = 0.1;
    inlier_pc.reset(new pcl::PointCloud<pcl::PointXYZISW>);
}

void RadarVelocityEstimatorCeresNoise::set_inlier_thresh(float thresh)
{
    this->inlier_thresh = thresh;
}

void RadarVelocityEstimatorCeresNoise::set_initial_velocity(Eigen::Vector3d init_v)
{
    this->initial_v = init_v;
}

float RadarVelocityEstimatorCeresNoise::doppler_projection(pcl::PointXYZISW p, Eigen::Vector3d v)
{
    Eigen::Vector3d unit_r(p.x, p.y, p.z);
    unit_r = -unit_r / unit_r.norm();
    return unit_r.dot(v);
}

Eigen::Vector3d RadarVelocityEstimatorCeresNoise::optimize(pcl::PointCloud<pcl::PointXYZISW>::ConstPtr radar_pc)
{
    double v_est[3] = {initial_v[0], initial_v[1], initial_v[2]};

    ceres::Problem problem;
    std::vector<ceres::ResidualBlockId> residual_ids;

    problem.AddParameterBlock(v_est, 3);

    for (pcl::PointXYZISW p : radar_pc->points)
    {
        Eigen::Vector3d pos = (Eigen::Vector3d() << p.x, p.y, p.z).finished();
        DopplerCostFunctionNoise *cost_function = new DopplerCostFunctionNoise(pos, p.speed, p.weight);
        ceres::ResidualBlockId id = problem.AddResidualBlock(cost_function, nullptr, v_est);
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

pcl::PointCloud<pcl::PointXYZISW>::Ptr RadarVelocityEstimatorCeresNoise::get_inlier(
    pcl::PointCloud<pcl::PointXYZISW>::ConstPtr radar_pc, Eigen::Vector3d velocity)
{
    // collect inliers
    inlier_pc.reset(new pcl::PointCloud<pcl::PointXYZISW>);
    for (pcl::PointXYZISW p : radar_pc->points)
    {
        float err = abs(doppler_projection(p, velocity) - p.speed);
        if (err < inlier_thresh)
        {
            inlier_pc->push_back(p);
        }
    }

    return inlier_pc;
}

Eigen::Matrix3d RadarVelocityEstimatorCeresNoise::get_covariance(
    pcl::PointCloud<pcl::PointXYZISW>::ConstPtr radar_pc, Eigen::Vector3d velocity)
{
    return covariance_matrix;
}

RadarVelocityEstimatorCeresNoise::~RadarVelocityEstimatorCeresNoise()
{
}
