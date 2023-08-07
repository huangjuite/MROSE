#include "RadarVelocityEstimatorCeresHCS.h"

void RadarVelocityEstimatorCeresHCS::set_uncertainty(
    double doppler_std, double azimuth_std, double elevation_std)
{
    this->doppler_std = doppler_std;
    this->azimuth_std = azimuth_std;
    this->elevation_std = elevation_std;
}

Eigen::Vector2d RadarVelocityEstimatorCeresHCS::toHCS(pcl::PointXYZIS &p)
{
    return Eigen::Vector2d(atan2(p.y, p.x), atan2(p.z, sqrt(p.x * p.x + p.y * p.y)));
}

Eigen::Vector3d RadarVelocityEstimatorCeresHCS::optimize(
    pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc)
{
    double *v_est = initial_v.data();

    ceres::Problem problem;
    std::vector<ceres::ResidualBlockId> residual_ids;

    problem.AddParameterBlock(v_est, 3);

    for (pcl::PointXYZIS p : radar_pc->points)
    {
        Eigen::Vector2d angles = toHCS(p);
        double *a = new double[2]{angles[0], angles[1]};

        problem.AddParameterBlock(a, 2);

        DopplerAngleCostFunction *cost_function = new DopplerAngleCostFunction(p.speed, doppler_std);

        ceres::ResidualBlockId id = problem.AddResidualBlock(
            cost_function, new ceres::CauchyLoss(0.2), v_est, a);
        residual_ids.push_back(id);

        // double s[2] = {2 * M_PI / 256.0, 2 * M_PI / 256.0};
        // s[1] *= 4;

        double s[2] = {azimuth_std, elevation_std}; // {1.41 deg, 18 deg}

        Eigen::Vector2d angle_sigma(s);
        Eigen::Matrix2d A_mat =
            (Eigen::Matrix2d() << 1 / angle_sigma[0], 0, 0, 1 / angle_sigma[1])
                .finished();
        ceres::NormalPrior *angle_prior = new ceres::NormalPrior(A_mat, angles);
        ceres::ResidualBlockId pid = problem.AddResidualBlock(angle_prior, nullptr, a);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 100;
    // options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    // std::cout << summary.FullReport() << std::endl;


    // pre compute covariance =====================================================
    ceres::Covariance::Options cov_opt;
    ceres::Covariance covariance(cov_opt);

    std::vector<std::pair<const double *, const double *>> covariance_blocks;
    covariance_blocks.push_back(std::make_pair(v_est, v_est));
    covariance.Compute(covariance_blocks, &problem);
    double cov[3 * 3];
    covariance.GetCovarianceBlock(v_est, v_est, cov);
    covariance_matrix = Eigen::Matrix3d(cov);

    return Eigen::Vector3d(v_est);
}

Eigen::Matrix3d RadarVelocityEstimatorCeresHCS::get_covariance(
    pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc, Eigen::Vector3d velocity)
{
    // double *v_est = velocity.data();

    // ceres::Problem problem;

    // problem.AddParameterBlock(v_est, 3);

    // for (pcl::PointXYZIS p : radar_pc->points)
    // {
    //     Eigen::Vector3d pos = (Eigen::Vector3d() << p.x, p.y, p.z).finished();
    //     ceres::CostFunction *cost_function =
    //         new ceres::AutoDiffCostFunction<DopplerResidual, 1, 3>(
    //             new DopplerResidual(pos, p.speed));
    //     problem.AddResidualBlock(cost_function, nullptr, v_est);
    // }

    // ceres::Covariance::Options cov_opt;
    // ceres::Covariance covariance(cov_opt);

    // std::vector<std::pair<const double *, const double *>> covariance_blocks;
    // covariance_blocks.push_back(std::make_pair(v_est, v_est));
    // covariance.Compute(covariance_blocks, &problem);
    // double cov[3 * 3];
    // covariance.GetCovarianceBlock(v_est, v_est, cov);
    // Eigen::Matrix3d covariance_matrix(cov);

    return covariance_matrix;
}
