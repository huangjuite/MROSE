#include "RadarVelocityEstimatorGtsamHCS.h"

void RadarVelocityEstimatorGtsamHCS::set_uncertainty(
    double doppler_std, double azimuth_std, double elevation_std)
{
    doppler_noise_model = gtsam::noiseModel::Isotropic::Sigma(1, doppler_std);
    angle_noise_model = gtsam::noiseModel::Isotropic::Sigmas(
        gtsam::Vector2(azimuth_std, elevation_std));
}

gtsam::Vector2 RadarVelocityEstimatorGtsamHCS::toCelestial(pcl::PointXYZIS &p)
{
    return gtsam::Vector2(atan2(p.y, p.x), atan2(p.z, sqrt(p.x * p.x + p.y * p.y)));
}

Eigen::Vector3d RadarVelocityEstimatorGtsamHCS::optimize(
    pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc)
{
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    initial.insert(0, initial_v);

    size_t size = 0;
    for (pcl::PointXYZIS p : radar_pc->points)
    {
        gtsam::Vector2 cel = toCelestial(p);
        auto dFactor = DopplerCelestial2Factor(0, C(size), p.speed, doppler_noise_model);
        graph.add(dFactor);
        graph.addPrior(C(size), cel, angle_noise_model);
        initial.insert(C(size), cel);
        size++;
    }

    // optimie
    gtsam::LevenbergMarquardtParams params;
    // params.setVerbosityLM("summary");
    // params.setRelativeErrorTol(1e-3);

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    gtsam::Values result = optimizer.optimize();

    gtsam::Vector3 v_est = result.at<gtsam::Vector3>(0);

    gtsam::Marginals marginals(graph, result);
    gtsam::Matrix covariance_matrix = marginals.marginalCovariance(0);

    return v_est;
}

Eigen::Matrix3d RadarVelocityEstimatorGtsamHCS::get_covariance(
    pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc, Eigen::Vector3d velocity)
{

    return covariance_matrix;
}
