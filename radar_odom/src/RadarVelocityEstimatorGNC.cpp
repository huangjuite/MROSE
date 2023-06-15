#include "RadarVelocityEstimatorGNC.h"

RadarVelocityEstimatorGNC::RadarVelocityEstimatorGNC(SharedNoiseModel doppler_noiseModel)
{
    this->doppler_noise_model = doppler_noiseModel;
    this->inlier_thresh = 0.1;
    inlier_pc.reset(new pcl::PointCloud<pcl::PointXYZIS>);
}

void RadarVelocityEstimatorGNC::set_inlier_thresh(float thresh)
{
    this->inlier_thresh = thresh;
}

void RadarVelocityEstimatorGNC::set_initial_velocity(Vector3 init_v)
{
    this->initial_v = init_v;
}

void RadarVelocityEstimatorGNC::set_prior_noise_model(SharedNoiseModel noise_model)
{
    this->prior_noise_model = noise_model;
}

float RadarVelocityEstimatorGNC::doppler_projection(pcl::PointXYZIS p, Vector3 v)
{
    Vector3 unit_r(p.x, p.y, p.z);
    unit_r = -unit_r / unit_r.norm();
    return unit_r.dot(v);
}

Vector3 RadarVelocityEstimatorGNC::optimize(pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc)
{
    NonlinearFactorGraph graph;

    for (pcl::PointXYZIS p : radar_pc->points)
    {
        auto dFactor = DopplerMeasurementFactor(0, p.x, p.y, p.z, p.speed, doppler_noise_model);
        graph.add(dFactor);
    }

    // initial estimate
    Values initial;
    initial.insert(0, initial_v);

    // optimization
    GncParams<LevenbergMarquardtParams> gncParams;
    gncParams.setLossType(GncLossType::TLS);
    // cout << "=====================" << endl;
    // gncParams.setVerbosityGNC(GncParams<LevenbergMarquardtParams>::Verbosity::SUMMARY);
    auto gnc_gm = GncOptimizer<GncParams<LevenbergMarquardtParams>>(
        graph, initial, gncParams);
    Values result = gnc_gm.optimize();
    Vector3 v_est = result.at<Vector3>(0);

    return v_est;
}

pcl::PointCloud<pcl::PointXYZIS>::Ptr RadarVelocityEstimatorGNC::get_inlier(
    pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc, Vector3 velocity)
{
    // collect inliers
    inlier_pc.reset(new pcl::PointCloud<pcl::PointXYZIS>);
    for (pcl::PointXYZIS p : radar_pc->points)
    {
        float err = abs(doppler_projection(p, velocity) - p.speed);
        if (err < inlier_thresh)
        {
            inlier_pc->push_back(p);
        }
    }

    return inlier_pc;
}

Matrix RadarVelocityEstimatorGNC::get_covariance(
    pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc, Vector3 velocity)
{
    NonlinearFactorGraph graph;

    for (pcl::PointXYZIS p : radar_pc->points)
    {
        auto dFactor = DopplerMeasurementFactor(0, p.x, p.y, p.z, p.speed, nullptr);
        graph.add(dFactor);
    }

    Values initial;
    initial.insert(0, velocity);
    Marginals marginals(graph, initial);
    covariance_matrix = marginals.marginalCovariance(0);

    return covariance_matrix;
}

RadarVelocityEstimatorGNC::~RadarVelocityEstimatorGNC()
{
}
