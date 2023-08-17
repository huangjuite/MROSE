#include "RadarVelocityEstimator.h"

RadarVelocityEstimator::RadarVelocityEstimator(SharedNoiseModel doppler_noiseModel)
{
    this->doppler_noise_model = doppler_noiseModel;
    this->inlier_thresh = 0.1;
    inlier_pc.reset(new pcl::PointCloud<pcl::PointXYZSD>);
}

void RadarVelocityEstimator::set_inlier_thresh(float thresh)
{
    this->inlier_thresh = thresh;
}

void RadarVelocityEstimator::set_initial_velocity(Vector3 init_v)
{
    this->initial_v = init_v;
}

void RadarVelocityEstimator::set_prior_noise_model(SharedNoiseModel noise_model)
{
    this->prior_noise_model = noise_model;
}

float RadarVelocityEstimator::doppler_projection(pcl::PointXYZSD p, Vector3 v)
{
    Vector3 unit_r(p.x, p.y, p.z);
    unit_r = -unit_r / unit_r.norm();
    return unit_r.dot(v);
}

Vector3 RadarVelocityEstimator::optimize(pcl::PointCloud<pcl::PointXYZSD>::ConstPtr radar_pc)
{
    NonlinearFactorGraph graph;

    for (pcl::PointXYZSD p : radar_pc->points)
    {
        auto dFactor = DopplerMeasurementFactor(0, p.x, p.y, p.z, p.doppler, doppler_noise_model);
        graph.add(dFactor);
    }

    // initial estimate
    Values initial;
    initial.insert(0, initial_v);

    // optimization
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    Vector3 v_est = result.at<Vector3>(0);

    return v_est;
}

pcl::PointCloud<pcl::PointXYZSD>::Ptr RadarVelocityEstimator::get_inlier(
    pcl::PointCloud<pcl::PointXYZSD>::ConstPtr radar_pc, Vector3 velocity)
{
    // collect inliers
    inlier_pc.reset(new pcl::PointCloud<pcl::PointXYZSD>);
    for (pcl::PointXYZSD p : radar_pc->points)
    {
        float err = abs(doppler_projection(p, velocity) - p.doppler);
        if (err < inlier_thresh)
        {
            inlier_pc->push_back(p);
        }
    }

    return inlier_pc;
}

Matrix RadarVelocityEstimator::get_covariance(
    pcl::PointCloud<pcl::PointXYZSD>::ConstPtr radar_pc, Vector3 velocity)
{
    NonlinearFactorGraph graph;

    for (pcl::PointXYZSD p : radar_pc->points)
    {
        auto dFactor = DopplerMeasurementFactor(0, p.x, p.y, p.z, p.doppler, nullptr);
        graph.add(dFactor);
    }

    Values initial;
    initial.insert(0, velocity);
    Marginals marginals(graph, initial);
    covariance_matrix = marginals.marginalCovariance(0);

    return covariance_matrix;
}

RadarVelocityEstimator::~RadarVelocityEstimator()
{
}
