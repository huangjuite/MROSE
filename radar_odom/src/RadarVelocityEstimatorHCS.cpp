#include "RadarVelocityEstimatorHCS.h"

RadarVelocityEstimatorHCS::RadarVelocityEstimatorHCS(SharedNoiseModel doppler_noiseModel)
{
    this->doppler_noise_model = doppler_noiseModel;
    this->inlier_thresh = 0.1;
}

void RadarVelocityEstimatorHCS::set_inlier_thresh(float thresh)
{
    this->inlier_thresh = thresh;
}

void RadarVelocityEstimatorHCS::set_initial_velocity(Vector3 init_v)
{
    this->initial_v = init_v;
}

void RadarVelocityEstimatorHCS::set_prior_noise_model(SharedNoiseModel noise_model)
{
    this->prior_noise_model = noise_model;
}

float RadarVelocityEstimatorHCS::doppler_projection(pcl::PointXYZIS p, Vector3 v)
{
    Vector3 unit_r(p.x, p.y, p.z);
    unit_r = -unit_r / unit_r.norm();
    return unit_r.dot(v);
}

gtsam::Vector2 RadarVelocityEstimatorHCS::toCelestial(pcl::PointXYZIS &p)
{
    return gtsam::Vector2(atan2(p.y, p.x), atan2(p.z, sqrt(p.x * p.x + p.y * p.y)));
}

Vector3 RadarVelocityEstimatorHCS::optimize(pcl::PointCloud<pcl::PointXYZIS>::Ptr radar_pc)
{
    NonlinearFactorGraph graph;
    Values initial;
    initial.insert(0, initial_v);

    size_t size = 0;
    for (pcl::PointXYZIS p : radar_pc->points)
    {

        Vector2 cel = toCelestial(p);
        auto dFactor = DopplerCelestialFactor(0, cel(0), cel(1), p.speed, doppler_noise_model);
        graph.add(dFactor);
        size++;
    }

    // optimie
    LevenbergMarquardtOptimizer optimizer(graph, initial);
    Values result = optimizer.optimize();
    Vector3 v_est = result.at<Vector3>(0);

    return v_est;
}

pcl::PointCloud<pcl::PointXYZIS>::Ptr RadarVelocityEstimatorHCS::get_inlier(
    pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc, Vector3 velocity)
{
    pcl::PointCloud<pcl::PointXYZIS>::Ptr inlier_pc;
    inlier_pc.reset(new pcl::PointCloud<pcl::PointXYZIS>);

    for (pcl::PointXYZIS p : radar_pc->points)
    {
        float err = abs(doppler_projection(p, velocity) - p.speed);
        if (err < inlier_thresh)
        {
            inlier_pc->points.push_back(p);
        }
    }
    return inlier_pc;
}

Matrix RadarVelocityEstimatorHCS::get_covariance(
    pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc, Vector3 velocity)
{
    NonlinearFactorGraph graph;
    Values initial;

    size_t size = 0;
    for (pcl::PointXYZIS p : radar_pc->points)
    {
        Vector2 cel = toCelestial(p);
        auto dFactor = DopplerCelestialFactor(
            0, cel(0), cel(1), p.speed, nullptr);
        graph.add(dFactor);
        size++;
    }

    initial.insert(0, velocity);
    Marginals marginals(graph, initial);
    Matrix covariance_matrix = marginals.marginalCovariance(0);

    return covariance_matrix;
}

RadarVelocityEstimatorHCS::~RadarVelocityEstimatorHCS()
{
}
