#include "RadarVelocityEstimatorCelestial.h"
#include <chrono>
using namespace std::chrono;

RadarVelocityEstimatorCelestial::RadarVelocityEstimatorCelestial(
    SharedNoiseModel doppler_noiseModel, SharedNoiseModel angle_noiseModel)
{
    this->doppler_noise_model = doppler_noiseModel;
    this->angle_noise_model = angle_noiseModel;
    this->inlier_thresh = 0.1;
    inlier_pc.reset(new pcl::PointCloud<pcl::PointXYZIS>);
}

void RadarVelocityEstimatorCelestial::set_inlier_thresh(float thresh)
{
    this->inlier_thresh = thresh;
}

void RadarVelocityEstimatorCelestial::set_initial_velocity(Vector3 init_v)
{
    this->initial_v = init_v;
}

void RadarVelocityEstimatorCelestial::set_prior_noise_model(SharedNoiseModel noise_model)
{
    this->prior_noise_model = noise_model;
}

float RadarVelocityEstimatorCelestial::doppler_projection(pcl::PointXYZIS p, Vector3 v)
{
    Vector3 unit_r(p.x, p.y, p.z);
    unit_r = -unit_r / unit_r.norm();
    return unit_r.dot(v);
}

gtsam::Vector2 RadarVelocityEstimatorCelestial::toCelestial(pcl::PointXYZIS &p)
{
    return gtsam::Vector2(atan2(p.y, p.x), atan2(p.z, sqrt(p.x * p.x + p.y * p.y)));
}

Vector3 RadarVelocityEstimatorCelestial::optimize(pcl::PointCloud<pcl::PointXYZIS>::Ptr radar_pc)
{
    NonlinearFactorGraph graph;
    Values initial;
    initial.insert(0, initial_v);

    size_t size = 0;
    for (pcl::PointXYZIS p : radar_pc->points)
    {
        Vector2 cel = toCelestial(p);
        auto dFactor = DopplerCelestial2Factor(0, C(size), p.speed, doppler_noise_model);
        graph.add(dFactor);
        graph.addPrior(C(size), cel, angle_noise_model);
        initial.insert(C(size), cel);
        size++;
    }

    auto start = high_resolution_clock::now();
    // optimie
    LevenbergMarquardtParams params;
    // params.setVerbosityLM("summary");
    // params.setRelativeErrorTol(1e-3);

    LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    Values result = optimizer.optimize();

    // optimizer.print();
    // Values result = GaussNewtonOptimizer(graph, initial).optimize();

    Vector3 v_est = result.at<Vector3>(0);

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    cout << "optimize:" << duration.count() / 1e6 << endl;

    for (size_t i = 0; i < size; i++)
    {
        pcl::PointXYZIS p = radar_pc->points[i];
        double r = Vector3(p.x, p.y, p.z).norm();
        Vector2 cel = result.at<Vector2>(C(i));
        p.x = r * cos(cel(1)) * cos(cel(0));
        p.y = r * cos(cel(1)) * sin(cel(0));
        p.z = r * sin(cel(1));
        radar_pc->points[i] = p;
    }

    return v_est;
}

pcl::PointCloud<pcl::PointXYZIS>::Ptr RadarVelocityEstimatorCelestial::get_inlier(
    pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc, Vector3 velocity)
{
    // collect inliers
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

Matrix RadarVelocityEstimatorCelestial::get_covariance(
    pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc, Vector3 velocity)
{
    NonlinearFactorGraph graph;
    Values initial;

    size_t size = 0;
    for (pcl::PointXYZIS p : radar_pc->points)
    {
        Vector2 cel = toCelestial(p);
        auto dFactor = DopplerCelestial2Factor(
            0, C(size), p.speed, nullptr);
        graph.add(dFactor);
        graph.addPrior(C(size), cel, nullptr);
        initial.insert(C(size), cel);
        size++;
    }

    initial.insert(0, velocity);
    Marginals marginals(graph, initial);
    Matrix covariance_matrix = marginals.marginalCovariance(0);

    return covariance_matrix;
}

RadarVelocityEstimatorCelestial::~RadarVelocityEstimatorCelestial()
{
}
