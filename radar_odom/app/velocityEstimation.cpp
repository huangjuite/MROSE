#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pclradar/RadarPointType.h>

// GTSAM related includes.
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "utils.h"
#include "RadarVelocityEstimator.h"

using namespace ros;
using namespace gtsam;
using namespace std;

class Vestimate
{
private:
    string radar_topic = "/radar_points";
    float doppler_resolution = 0.05512325;
    float doppler_sigma = doppler_resolution;
    float inlier_thresh;
    bool estimate_inlier = true;
    int robust_estimator = 0;
    // 0: Fair, 1: Huber, 2: Cauchy, 3: Geman-McClure, 4: Welsch, 5: Tukey

    NodeHandle *nh;
    Subscriber sub_radar;
    Publisher pub_v;
    Publisher pub_inlier;
    Publisher pub_twist;

    noiseModel::Robust::shared_ptr robustNoise;

    // radar points memory
    pcl::PointCloud<pcl::PointXYZIS>::Ptr radar_pc;
    pcl::PointCloud<pcl::PointXYZIS>::Ptr inlier_pc;
    sensor_msgs::PointCloud2 inliers_msg;

    Vector3 initial_v;
    noiseModel::Robust::shared_ptr doppler_noise_model;

    float doppler_projection(pcl::PointXYZIS p, Vector3 v);
    void cb_radar(const sensor_msgs::PointCloud2 pc_msg);

public:
    Vestimate(NodeHandle *nh);
    ~Vestimate();
};

Vestimate::Vestimate(NodeHandle *nh)
{

    param::get("~robust_estimator", robust_estimator);
    param::get("~estimate_inlier", estimate_inlier);
    param::get("~doppler_sigma", doppler_sigma);

    // radar init
    radar_pc.reset(new pcl::PointCloud<pcl::PointXYZIS>);

    // radar noise model
    auto unaryNoise = noiseModel::Diagonal::Sigmas(Vector1(0.1));
    auto estimator = noiseModel::mEstimator::Cauchy::Create(0.5);
    robustNoise = noiseModel::Robust::Create(estimator, unaryNoise);

    noiseModel::mEstimator::Base::shared_ptr m_estimator;
    ROS_INFO("estimator: %d", robust_estimator);
    switch (robust_estimator)
    {
    case 0:
        m_estimator = noiseModel::mEstimator::Fair::Create(1.4);
        break;
    case 1:
        m_estimator = noiseModel::mEstimator::Huber::Create(1.345);
        break;
    case 2:
        m_estimator = noiseModel::mEstimator::Cauchy::Create(0.1);
        break;
    case 3:
        m_estimator = noiseModel::mEstimator::GemanMcClure::Create(1.0);
        break;
    case 4:
        m_estimator = noiseModel::mEstimator::Welsch::Create(2.69846);
        break;
    default:
        m_estimator = noiseModel::mEstimator::Tukey::Create(4.6851);
        break;
    }

    doppler_noise_model = noiseModel::Robust::Create(
        m_estimator,
        noiseModel::Diagonal::Sigmas(Vector1(doppler_sigma)));
    initial_v = Vector3(1, 0, 0);
    inlier_thresh = doppler_resolution / 2;

    // ros init
    pub_twist = nh->advertise<geometry_msgs::TwistWithCovarianceStamped>("velocity", 1);
    pub_v = nh->advertise<visualization_msgs::Marker>("velocity_marker", 1);
    pub_inlier = nh->advertise<sensor_msgs::PointCloud2>("inliers", 1);
    sub_radar = nh->subscribe(radar_topic, 1, &Vestimate::cb_radar, this);
}

float Vestimate::doppler_projection(pcl::PointXYZIS p, Vector3 v)
{
    Vector3 unit_r(p.x, p.y, p.z);
    unit_r = -unit_r / unit_r.norm();
    return unit_r.dot(v);
}

void Vestimate::cb_radar(const sensor_msgs::PointCloud2 pc_msg)
{
    // body frame velocity estimation
    pcl::fromROSMsg(pc_msg, *radar_pc);

    NonlinearFactorGraph graph;

    for (pcl::PointXYZIS p : radar_pc->points)
    {
        auto dFactor = DopplerMeasurementFactor(0, p.x, p.y, p.z, p.speed, doppler_noise_model);
        graph.add(dFactor);
    }

    // initial estimate
    Values initial;
    initial.insert(0, initial_v);

    // Robust optimization
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    Vector3 v_est = result.at<Vector3>(0);

    // outlier removal
    if (estimate_inlier)
    {
        graph.resize(0);
        initial.clear();
    }

    inlier_pc.reset(new pcl::PointCloud<pcl::PointXYZIS>);
    for (pcl::PointXYZIS p : radar_pc->points)
    {
        float err = abs(doppler_projection(p, v_est) - p.speed);
        if (err < inlier_thresh)
        {
            inlier_pc->push_back(p);
            auto dFactor = DopplerMeasurementFactor(0, p.x, p.y, p.z, p.speed, nullptr);
            graph.add(dFactor);
        }
    }

    // inlier optimization
    // if (prior_noise_model != NULL)
    // {
    //     graph.addPrior(0, initial_v, prior_noise_model);
    // }

    if (estimate_inlier)
    {
        initial.insert(0, initial_v);
        result = LevenbergMarquardtOptimizer(graph, initial).optimize();
        v_est = result.at<Vector3>(0);
    }

    Marginals marginals(graph, result);
    Matrix radar_cov = marginals.marginalCovariance(0);

    pcl::toROSMsg(*inlier_pc, inliers_msg);
    inliers_msg.header = pc_msg.header;
    pub_inlier.publish(inliers_msg);

    // reset and visualization
    pub_twist.publish(toROSTwist(v_est, radar_cov, pc_msg.header));
    radar_pc->points.clear();
    pub_v.publish(velocityMarker(v_est));
}

Vestimate::~Vestimate()
{
}

int main(int argc, char *argv[])
{
    init(argc, argv, "velocity_estimator");
    NodeHandle nh;
    Vestimate ve(&nh);
    spin();
    return 0;
}