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
#include "RadarVelocityEstimatorCelestial.h"

using namespace ros;
using namespace gtsam;
using namespace std;

class Vestimate
{
private:
    string radar_topic = "/radar_points";
    float doppler_resolution = 0.05512325;
    float doppler_sigma = doppler_resolution;
    double angle_sigma = 2 * M_PI / 256.0;
    float inlier_thresh;
    bool estimate_inlier = true;
    int robust_estimator = 0;
    // 0: Fair, 1: Huber, 2: Cauchy, 3: Geman-McClure, 4: Welsch, 5: Tukey

    NodeHandle *nh;
    Subscriber sub_radar;
    Publisher pub_v;
    Publisher pub_inlier;

    RadarVelocityEstimatorCelestial *v_estimator;

    // radar points memory
    pcl::PointCloud<pcl::PointXYZIS>::Ptr radar_pc;
    pcl::PointCloud<pcl::PointXYZIS>::Ptr inlier_pc;
    sensor_msgs::PointCloud2 inliers_msg;

    Vector3 initial_v;

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

    auto doppler_noise_model = noiseModel::Robust::Create(
        m_estimator,
        noiseModel::Diagonal::Sigmas(Vector1(doppler_sigma)));
    initial_v = Vector3(1, 0, 0);
    inlier_thresh = doppler_resolution / 2;

    auto angle_noise_modle = noiseModel::Diagonal::Sigmas(
        Vector2(angle_sigma, angle_sigma));

    v_estimator = new RadarVelocityEstimatorCelestial(doppler_noise_model, angle_noise_modle);
    v_estimator->set_inlier_thresh(inlier_thresh);

    // ros init
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

    v_estimator->set_initial_velocity(Vector3(1, 0, 0));
    auto v_est = v_estimator->optimize(radar_pc);
    inlier_pc = v_estimator->get_inlier(radar_pc, v_est);

    auto cov = v_estimator->get_covariance(inlier_pc, v_est);
    cout << "===============" << endl;
    cout << cov << endl;

    pcl::toROSMsg(*inlier_pc, inliers_msg);
    inliers_msg.header = pc_msg.header;
    pub_inlier.publish(inliers_msg);

    // reset and visualization
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