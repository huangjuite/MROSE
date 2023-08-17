#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

// PCL includes
#define PCL_NO_PRECOMPILE
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pclradar/RadarPointType.h>

// GTSAM related includes.
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "utils.h"

using namespace ros;
using namespace gtsam;
using namespace std;

class ScanMatch
{
private:
    string radar_topic = "/radar_points";

    NodeHandle *nh;
    Subscriber sub_radar;
    Publisher pub_v;
    Publisher pub_odom;
    Publisher pub_matched;
    Publisher pub_inlier;
    Publisher pub_pre;

    noiseModel::Robust::shared_ptr robustNoise;

    pcl::PointCloud<pcl::PointXYZSD>::Ptr radar_pc;
    pcl::PointCloud<pcl::PointXYZSD>::Ptr pre_pc;
    pcl::PointCloud<pcl::PointXYZSD>::Ptr inlier_pc;
    sensor_msgs::PointCloud2 inliers_msg;

    Eigen::Matrix4f global_pose;
    pcl::IterativeClosestPoint<pcl::PointXYZSD, pcl::PointXYZSD> icp;

    void cb_radar(const sensor_msgs::PointCloud2 pc_msg);

public:
    ScanMatch(NodeHandle *nh);
    ~ScanMatch();
};

ScanMatch::ScanMatch(NodeHandle *nh)
{
    // radar init
    radar_pc.reset(new pcl::PointCloud<pcl::PointXYZSD>);
    pre_pc.reset(new pcl::PointCloud<pcl::PointXYZSD>);
    inlier_pc.reset(new pcl::PointCloud<pcl::PointXYZSD>);

    // icp init
    global_pose = Eigen::Matrix4f::Identity();
    cout << global_pose << endl;
    icp.setMaxCorrespondenceDistance(1);
    icp.setTransformationEpsilon(0.001);
    icp.setEuclideanFitnessEpsilon(0.001);
    icp.setMaximumIterations(200);
    // icp.setRANSACOutlierRejectionThreshold(1.5);

    // radar noise model
    auto unaryNoise = noiseModel::Diagonal::Sigmas(Vector1(0.1));
    auto estimator = noiseModel::mEstimator::Cauchy::Create(0.5);
    robustNoise = noiseModel::Robust::Create(estimator, unaryNoise);

    // ros init
    pub_v = nh->advertise<visualization_msgs::Marker>("radar_velocity", 1);
    pub_odom = nh->advertise<nav_msgs::Odometry>("dopplerIO", 1);
    pub_inlier = nh->advertise<sensor_msgs::PointCloud2>("inliers", 1);
    pub_pre = nh->advertise<sensor_msgs::PointCloud2>("preframe", 1);
    pub_matched = nh->advertise<sensor_msgs::PointCloud2>("matched", 1);
    sub_radar = nh->subscribe(radar_topic, 1, &ScanMatch::cb_radar, this);
}

void ScanMatch::cb_radar(const sensor_msgs::PointCloud2 pc_msg)
{
    // body frame velocity estimation
    pcl::fromROSMsg(pc_msg, *radar_pc);
    Matrix radar_velocity_cov;
    Vector3 velocity = BodyFrameVelocityEstimation(
        radar_pc, robustNoise, radar_velocity_cov, inlier_pc, 0.05);

    if (pre_pc->empty())
    {
        *pre_pc = *inlier_pc;
        radar_pc->clear();
        inlier_pc->clear();
        return;
    }

    // scan matching
    auto t1 = ros::WallTime::now();
    icp.setInputSource(inlier_pc);
    icp.setInputTarget(pre_pc);

    pcl::PointCloud<pcl::PointXYZSD> result_pc;
    icp.align(result_pc);
    Eigen::Matrix4f tf = icp.getFinalTransformation();
    cout << "==========================" << endl;
    cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
    auto t2 = ros::WallTime::now();
    cout << "time : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;

    // pub pose.
    global_pose = global_pose * tf;
    nav_msgs::Odometry odom = toOdometry(global_pose.cast<double>());
    odom.header.stamp = pc_msg.header.stamp;
    odom.header.seq = pc_msg.header.seq;
    odom.header.frame_id = "map";
    pub_odom.publish(odom);

    // pub points
    pcl::toROSMsg(*inlier_pc, inliers_msg);
    inliers_msg.header.frame_id = "radar";
    inliers_msg.header.stamp = pc_msg.header.stamp;
    pub_inlier.publish(inliers_msg);

    pcl::toROSMsg(*pre_pc, inliers_msg);
    inliers_msg.header.frame_id = "radar";
    inliers_msg.header.stamp = pc_msg.header.stamp;
    pub_pre.publish(inliers_msg);

    pcl::toROSMsg(result_pc, inliers_msg);
    inliers_msg.header.frame_id = "radar";
    inliers_msg.header.stamp = pc_msg.header.stamp;
    pub_matched.publish(inliers_msg);

    // reset and visualization
    *pre_pc = *inlier_pc;
    radar_pc->clear();
    inlier_pc->clear();
    pub_v.publish(velocityMarker(velocity));
}

ScanMatch::~ScanMatch()
{
}

int main(int argc, char *argv[])
{
    init(argc, argv, "velocity_estimator");
    NodeHandle nh;
    ScanMatch sm(&nh);
    spin();
    return 0;
}