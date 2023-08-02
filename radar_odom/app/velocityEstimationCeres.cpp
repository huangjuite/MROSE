#include <string>
#include <vector>
#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pclradar/RadarPointType.h>

#include <Eigen/Dense>
#include <ceres/ceres.h>

#include "RadarVelocityEstimatorCeres.h"
#include "RadarVelocityEstimatorCeresHCS.h"
#include "utils.h"
#include "params.h"

using namespace std;
using namespace ros;

class VestimateCeres
{
private:
    rplrio::Params param = rplrio::get_params(*nh);
    float inlier_thresh;

    NodeHandle *nh;
    Subscriber sub_radar;
    Publisher pub_v;
    Publisher pub_inlier;
    Publisher pub_twist;

    // radar points memory
    pcl::PointCloud<pcl::PointXYZIS>::Ptr radar_pc;
    pcl::PointCloud<pcl::PointXYZIS>::Ptr inlier_pc;
    sensor_msgs::PointCloud2 inliers_msg;

    // RadarVelocityEstimatorCeres v_estimator;
    std::shared_ptr<RadarVelocityEstimatorCeresHCS> v_estimator;

    void cb_radar(const sensor_msgs::PointCloud2 pc_msg);

public:
    VestimateCeres(NodeHandle *nh);
    ~VestimateCeres();
};

VestimateCeres::VestimateCeres(NodeHandle *nh)
{

    // radar init
    auto radar_param = param.radars[param.radar_list[0]];

    radar_pc.reset(new pcl::PointCloud<pcl::PointXYZIS>);
    v_estimator.reset(new RadarVelocityEstimatorCeresHCS());
    v_estimator->set_inlier_thresh(param.doppler_inlier_thresh);
    v_estimator->set_uncertainty(
        radar_param.doppler_std, radar_param.azimuth_std, radar_param.elevation_std);

    // ros init
    pub_twist = nh->advertise<geometry_msgs::TwistWithCovarianceStamped>("velocity", 1);
    pub_v = nh->advertise<visualization_msgs::Marker>("velocity_marker", 1);
    pub_inlier = nh->advertise<sensor_msgs::PointCloud2>("inliers", 1);
    sub_radar = nh->subscribe(radar_param.topic, 1, &VestimateCeres::cb_radar, this);
}

void VestimateCeres::cb_radar(const sensor_msgs::PointCloud2 pc_msg)
{
    // body frame velocity estimation
    pcl::fromROSMsg(pc_msg, *radar_pc);

    v_estimator->set_initial_velocity(Eigen::Vector3d(1, 0, 0));
    Eigen::Vector3d v_est = v_estimator->optimize(radar_pc);
    inlier_pc = v_estimator->get_inlier(radar_pc, v_est);
    Eigen::Matrix3d radar_cov = v_estimator->get_covariance(radar_pc, v_est);

    pcl::toROSMsg(*inlier_pc, inliers_msg);
    inliers_msg.header = pc_msg.header;
    pub_inlier.publish(inliers_msg);

    // reset and visualization
    pub_twist.publish(toROSTwist(v_est, radar_cov, pc_msg.header));
    radar_pc->points.clear();
    pub_v.publish(velocityMarker(v_est));
}

VestimateCeres::~VestimateCeres()
{
}

int main(int argc, char *argv[])
{
    init(argc, argv, "velocity_estimator");
    NodeHandle nh;
    VestimateCeres ve(&nh);
    spin();
    return 0;
}