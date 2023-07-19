#ifndef UTILS_H
#define UTILS_H

#include <fstream>
#include <vector>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pclradar/RadarPointType.h>

// ROS includes
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf/tf.h>

// GTSAM includes.
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/navigation/ImuBias.h>

#include "types.h"
#include "params.h"


visualization_msgs::Marker velocityMarker(gtsam::Vector3 velocity, gtsam::Vector3 position = gtsam::Vector3(0, 0, 0),
                                          std::string frame_id = "radar", std::string ns = "radar_doppler")
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    geometry_msgs::Point init_p, v_p;
    init_p.x = position(0);
    init_p.y = position(1);
    init_p.z = position(2);
    v_p.x = velocity(0) + position(0);
    v_p.y = velocity(1) + position(1);
    v_p.z = velocity(2) + position(2);
    marker.points.push_back(init_p);
    marker.points.push_back(v_p);
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1;
    marker.color.a = 1;

    return marker;
}

boost::array<double, 36> toROSCov(Eigen::MatrixXd cov)
{
    boost::array<double, 36> rosCov;
    Eigen::MatrixXd cov_matrix = Eigen::Matrix<double, 6, 6>();
    cov_matrix.block(0, 0, 3, 3) = cov.block(3, 3, 3, 3);
    cov_matrix.block(3, 3, 3, 3) = cov.block(0, 0, 3, 3);
    cov_matrix.block(0, 3, 3, 3) = cov.block(3, 0, 3, 3);
    cov_matrix.block(3, 0, 3, 3) = cov.block(0, 3, 3, 3);
    double *data = cov_matrix.data();
    for (uint i = 0; i < 36; i++)
        rosCov[i] = data[i];

    return rosCov;
}

Eigen::MatrixXd fromROSCov(boost::array<double, 36> ros_cov)
{
    Eigen::MatrixXd cov_matrix = Eigen::Matrix<double, 6, 6>();
    Eigen::MatrixXd cov = Eigen::Matrix<double, 6, 6>();
    double *data = cov.data();
    for (uint i = 0; i < 36; i++)
        data[i] = ros_cov[i];

    cov_matrix.block(0, 0, 3, 3) = cov.block(3, 3, 3, 3);
    cov_matrix.block(3, 3, 3, 3) = cov.block(0, 0, 3, 3);
    cov_matrix.block(0, 3, 3, 3) = cov.block(3, 0, 3, 3);
    cov_matrix.block(3, 0, 3, 3) = cov.block(0, 3, 3, 3);

    return cov_matrix;
}

Eigen::MatrixXd fromROSCov_2d(boost::array<double, 36> ros_cov)
{
    Eigen::MatrixXd cov = Eigen::Matrix<double, 6, 6>();
    double *data = cov.data();
    for (uint i = 0; i < 36; i++)
        data[i] = ros_cov[i];

    Eigen::MatrixXd cov2d = cov.block(0, 0, 3, 3);
    cov2d.block(0, 2, 2, 1) = cov.block(0, 5, 2, 1);
    cov2d.block(2, 0, 1, 2) = cov.block(5, 0, 1, 2);
    cov2d(2, 2) = cov(5, 5);

    return cov2d;
}

nav_msgs::Odometry toOdometry(Eigen::Matrix4d global_pose, gtsam::Matrix covariance = gtsam::Matrix())
{
    nav_msgs::Odometry odom;
    Eigen::Matrix3d orien = global_pose.block(0, 0, 3, 3);
    Eigen::Quaterniond qd(orien);

    odom.pose.pose.position.x = global_pose(0, 3);
    odom.pose.pose.position.y = global_pose(1, 3);
    odom.pose.pose.position.z = global_pose(2, 3);
    odom.pose.pose.orientation.x = qd.x();
    odom.pose.pose.orientation.y = qd.y();
    odom.pose.pose.orientation.z = qd.z();
    odom.pose.pose.orientation.w = qd.w();

    if (covariance.size() == 36)
    {
        odom.pose.covariance = toROSCov(covariance);
    }

    return odom;
}

geometry_msgs::TwistWithCovarianceStamped toROSTwist(
    gtsam::Vector3 v, gtsam::Matrix cov, std_msgs::Header header)
{
    geometry_msgs::TwistWithCovarianceStamped twist;
    twist.header = header;
    twist.twist.twist.linear.x = v.x();
    twist.twist.twist.linear.y = v.y();
    twist.twist.twist.linear.z = v.z();

    for (uint i = 0; i < 3; i++)
        for (uint j = 0; j < 3; j++)
            twist.twist.covariance[i * 6 + j] = cov(i, j);

    return twist;
}

geometry_msgs::Twist toROStwist(gtsam::imuBias::ConstantBias bias)
{
    geometry_msgs::Twist twist;
    gtsam::Vector3 acc = bias.accelerometer();
    gtsam::Vector3 gyr = bias.gyroscope();
    twist.linear.x = acc(0);
    twist.linear.y = acc(1);
    twist.linear.z = acc(2);
    twist.angular.x = gyr(0);
    twist.angular.y = gyr(1);
    twist.angular.z = gyr(2);
    return twist;
}

gtsam::Pose3 toPose3(const gtsam::Pose2 gtsam_pose)
{
    return gtsam::Pose3(
        gtsam::Rot3::Yaw(gtsam_pose.theta()),
        gtsam::Point3(gtsam_pose.x(), gtsam_pose.y(), 0));
}

gtsam::Pose3 toPose3(const nav_msgs::Odometry odom)
{
    return gtsam::Pose3(
        gtsam::Rot3(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                    odom.pose.pose.orientation.y, odom.pose.pose.orientation.z),
        gtsam::Point3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
}

gtsam::Pose3 toPose3(const geometry_msgs::PoseStamped pst)
{
    return gtsam::Pose3(
        gtsam::Rot3(pst.pose.orientation.w, pst.pose.orientation.x,
                    pst.pose.orientation.y, pst.pose.orientation.z),
        gtsam::Point3(pst.pose.position.x, pst.pose.position.y, pst.pose.position.z));
}

geometry_msgs::Point toROSPoint(gtsam::Point3 gtsam_point)
{
    geometry_msgs::Point point;
    point.x = gtsam_point.x();
    point.y = gtsam_point.y();
    point.z = gtsam_point.z();
    return point;
}

geometry_msgs::Point toROSPoint(gtsam::Point2 gtsam_point)
{
    geometry_msgs::Point point;
    point.x = gtsam_point.x();
    point.y = gtsam_point.y();
    point.z = 0;
    return point;
}

geometry_msgs::Pose toROSPose(gtsam::Pose3 gtsam_pose)
{
    geometry_msgs::Pose pose;
    auto trans = gtsam_pose.translation();
    auto rotat = gtsam_pose.rotation().toQuaternion();
    pose.position.x = trans.x();
    pose.position.y = trans.y();
    pose.position.z = trans.z();
    pose.orientation.w = rotat.w();
    pose.orientation.x = rotat.x();
    pose.orientation.y = rotat.y();
    pose.orientation.z = rotat.z();
    return pose;
}

geometry_msgs::Pose toROSPose(gtsam::Pose2 gtsam_pose)
{
    return toROSPose(toPose3(gtsam_pose));
}

geometry_msgs::PoseStamped toROSPoseStamped(const gtsam::Pose3 gtsam_pose, std_msgs::Header header)
{
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.pose = toROSPose(gtsam_pose);
    return pose_stamped;
}

geometry_msgs::PoseStamped toROSPoseStamped(const gtsam::Pose2 gtsam_pose, std_msgs::Header header)
{
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.pose = toROSPose(toPose3(gtsam_pose));
    return pose_stamped;
}

pcl::PointCloud<pcl::PointXYZIS> from_xrio_type(pcl::PointCloud<pcl::PointXYZSNV> &radar_xrio)
{
    pcl::PointCloud<pcl::PointXYZIS> radar_rpl;
    radar_rpl.resize(radar_xrio.size());
    for (size_t i = 0; i < radar_xrio.points.size(); i++)
    {
        radar_rpl.points[i].x = radar_xrio.points[i].x;
        radar_rpl.points[i].y = radar_xrio.points[i].y;
        radar_rpl.points[i].z = radar_xrio.points[i].z;
        radar_rpl.points[i].intensity = radar_xrio.points[i].snr_db;
        radar_rpl.points[i].speed = radar_xrio.points[i].v_doppler_mps;
    }

    return radar_rpl;
}

Eigen::Matrix4f toMatrix(tf::StampedTransform tf_ros)
{
    tf::Quaternion q = tf_ros.getRotation();
    tf::Vector3 p = tf_ros.getOrigin();
    return gtsam::Pose3(gtsam::Rot3(q.w(), q.x(), q.y(), q.z()), gtsam::Point3(p.x(), p.y(), p.z())).matrix().cast<float>();
}

void logdB_SNR(pcl::PointCloud<pcl::PointXYZIS>::Ptr pc)
{
    /**
     * @brief take log on the signal to noise ratio value
     * @param pc pcl::PointCloud<pcl::PointXYZIS>::Ptr change SNR inline
     */
    for (size_t i = 0; i < pc->size(); i++)
    {
        pc->points[i].intensity = log(pc->points[i].intensity);
    }
}

void logdB_SNR(pcl::PointCloud<pcl::PointXYZI>::Ptr pc)
{
    /**
     * @brief take log on the signal to noise ratio value
     * @param pc pcl::PointCloud<pcl::PointXYZI>::Ptr change SNR inline
     */
    for (size_t i = 0; i < pc->size(); i++)
    {
        pc->points[i].intensity = log(pc->points[i].intensity);
    }
}

namespace rplrio
{

    Params get_params(ros::NodeHandle &nh)
    {
        Params param;

        // odom
        std::string radar_, imu_;
        nh.getParam("/odom/imu", imu_);
        nh.getParam("/odom/radar", radar_);
        nh.getParam(radar_ + std::string("/rostopic"), param.radar_topic);
        nh.getParam(imu_ + std::string("/rostopic"), param.imu_topic);

        // imu
        nh.getParam(imu_ + std::string("/accelerometer_noise_density"), param.acc_noise);
        nh.getParam(imu_ + std::string("/accelerometer_random_walk"), param.acc_rw);
        nh.getParam(imu_ + std::string("/gyroscope_noise_density"), param.gyro_noise);
        nh.getParam(imu_ + std::string("/gyroscope_random_walk"), param.gyro_rw);
        nh.getParam(imu_ + std::string("/gravity"), param.gravity);

        XmlRpc::XmlRpcValue tf;
        nh.getParam(imu_ + std::string("/T_i_b"), tf);
        for (uint i = 0; i < tf.size(); i++)
        {
            for (uint j = 0; j < tf[i].size(); j++)
            {
                param.T_i_b(i, j) = tf[i][j];
            }
        }
        param.R_i_b = param.T_i_b.block(0, 0, 3, 3);

        // radar
        nh.getParam(radar_ + std::string("/doppler_sigma"), param.doppler_std);
        nh.getParam(radar_ + std::string("/azimuth_sigma"), param.azimuth_std);
        nh.getParam(radar_ + std::string("/elevation_sigma"), param.elevation_std);
        nh.getParam(radar_ + std::string("/T_r_i"), tf);
        for (uint i = 0; i < tf.size(); i++)
        {
            for (uint j = 0; j < tf[i].size(); j++)
            {
                param.T_r_i(i, j) = tf[i][j];
            }
        }
        param.T_r_b = param.T_i_b * param.T_r_i;
        param.R_r_b = param.T_r_b.block(0, 0, 3, 3);

        // initializer
        nh.getParam("/odom/static_init_window", param.init_window);
        nh.getParam("/odom/static_init_threshold", param.init_thres);
        nh.getParam("/odom/optimization_window", param.lag);

        // loop close
        nh.getParam("/loop_close/enable_loop_close", param.enable_loop_close);
        nh.getParam("/loop_close/loop_close_rate", param.loop_close_rate);
        nh.getParam("/loop_close/loop_ignore", param.loop_ignore);
        nh.getParam("/loop_close/key_frame_search", param.key_frame_search);
        nh.getParam("/loop_close/loop_distance_thresh", param.loop_distance_thresh);
        nh.getParam("/loop_close/process_voxel_size", param.process_voxel_size);
        nh.getParam("/loop_close/ndt_voxel_size", param.ndt_voxel_size);
        nh.getParam("/loop_close/ndt_thread", param.ndt_thread);
        nh.getParam("/loop_close/snr_thresh", param.snr_thresh);

        // mapping
        nh.getParam("/mapping/map_snr_thresh", param.map_snr_thresh);
        nh.getParam("/mapping/render_rate", param.render_rate);

        return param;
    }

    

}

#endif