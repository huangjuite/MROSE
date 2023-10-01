// std includes
#include <string>
#include <vector>
#include <filesystem>
#include <memory>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <radar_odom/extrinsicConfig.h>
#include <dynamic_reconfigure/server.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pclradar/RadarPointType.h>

#include "RadarVelocityEstimatorCeres.h"
#include "RadarVelocityEstimatorEigenWeighted.h"

#include "radar_odom/Prediction.h"

#include "rplrio_types.h"
#include "utils.h"
#include "params.h"

int main(int argc, char *argv[])
{
    std::cout << std::fixed << std::setprecision(6);

    ros::init(argc, argv, "radar_cov");
    ros::NodeHandle nh;
    std::string bag_folder;
    ros::param::get("~bag_folder", bag_folder);

    rplrio::Params params = rplrio::get_params(nh);
    std::vector<Eigen::Vector3d> body_velocity, radar_velocity, radar_velocity_weighted;
    std::vector<double> stamps;

    std::vector<std::string> bags;
    std::vector<std::string> topics;

    for (const auto &entry : std::filesystem::directory_iterator(bag_folder))
        bags.emplace_back(entry.path().string());
    std::sort(bags.begin(), bags.end());

    for (auto r : params.radar_list)
    {
        auto radarParam = params.radars[r];
        topics.push_back(radarParam.topic);
    }

    std::ofstream ofiles;
    ofiles.open("/tmp/radar_v_std.csv");
    ofiles << std::fixed << std::setprecision(15);
    ofiles << "stamp,radar,std_x,std_y,std_z" << std::endl;

    RadarVelocityEstimatorCeres v_opt;

    for (auto b : bags)
    {
        rosbag::Bag bag;
        bag.open(b, rosbag::bagmode::Read);

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        foreach (rosbag::MessageInstance const m, view)
        {
            sensor_msgs::PointCloud2::ConstPtr radar_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (radar_msg != NULL)
            {

                double current_t = radar_msg->header.stamp.toSec();

                std::cout << "\r" << current_t << ", " << radar_msg->header.frame_id << std::flush;

                pcl::PointCloud<pcl::PointXYZSD>::Ptr radar_pc;
                radar_pc.reset(new pcl::PointCloud<pcl::PointXYZSD>);
                pcl::fromROSMsg(*radar_msg, *radar_pc);
                v_opt.set_initial_velocity(Eigen::Vector3d(1, 0, 0));
                gtsam::Vector3 measured_v = v_opt.optimize(radar_pc);
                gtsam::Matrix3 radar_velocity_cov = v_opt.get_covariance(
                    radar_pc, measured_v);

                auto rp = params.radars[radar_msg->header.frame_id];
                measured_v = rp.R_r_b * measured_v;
                radar_velocity_cov = rp.R_r_b * radar_velocity_cov * rp.R_r_b.transpose();

                ofiles << current_t << ", "
                       << radar_msg->header.frame_id << ", "
                       << sqrt(radar_velocity_cov(0, 0)) << ", "
                       << sqrt(radar_velocity_cov(1, 1)) << ", "
                       << sqrt(radar_velocity_cov(2, 2)) << std::endl;
            }
        }

        bag.close();
    }

    ofiles.close();
    std::cout << std::endl;

    return 0;
}