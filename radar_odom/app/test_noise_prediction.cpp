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
#include "RadarVelocityEstimatorCeresWeighted.h"
#include "RadarVelocityEstimatorEigenWeighted.h"

#include "radar_odom/Prediction.h"

#include "rplrio_types.h"
#include "utils.h"
#include "params.h"

std::vector<rplrio::Odom> odom_csv(std::string path)
{
    std::vector<rplrio::Odom> gt;

    std::ifstream file;
    file.open(path, std::ios::in);

    std::string line;
    std::getline(file, line, '\n');

    gtsam::Pose3 tf(gtsam::Rot3::Yaw(M_PI / 2), gtsam::Point3(0, 0, 0));

    while (file.good())
    {
        std::string line;
        std::getline(file, line, '\n');

        if (line.empty())
            continue;

        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream ss(line);

        double stamp;
        ss >> stamp;
        uint seq;
        ss >> seq;

        std::vector<double> v;
        std::copy(std::istream_iterator<double>(ss),
                  std::istream_iterator<double>(),
                  std::back_inserter(v));

        // tx, ty, tz, qx, qy, qz, qw, vx, vy, vz, ax, ay, az

        gtsam::Pose3 pose = tf * gtsam::Pose3(gtsam::Rot3(v[6], v[3], v[4], v[5]),
                                              gtsam::Point3(v[0], v[1], v[2]));

        gt.emplace_back(rplrio::Odom(stamp, seq, pose,
                                     Eigen::Vector3d(v[7], v[8], v[9]),
                                     Eigen::Vector3d(v[10], v[11], v[12])));
    }

    return gt;
}

class ExtrinsicFactor : public gtsam::NoiseModelFactor1<gtsam::Rot3>
{
private:
    gtsam::Vector3 radar_velocity;
    gtsam::Vector3 body_velocity;

public:
    ExtrinsicFactor(){};

    ExtrinsicFactor(gtsam::Key rotKey, const gtsam::Vector3 &radar_v,
                    const gtsam::Vector3 &body_v, const gtsam::SharedNoiseModel &model)
        : gtsam::NoiseModelFactor1<gtsam::Rot3>(model, rotKey),
          radar_velocity(radar_v), body_velocity(body_v){};

    gtsam::Vector evaluateError(const gtsam::Rot3 &pose, boost::optional<gtsam::Matrix &> H = boost::none) const override
    {
        gtsam::Vector3 err = pose.rotate(radar_velocity, H) - body_velocity;
        return err;
    }
};

int main(int argc, char *argv[])
{
    std::cout << std::fixed << std::setprecision(6);

    ros::init(argc, argv, "radar_extrinsic");
    ros::NodeHandle nh;
    std::string bag_folder;
    ros::param::get("~bag_folder", bag_folder);

    std::string csv_path;
    ros::param::get("~gt_path", csv_path);
    auto odom_vio = odom_csv(csv_path);

    auto predict_noise = nh.serviceClient<radar_odom::Prediction>("radar_uncertainty");

    rplrio::Params params = rplrio::get_params(nh);
    std::vector<Eigen::Vector3d> body_velocity, radar_velocity, radar_velocity_weighted;
    std::vector<double> stamps;

    std::vector<std::string> bags;
    std::vector<std::string> topics;
    for (const auto &entry : std::filesystem::directory_iterator(bag_folder))
        bags.emplace_back(entry.path().string());
    std::sort(bags.begin(), bags.end());
    topics.push_back(params.radar_topic);

    RadarVelocityEstimatorCeres v_opt;
    // RadarVelocityEstimatorCeresWeighted v_opt_noise;
    RadarVelocityEstimatorEigenWeighted v_opt_noise;

    std::cout << "input any to start:\n";
    std::string tmp;
    std::cin >> tmp;

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

                auto it = std::lower_bound(odom_vio.begin(), odom_vio.end(), current_t);

                if (it == odom_vio.begin() || it == odom_vio.end())
                    continue;

                double ratio = (current_t - (it - 1)->stamp) / (it->stamp - (it - 1)->stamp);
                auto body_v = ratio * it->v + (1 - ratio) * (it - 1)->v;
                double inter_t = ratio * it->stamp + (1 - ratio) * (it - 1)->stamp;

                radar_odom::Prediction req;
                req.request.input = *radar_msg;

                if (predict_noise.call(req))
                {
                    // std::cout << "\033c";
                    std::cout << current_t << ", " << ratio << std::endl;

                    pcl::PointCloud<pcl::PointXYZISW>::Ptr radar_pc_noise;
                    radar_pc_noise.reset(new pcl::PointCloud<pcl::PointXYZISW>);
                    pcl::fromROSMsg(req.response.output, *radar_pc_noise);
                    v_opt_noise.set_initial_velocity(Eigen::Vector3d(1, 0, 0));
                    gtsam::Vector3 measured_v_weighted = v_opt_noise.optimize(radar_pc_noise);
                    gtsam::Matrix3 radar_velocity_cov_weighted = v_opt_noise.get_covariance(
                        radar_pc_noise, measured_v_weighted);

                    measured_v_weighted[0] = radar_velocity_cov_weighted(0,0);
                    measured_v_weighted[1] = radar_velocity_cov_weighted(1,1);
                    measured_v_weighted[2] = radar_velocity_cov_weighted(2,2);

                    pcl::PointCloud<pcl::PointXYZIS>::Ptr radar_pc;
                    radar_pc.reset(new pcl::PointCloud<pcl::PointXYZIS>);
                    pcl::fromROSMsg(*radar_msg, *radar_pc);
                    v_opt.set_initial_velocity(Eigen::Vector3d(1, 0, 0));
                    gtsam::Vector3 measured_v = v_opt.optimize(radar_pc);
                    gtsam::Matrix3 radar_velocity_cov = v_opt.get_covariance(
                        radar_pc, measured_v);

                    measured_v[0] = radar_velocity_cov(0,0);
                    measured_v[1] = radar_velocity_cov(1,1);
                    measured_v[2] = radar_velocity_cov(2,2);

                    stamps.emplace_back(current_t);
                    body_velocity.emplace_back(body_v);
                    radar_velocity.emplace_back(measured_v);
                    radar_velocity_weighted.emplace_back(measured_v_weighted);
                }
            }
        }

        bag.close();
    }

    std::ofstream odom_file;
    odom_file.open("/tmp/odom_tmp.csv");
    odom_file << std::fixed << std::setprecision(15);
    // odom_file << "stamp, vx, vy, vz, rx, ry, rz" << std::endl;

    for (uint i = 0; i < stamps.size(); i++)
    {
        gtsam::Vector3 ceres = params.R_r_b * radar_velocity[i];
        gtsam::Vector3 weighted = radar_velocity_weighted[i];

        odom_file << stamps[i] << ", ";
        odom_file << ceres[0] << ", ";
        odom_file << ceres[1] << ", ";
        odom_file << ceres[2] << ", ";
        odom_file << body_velocity[i][0] << ", ";
        odom_file << body_velocity[i][1] << ", ";
        odom_file << body_velocity[i][2] << ", ";
        odom_file << weighted[0] << ", ";
        odom_file << weighted[1] << ", ";
        odom_file << weighted[2] << std::endl;
    }

    odom_file.close();

    return 0;
}