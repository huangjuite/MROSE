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

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pclradar/RadarPointType.h>

#include <torch/script.h>

#include "types.h"
#include "utils.h"
#include "params.h"

int main(int argc, char *argv[])
{
    std::cout << std::fixed << std::setprecision(6);

    torch::Tensor tensor =
        torch::rand({2, 3}, torch::TensorOptions().device(torch::kCUDA));
    std::cout << tensor << std::endl;

    std::string model_path = "/home/rpl/Downloads/model_cpp.pt";
    torch::jit::script::Module module;
    try
    {
        // Deserialize the ScriptModule from a file using torch::jit::load().
        module = torch::jit::load(model_path);
    }
    catch (const c10::Error &e)
    {
        std::cerr << "error loading the model\n";
        return -1;
    }
    // std::cout << module.to()
    std::cout << "ok\n";

    // ros::init(argc, argv, "radar_transofrmer");
    // ros::NodeHandle nh;
    // std::string bag_folder;
    // ros::param::get("~bag_folder", bag_folder);

    // std::string csv_path;
    // ros::param::get("~gt_path", csv_path);

    // rplrio::Params params = rplrio::get_params(nh);
    // std::vector<Eigen::Vector3d> body_velocity, radar_velocity;
    // std::vector<double> stamps;

    // std::vector<std::string> bags;
    // std::vector<std::string> topics;
    // for (const auto &entry : std::filesystem::directory_iterator(bag_folder))
    //     bags.emplace_back(entry.path().string());
    // std::sort(bags.begin(), bags.end());
    // topics.push_back(params.radar_topic);

    // for (auto b : bags)
    // {
    //     rosbag::Bag bag;
    //     bag.open(b, rosbag::bagmode::Read);

    //     rosbag::View view(bag, rosbag::TopicQuery(topics));

    //     foreach (rosbag::MessageInstance const m, view)
    //     {
    //         sensor_msgs::PointCloud2::ConstPtr radar_msg = m.instantiate<sensor_msgs::PointCloud2>();
    //         if (radar_msg != NULL)
    //         {
    //             pcl::PointCloud<pcl::PointXYZIS>::Ptr radar_pc;
    //             radar_pc.reset(new pcl::PointCloud<pcl::PointXYZIS>);
    //             pcl::fromROSMsg(*radar_msg, *radar_pc);
    //             double current_t = radar_msg->header.stamp.toSec();

    //         }
    //     }

    //     bag.close();
    // }

    return 0;
}