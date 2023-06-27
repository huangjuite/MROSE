// std includes
#include <string>
#include <vector>
#include <filesystem>

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pclradar/RadarPointType.h>

// GTSAM includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "DopplerMeasurementFactor.h"
#include "VelocityMeasurementFactor.h"
#include "VelocityPoseMeasurementFactor.h"
#include "VelocityPoseExtrinsicFactor.h"
#include "AltitudeFactor.h"
#include "RadarVelocityEstimator.h"

#include "utils.h"
#include "params.h"

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::E;
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

class DopplerIOex
{
private:
    rplrio::Params params;

    double pre_imu_t = 0;
    int imu_count = 0;
    bool correct_now = false;

    ros::NodeHandle *nh;
    ros::Subscriber sub_radar;
    ros::Subscriber sub_imu;
    ros::Publisher pub_odom;
    ros::Publisher pub_v;
    ros::Publisher pub_radar;
    ros::Publisher pub_path;
    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    ros::ServiceServer calibrate_srv;

    // radar
    RadarVelocityEstimator *velocity_estimator;

    // imu settings
    std::shared_ptr<PreintegrationType> preintegrated;

    // states
    Values initial_values;
    Values cache_results;
    NavState prev_state;
    NavState prop_state;
    imuBias::ConstantBias prev_bias;
    FixedLagSmoother::KeyTimestampMap key_time_stamps;

    // noise models
    noiseModel::Diagonal::shared_ptr pose_noise_model;
    noiseModel::Isotropic::shared_ptr velocity_noise_model;
    noiseModel::Isotropic::shared_ptr bias_noise_model;

    // optimizer & graph
    FixedLagSmoother::KeyTimestampMap key_timestamps;
    IncrementalFixedLagSmoother *smootherISAM2;
    NonlinearFactorGraph *graph;

    void feed_radar(const sensor_msgs::PointCloud2 pc_msg);
    void feed_imu(const sensor_msgs::Imu imu);

public:
    DopplerIOex(ros::NodeHandle *nh);
    ~DopplerIOex();

    uint step = 0;
    NonlinearFactorGraph offline_graph;
    Values result;
    Rot3 tf_radar2imu;
    map<uint, Vector3> radar_velocity_cache;
    map<uint, Matrix> cov_cache;
    noiseModel::Diagonal::shared_ptr radar_velocity_noise;
    noiseModel::Diagonal::shared_ptr altitude_noise_model;
};

DopplerIOex::DopplerIOex(ros::NodeHandle *nh)
{
}

void DopplerIOex::feed_radar(const sensor_msgs::PointCloud2 pc_msg)
{
}

void DopplerIOex::feed_imu(const sensor_msgs::Imu imu_msg)
{
}

DopplerIOex::~DopplerIOex()
{
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "radar_inertial_extrinsic");
    ros::NodeHandle nh;
    DopplerIOex dio(&nh);

    string bag_folder;
    ros::param::get("~bag_folder", bag_folder);

    std::vector<string> bags;
    for (const auto &entry : std::filesystem::directory_iterator(bag_folder))
        bags.emplace_back(entry.path().string());
    std::sort(bags.begin(), bags.end());

    for (auto b : bags)
    {
        rosbag::Bag bag;
        bag.open(b, rosbag::bagmode::Read);
        
        // rosbag::View view(bag, )

        bag.close();
    }

    // // extrinsic optimization
    // altitude_noise_model = noiseModel::Diagonal::Sigmas(Vector1(0.05));
    // auto extrinsic_noise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));
    // offline_graph.addPrior(E(0), tf_radar2imu, extrinsic_noise);
    // cache_results.insert(E(0), tf_radar2imu);

    // Values calibrate_result;

    // for (uint i = 1; i <= step; i++)
    // {

    //     auto rv_noise = noiseModel::Gaussian::Covariance(cov_cache[i]);
    //     // auto rv_noise = noiseModel::Diagonal::Sigmas(Vector3(0.05, 0.05, 0.05));
    //     offline_graph.add(VelocityPoseExtrinsicFactor(
    //         V(i), X(i), E(0), radar_velocity_cache[i], rv_noise));
    //     offline_graph.add(AltitudeFactor(X(i), 0, altitude_noise_model));
    // }

    // calibrate_result = LevenbergMarquardtOptimizer(offline_graph, cache_results).optimize();

    // Rot3 extrinsic = calibrate_result.at<Rot3>(E(0));
    // tf_radar2imu.print("before");
    // extrinsic.print("after");
    // auto q = extrinsic.toQuaternion();
    // auto e = extrinsic.ypr();
    // printf("tf radar to imu\n");
    // printf("x,y,z,w: %.4f, %.4f, %.4f, %.4f \n", q.x(), q.y(), q.z(), q.w());
    // printf("y,p,r: %.4f, %.4f, %.4f \n", e[0], e[1], e[2]);

    // extrinsic = extrinsic.inverse();
    // q = extrinsic.toQuaternion();
    // e = extrinsic.ypr();
    // printf("\ntf imu to radar\n");
    // printf("x,y,z,w: %.4f, %.4f, %.4f, %.4f \n", q.x(), q.y(), q.z(), q.w());
    // printf("y,p,r: %.4f, %.4f, %.4f \n", e[0], e[1], e[2]);

    // nav_msgs::Path path;
    // path.header.frame_id = "map";
    // for (uint i = 0; i < step; i++)
    // {
    //     path.poses.push_back(toROSPoseStamped(calibrate_result.at<Pose3>(X(i)), path.header));
    // }
    // pub_path.publish(path);

    return 0;
}