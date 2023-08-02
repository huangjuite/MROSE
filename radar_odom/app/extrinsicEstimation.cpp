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

// GTSAM includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "VelocityMeasurementFactor.h"
#include "VelocityPoseMeasurementFactor.h"
#include "VelocityPoseExtrinsicFactor.h"
#include "AltitudeFactor.h"

#include "RadarVelocityEstimatorCeresHCS.h"
#include "RadarVelocityEstimatorCeres.h"
#include "StaticInitializer.hpp"

#include "rplrio_types.h"
#include "utils.h"
#include "params.h"

using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::E;
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

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

class DopplerIOex
{
private:
    double pre_imu_t = 0;
    int imu_count = 0;
    bool correct_now = false;

    bool initialized = false;
    std::deque<std::shared_ptr<rplrio::IMU>> imu_que;
    std::shared_ptr<StaticInitializer> initializer;

    ros::NodeHandle *nh;
    ros::Publisher pub_odom;
    ros::Publisher pub_v;
    ros::Publisher pub_radar;
    tf::TransformBroadcaster br;
    ros::ServiceServer calibrate_srv;

    // radar
    std::unique_ptr<RadarVelocityEstimatorCeres> velocity_estimator;
    // std::unique_ptr<RadarVelocityEstimatorCeresHCS> velocity_estimator;
    std::deque<sensor_msgs::PointCloud2::ConstPtr> radar_que;

    // imu settings
    std::shared_ptr<gtsam::PreintegrationType> preintegrated;

    // states
    gtsam::Values initial_values;
    gtsam::NavState prev_state;
    gtsam::NavState prop_state;
    gtsam::imuBias::ConstantBias prev_bias;
    gtsam::FixedLagSmoother::KeyTimestampMap key_time_stamps;

    // noise models
    gtsam::noiseModel::Diagonal::shared_ptr pose_noise_model;
    gtsam::noiseModel::Isotropic::shared_ptr velocity_noise_model;
    gtsam::noiseModel::Isotropic::shared_ptr bias_noise_model;

    // optimizer & graph
    gtsam::FixedLagSmoother::KeyTimestampMap key_timestamps;
    std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smootherISAM2;
    std::unique_ptr<gtsam::NonlinearFactorGraph> graph;

    void graph_init(const Eigen::Quaterniond &rotation, const Eigen::Vector3d &bias_acc,
                    const Eigen::Vector3d &bias_gyro, const double &stamp);
    void optimize(const sensor_msgs::PointCloud2::ConstPtr radar_msg);
    void update();

public:
    DopplerIOex(ros::NodeHandle *nh);
    ~DopplerIOex();

    rplrio::Params params;
    rplrio::RadarParams radarParam;

    uint step = 0;
    gtsam::NonlinearFactorGraph offline_graph;
    gtsam::Values cache_results;
    std::vector<gtsam::Vector3> radar_velocity_cache;
    std::vector<gtsam::Matrix3> radar_cov_cache;
    std::map<uint, double> key_frame_stamp;

    std::vector<rplrio::Odom> odom_vio;
    std::vector<std::string> bags;
    std::vector<std::string> topics;

    ros::Publisher pub_path, pub_gt_path;

    void feed_radar(const sensor_msgs::PointCloud2::ConstPtr pc_msg);
    void feed_imu(const sensor_msgs::Imu::ConstPtr imu);
    void run_bags();
    bool trigger_bags(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &rep);
    void cb_config(radar_odom::extrinsicConfig &config, uint32_t level);
    bool optimize(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &rep);
};

DopplerIOex::DopplerIOex(ros::NodeHandle *nh)
{
    params = rplrio::get_params(*nh);
    radarParam = params.radars[params.radar_list[0]];

    // imu init
    initializer = std::make_unique<StaticInitializer>(params.init_window, params.init_thres);

    // velocity optimizer init
    velocity_estimator = std::make_unique<RadarVelocityEstimatorCeres>();
    // velocity_estimator = std::make_unique<RadarVelocityEstimatorCeresHCS>(params);
    velocity_estimator->set_inlier_thresh(params.doppler_inlier_thresh);

    // ros init
    pub_v = nh->advertise<visualization_msgs::Marker>("velocity", 1);
    pub_odom = nh->advertise<nav_msgs::Odometry>("dopplerIO", 1);
    pub_radar = nh->advertise<sensor_msgs::PointCloud2>("radar_inlier", 1);
    pub_path = nh->advertise<nav_msgs::Path>("optimized_odom", 1);
    pub_gt_path = nh->advertise<nav_msgs::Path>("gt_odom", 1);
}

void DopplerIOex::graph_init(const Eigen::Quaterniond &rotation, const Eigen::Vector3d &bias_acc,
                             const Eigen::Vector3d &bias_gyro, const double &stamp)
{
    step = 0;

    // priors
    gtsam::Pose3 prior_pose(gtsam::Rot3::Quaternion(rotation.w(), rotation.x(), rotation.y(), rotation.z()), gtsam::Point3(0, 0, 0));
    gtsam::Vector3 prior_velocity(0, 0, 0);
    gtsam::imuBias::ConstantBias prior_imu_bias(bias_acc, bias_gyro);
    preintegrated = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(rplrio::imuParams(params), prior_imu_bias);
    assert(preintegrated);

    // pose init
    initial_values.clear();
    initial_values.insert(X(0), prior_pose);
    initial_values.insert(V(0), prior_velocity);
    initial_values.insert(B(0), prior_imu_bias);
    key_timestamps.clear();
    key_timestamps[X(0)] = stamp;
    key_timestamps[V(0)] = stamp;
    key_timestamps[B(0)] = stamp;

    prev_state = gtsam::NavState(prior_pose, prior_velocity);
    prop_state = prev_state;
    prev_bias = prior_imu_bias;

    // prior noise model
    pose_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
    velocity_noise_model = gtsam::noiseModel::Isotropic::Sigma(3, 1e-2);      // m/s
    bias_noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

    // Add all prior
    graph = std::make_unique<gtsam::NonlinearFactorGraph>();
    graph->addPrior(X(0), prior_pose, pose_noise_model);
    graph->addPrior(V(0), prior_velocity, velocity_noise_model);
    graph->addPrior(B(0), prior_imu_bias, bias_noise_model);

    cache_results.clear();
    cache_results.insert(X(0), prior_pose);
    cache_results.insert(V(0), prior_velocity);
    cache_results.insert(B(0), prior_imu_bias);

    offline_graph = gtsam::NonlinearFactorGraph();
    offline_graph.addPrior(X(0), prior_pose, pose_noise_model);
    offline_graph.addPrior(V(0), prior_velocity, velocity_noise_model);
    offline_graph.addPrior(B(0), prior_imu_bias, bias_noise_model);

    // fixed lag smoother init
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.0;
    parameters.relinearizeSkip = 1;
    smootherISAM2 = std::make_unique<gtsam::IncrementalFixedLagSmoother>(params.lag, parameters);
}

void DopplerIOex::update()
{
    // remove out of date radar measurements
    while (!radar_que.empty() && radar_que.front()->header.stamp.toSec() < imu_que.front()->stamp)
    {
        radar_que.pop_front();
    }
    if (radar_que.empty())
    {
        return;
    }

    auto radar_msg = radar_que.front();
    double radar_stamp = radar_msg->header.stamp.toSec();
    radar_que.pop_front();

    // integrate imu measurements
    while (!imu_que.empty() && radar_stamp > imu_que.front()->stamp)
    {
        auto imu_data = imu_que.front();
        double dt = imu_data->stamp - pre_imu_t;
        preintegrated->integrateMeasurement(imu_data->acc, imu_data->gyro, dt);
        pre_imu_t = imu_data->stamp;

        imu_que.pop_front();
    }

    step++;
    optimize(radar_msg);

    // recurisvely update until latest radar data is processed
    update();
}

void DopplerIOex::optimize(const sensor_msgs::PointCloud2::ConstPtr radar_msg)
{
    pcl::PointCloud<pcl::PointXYZIS>::Ptr radar_pc;
    radar_pc.reset(new pcl::PointCloud<pcl::PointXYZIS>);

    pcl::fromROSMsg(*radar_msg, *radar_pc);
    double current_t = radar_msg->header.stamp.toSec();

    // combined IMU factor ---------------------------------------------------------------------------
    auto preint_imu_combined = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements &>(*preintegrated);
    gtsam::CombinedImuFactor imu_factor(
        X(step - 1), V(step - 1), X(step),
        V(step), B(step - 1), B(step), preint_imu_combined);
    graph->add(imu_factor);
    offline_graph.add(imu_factor);

    prop_state = preintegrated->predict(prev_state, prev_bias);
    initial_values.insert(X(step), prop_state.pose());
    initial_values.insert(V(step), prop_state.v());
    initial_values.insert(B(step), prev_bias);

    key_timestamps[X(step)] = current_t;
    key_timestamps[V(step)] = current_t;
    key_timestamps[B(step)] = current_t;

    key_frame_stamp[step] = radar_msg->header.stamp.toSec();

    // ------------------------------------------------------------------------------------

    velocity_estimator->set_initial_velocity(prop_state.bodyVelocity());
    gtsam::Vector3 measured_v = velocity_estimator->optimize(radar_pc);
    pcl::PointCloud<pcl::PointXYZIS>::Ptr radar_inlier =
        velocity_estimator->get_inlier(radar_pc, measured_v);

    // second inlier estimate
    // velocity_estimator->set_initial_velocity(measured_v);
    // measured_v = velocity_estimator->optimize(radar_inlier);

    gtsam::Matrix3 radar_velocity_cov = velocity_estimator->get_covariance(
        radar_pc, measured_v);

    radar_velocity_cache.emplace_back(measured_v);
    radar_cov_cache.emplace_back(radar_velocity_cov);

    // velocity pose measurement
    measured_v = radarParam.R_r_b * measured_v;
    radar_velocity_cov = radarParam.R_r_b * radar_velocity_cov * radarParam.R_r_b.transpose();

    // auto robust_rv_noise = noiseModel::Gaussian::Covariance(radar_velocity_cov);
    auto robust_rv_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345),
        gtsam::noiseModel::Gaussian::Covariance(radar_velocity_cov));

    graph->add(VelocityPoseMeasurementFactor(
        V(step), X(step), measured_v, robust_rv_noise));

    // optimize ----------------------------------------------------------------------------
    smootherISAM2->update(*graph, initial_values, key_timestamps);
    gtsam::Values result = smootherISAM2->calculateEstimate();
    gtsam::Matrix x_cov = smootherISAM2->marginalCovariance(X(step));

    graph->resize(0);
    initial_values.clear();
    key_timestamps.clear();

    //-----------------------------------------------------------------------------
    // Overwrite the beginning of the preintegration for the next step.
    prev_state = gtsam::NavState(result.at<gtsam::Pose3>(X(step)),
                                 result.at<gtsam::Vector3>(V(step)));
    prev_bias = result.at<gtsam::imuBias::ConstantBias>(B(step));
    preintegrated->resetIntegrationAndSetBias(prev_bias);

    // gtsam::Pose3 xg(prev_state.pose().rotation(),
    //                 gtsam::Point3(prev_state.pose().x(), prev_state.pose().y(), 0));
    cache_results.insert(X(step), prev_state.pose());
    cache_results.insert(V(step), prev_state.velocity());
    cache_results.insert(B(step), prev_bias);

    // reset & publish -----------------------------------------------------------------------------
    radar_pc->clear();

    nav_msgs::Odometry odom = toOdometry(prev_state.pose().matrix(), x_cov);
    odom.header.stamp = radar_msg->header.stamp;
    odom.header.seq = step;
    odom.header.frame_id = "map";
    pub_odom.publish(odom);

    sensor_msgs::PointCloud2 inlier_msg;
    pcl::toROSMsg(*radar_inlier, inlier_msg);
    inlier_msg.header.stamp = radar_msg->header.stamp;
    inlier_msg.header.frame_id = "base";
    inlier_msg.header.seq = step;
    geometry_msgs::TwistWithCovarianceStamped radar_v =
        toROSTwist(measured_v, radar_velocity_cov, radar_msg->header);
    radar_v.header.seq = step;
    radar_v.header.frame_id = "base";

    pub_radar.publish(inlier_msg);
    pub_v.publish(velocityMarker(prev_state.velocity(),
                                 prev_state.pose().translation(), "map", "velocity"));

    // broadcast odometry transformation
    tf::Transform transform;
    auto trans = prev_state.pose().translation();
    auto orien = prev_state.pose().rotation().toQuaternion();
    transform.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()));
    transform.setRotation(tf::Quaternion(orien.x(), orien.y(), orien.z(), orien.w()));
    br.sendTransform(tf::StampedTransform(transform, radar_msg->header.stamp, "map", "base"));
}

void DopplerIOex::feed_radar(const sensor_msgs::PointCloud2::ConstPtr pc_msg)
{
    radar_que.emplace_back(pc_msg);

    if (!initialized)
    {
        return;
    }

    update();

    return;
}

void DopplerIOex::feed_imu(const sensor_msgs::Imu::ConstPtr imu_msg)
{
    double imu_t = imu_msg->header.stamp.toSec();

    gtsam::Vector3 acc = params.R_i_b * (gtsam::Vector3() << imu_msg->linear_acceleration.x,
                                         imu_msg->linear_acceleration.y,
                                         imu_msg->linear_acceleration.z)
                                            .finished();
    gtsam::Vector3 omega = params.R_i_b * (gtsam::Vector3() << imu_msg->angular_velocity.x,
                                           imu_msg->angular_velocity.y,
                                           imu_msg->angular_velocity.z)
                                              .finished();
    if (!initialized)
    {
        bool init_success = initializer->init_update(
            std::make_shared<rplrio::IMU>(acc, omega, imu_t));
        if (init_success)
        {
            initialized = true;
            Eigen::Vector3d bias_a = initializer->get_bias_acc();
            Eigen::Vector3d bias_g = initializer->get_bias_gyro();
            Eigen::Quaterniond init_r = initializer->get_rotation();

            pre_imu_t = initializer->get_first_stamp();
            graph_init(init_r, bias_a, bias_g, pre_imu_t);
            imu_que = initializer->get_imu_queue();
            ROS_INFO("static init success");
            ROS_INFO("bias acc: %.4f, %.4f, %.4f", bias_a[0], bias_a[1], bias_a[2]);
            ROS_INFO("bias gyro: %.4f, %.4f, %.4f", bias_g[0], bias_g[1], bias_g[2]);

            update();
        }

        return;
    }

    imu_que.emplace_back(std::make_shared<rplrio::IMU>(acc, omega, imu_t));
}

DopplerIOex::~DopplerIOex()
{
}

void DopplerIOex::run_bags()
{
    initialized = false;
    radar_que.clear();
    imu_que.clear();

    initializer = std::make_unique<StaticInitializer>(params.init_window, params.init_thres);

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
                feed_radar(radar_msg);
            }

            sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
            if (imu_msg != NULL)
            {
                feed_imu(imu_msg);
            }
        }

        bag.close();
    }
}

void DopplerIOex::cb_config(radar_odom::extrinsicConfig &config, uint32_t level)
{
    ROS_INFO("%f, %f, %f", config.Rx, config.Ry, config.Rz);

    gtsam::Rot3 R_r_i = gtsam::Rot3::Ypr(config.Rz, config.Ry, config.Rx);

    radarParam.R_r_b = params.R_i_b * R_r_i.matrix();
    std::cout << radarParam.R_r_b << std::endl;
}

bool DopplerIOex::trigger_bags(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &rep)
{
    run_bags();
    return true;
}

bool DopplerIOex::optimize(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &rep)
{
    run_bags();

    // extrinsic optimization
    auto vel_gt_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1e-6, 1e-6, 1e-6));
    auto ground_prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
    cache_results.insert(E(0), gtsam::Rot3(radarParam.R_r_b));

    gtsam::Values calibrate_result;
    nav_msgs::Path path;
    path.header.frame_id = "map";

    for (uint i = 1; i <= step; i++)
    {
        auto rv_noise = gtsam::noiseModel::Gaussian::Covariance(
            radarParam.R_r_b * radar_cov_cache[i] * radarParam.R_r_b.transpose());
        offline_graph.add(VelocityPoseExtrinsicFactor(
            V(i), X(i), E(0), radar_velocity_cache[i], rv_noise));

        double key_stamp = key_frame_stamp[i];
        auto it = std::lower_bound(odom_vio.begin(), odom_vio.end(), key_stamp);

        if (it == odom_vio.begin() || it == odom_vio.end())
            continue;

        gtsam::Pose3 T_t0 = (it - 1)->pose;
        gtsam::Pose3 T_t1 = it->pose;
        double t = (key_stamp - (it - 1)->stamp) / (it->stamp - (it - 1)->stamp);
        auto T_g = T_t0.interpolateRt(T_t1, t);
        path.poses.push_back(toROSPoseStamped(T_g, path.header));

        offline_graph.addPrior(
            X(i), T_g, ground_prior_noise);

        // gtsam::Vector3 gv = T_g.rotation().matrix() * it->v;
        // offline_graph.add(VelocityMeasurementFactor(V(i), gv, vel_gt_noise));
    }
    pub_gt_path.publish(path);

    calibrate_result = gtsam::LevenbergMarquardtOptimizer(offline_graph, cache_results).optimize();

    gtsam::Rot3 extrinsic = calibrate_result.at<gtsam::Rot3>(E(0));
    auto R_r_i = params.R_i_b.transpose() * extrinsic.matrix();
    std::cout << "R_r_i: " << std::endl
              << R_r_i << std::endl;

    path.poses.clear();
    for (uint i = 0; i < step; i++)
    {
        path.poses.push_back(toROSPoseStamped(calibrate_result.at<gtsam::Pose3>(X(i)), path.header));
    }
    pub_path.publish(path);

    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "radar_inertial_extrinsic");
    ros::NodeHandle nh;
    DopplerIOex dio(&nh);

    dynamic_reconfigure::Server<radar_odom::extrinsicConfig> server;
    dynamic_reconfigure::Server<radar_odom::extrinsicConfig>::CallbackType f;
    f = boost::bind(&DopplerIOex::cb_config, &dio, _1, _2);
    server.setCallback(f);

    ros::ServiceServer run_service = nh.advertiseService(
        "run_bag", &DopplerIOex::trigger_bags, &dio);

    ros::ServiceServer run_optimize = nh.advertiseService(
        "run_optimize", &DopplerIOex::optimize, &dio);

    std::string bag_folder;
    ros::param::get("~bag_folder", bag_folder);

    for (const auto &entry : std::filesystem::directory_iterator(bag_folder))
        dio.bags.emplace_back(entry.path().string());
    std::sort(dio.bags.begin(), dio.bags.end());
    dio.topics.push_back(dio.radarParam.topic);
    dio.topics.push_back(dio.params.imu_topic);

    std::string csv_path;
    ros::param::get("~gt_path", csv_path);
    dio.odom_vio = odom_csv(csv_path);
    std::cout << "vio: " << dio.odom_vio.size() << std::endl;

    ros::spin();

    return 0;
}