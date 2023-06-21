
// std includes
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <deque>
#include <memory>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

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
#include "AltitudeFactor.h"
#include "RotationFactor.h"
#include "RadarVelocityEstimator.h"
#include "RadarVelocityEstimatorGNC.h"
#include "RadarVelocityEstimatorHCS.h"
#include "RadarVelocityEstimatorCelestial.h"
#include "RadarVelocityEstimatorCeres.h"
#include "RadarVelocityEstimatorCeresHCS.h"
#include "StaticInitializer.hpp"

#include "types.h"
#include "utils.h"
#include "params.h"

using namespace ros;
using namespace std;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
class DopplerIO
{
private:
    rplrio::Params params;

    gtsam::Rot3 tf_base2map;
    double pre_imu_t = 0;
    int step = 0;
    int imu_count = 0;
    bool correct_now = false;
    bool initialized = false;

    NodeHandle *nh;
    Subscriber sub_radar;
    Subscriber sub_imu;
    Publisher pub_odom;
    Publisher pub_v;
    Publisher pub_twist;
    Publisher pub_bias;
    Publisher pub_radar;
    Publisher pub_error;
    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    // radar
    // RadarVelocityEstimator *velocity_estimator;
    std::unique_ptr<RadarVelocityEstimatorCeres> velocity_estimator;
    // std::unique_ptr<RadarVelocityEstimatorCeresHCS> velocity_estimator;
    // RadarVelocityEstimatorGNC *velocity_estimator;
    // RadarVelocityEstimatorCelestial *velocity_estimator;
    // RadarVelocityEstimatorHCS *velocity_estimator;
    std::deque<sensor_msgs::PointCloud2::ConstPtr> radar_que;

    // imu settings
    std::shared_ptr<PreintegrationType> preintegrated;
    std::deque<std::shared_ptr<rio::IMU>> imu_que;
    std::shared_ptr<StaticInitializer> initializer;

    // states
    Values initial_values;
    NavState prev_state;
    NavState prop_state;
    imuBias::ConstantBias prev_bias;

    // noise models
    noiseModel::Diagonal::shared_ptr pose_noise_model;
    noiseModel::Isotropic::shared_ptr velocity_noise_model;
    noiseModel::Isotropic::shared_ptr bias_noise_model;

    // optimizer & graph
    FixedLagSmoother::KeyTimestampMap key_timestamps;
    std::unique_ptr<IncrementalFixedLagSmoother> smootherISAM2;
    std::unique_ptr<NonlinearFactorGraph> graph;

    void cb_radar(const sensor_msgs::PointCloud2::ConstPtr &pc_msg);
    void cb_imu(const sensor_msgs::Imu::ConstPtr &imu);
    void graph_init(const Eigen::Quaterniond &rotation, const Eigen::Vector3d &bias_acc,
                           const Eigen::Vector3d &bias_gyro, const double &stamp);
    void update();
    void optimize(const sensor_msgs::PointCloud2::ConstPtr radar_msg);

public:
    DopplerIO(NodeHandle *nh);
    ~DopplerIO();

    // tmp
    ofstream err_file;
};
