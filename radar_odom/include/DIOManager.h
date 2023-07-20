
// std includes
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <deque>
#include <memory>

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

#include "VelocityPoseMeasurementFactor.h"
#include "RadarVelocityEstimatorCeres.h"
#include "RadarVelocityEstimatorCeresHCS.h"
#include "StaticInitializer.hpp"

#include "types.h"
#include "params.h"

using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

namespace rplrio
{
    class DIOManager
    {
    private:
        rplrio::Params params;

        double pre_imu_t = 0;
        int step = 0;
        int imu_count = 0;
        bool correct_now = false;
        bool initialized = false;

        // radar
        std::unique_ptr<RadarVelocityEstimatorCeres> velocity_estimator;
        // std::unique_ptr<RadarVelocityEstimatorCeresHCS> velocity_estimator;
        std::deque<std::shared_ptr<rplrio::Radar>> radar_que;

        // imu settings
        std::shared_ptr<gtsam::PreintegrationType> preintegrated;
        std::deque<std::shared_ptr<rplrio::IMU>> imu_que;
        std::shared_ptr<StaticInitializer> initializer;

        // states
        gtsam::Values initial_values;
        gtsam::NavState prev_state;
        gtsam::NavState prop_state;
        gtsam::imuBias::ConstantBias prev_bias;
        std::vector<std::shared_ptr<rplrio::Keyframe>> latest_states;

        // optimizer & graph
        gtsam::FixedLagSmoother::KeyTimestampMap key_timestamps;
        std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smootherISAM2;
        std::unique_ptr<gtsam::NonlinearFactorGraph> graph;

                void graph_init(const Eigen::Quaterniond &rotation, const Eigen::Vector3d &bias_acc,
                        const Eigen::Vector3d &bias_gyro, const double &stamp);
        void update();
        std::shared_ptr<rplrio::Keyframe> optimize(const rplrio::Radar &radar);

    public:
        DIOManager(rplrio::Params params);
        ~DIOManager();

        void feed_radar(std::shared_ptr<rplrio::Radar> radar);
        void feed_imu(std::shared_ptr<rplrio::IMU> imu_data);
        std::vector<std::shared_ptr<rplrio::Keyframe>> get_latest_states();
    };
}