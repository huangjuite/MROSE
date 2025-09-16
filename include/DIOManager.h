
// std includes
#include <deque>
#include <memory>
#include <vector>

// PCL includes
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// GTSAM includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "RadarVelocityEstimatorBase.h"
#include "StaticInitializer.h"
#include "VelocityPoseMeasurementFactor.h"
#include "params.h"
#include "rplrio_types.h"

using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

typedef VelocityPoseMeasurementFactor RadarFactor;

namespace rplrio {
template <class PointType> class DIOManager {
private:
  typedef rplrio::Keyframe<PointType> DIOKeyframe;
  typedef rplrio::Radar<PointType> DIORadar;

  rplrio::Params params;

  double pre_imu_t = 0;
  int step = 0;
  int imu_count = 0;
  bool correct_now = false;
  bool initialized = false;

  // radar
  std::deque<std::shared_ptr<DIORadar>> radar_que;

  // imu settings
  std::shared_ptr<gtsam::PreintegrationType> preintegrated;
  std::deque<std::shared_ptr<rplrio::IMU>> imu_que;
  std::shared_ptr<StaticInitializer> initializer;

  // states
  gtsam::Values initial_values;
  gtsam::NavState prev_state;
  gtsam::NavState prop_state;
  gtsam::imuBias::ConstantBias prev_bias;

  std::vector<std::shared_ptr<DIOKeyframe>> latest_states;

  // optimizer & graph
  gtsam::FixedLagSmoother::KeyTimestampMap key_timestamps;
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smootherISAM2;
  std::unique_ptr<gtsam::NonlinearFactorGraph> graph;

  void graph_init(const Eigen::Quaterniond &rotation,
                  const Eigen::Vector3d &bias_acc,
                  const Eigen::Vector3d &bias_gyro, const double &stamp);
  void update();

  std::shared_ptr<DIOKeyframe> optimize(const DIORadar &radar);

public:
  DIOManager(rplrio::Params params,
             std::shared_ptr<RadarVelocityEstimatorBase<PointType>>
                 velocity_estimator);
  ~DIOManager();

  void feed_radar(std::shared_ptr<DIORadar> radar);

  void feed_imu(std::shared_ptr<rplrio::IMU> imu_data);

  std::vector<std::shared_ptr<rplrio::Keyframe<PointType>>> get_latest_states();

  std::shared_ptr<RadarVelocityEstimatorBase<PointType>> velocity_estimator;
};
} // namespace rplrio