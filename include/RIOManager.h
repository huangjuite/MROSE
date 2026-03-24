#pragma once

// std includes
#include <deque>
#include <memory>
#include <vector>

// spdlog includes
#include <spdlog/spdlog.h>

// PCL includes
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// GTSAM includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "RadarVelocityEstimatorBase.h"
#include "StaticInitializer.h"
#include "VelocityPoseBiasFactor.h"
#include "form/RIOConstraintManager.h"
#include "form/formRadar.h"
#include "params.h"
#include "rplrio_types.h"

// typedef VelocityPoseMeasurementFactor RadarFactor;
typedef VelocityPoseBiasFactor RadarFactor;

namespace rplrio {
template <class PointType> class RIOManager {
private:
  typedef rplrio::Keyframe<PointType> RIOKeyframe;
  typedef rplrio::Radar<PointType> RIORadar;

  rplrio::Params params;
  std::shared_ptr<spdlog::logger> logger_;

  double pre_imu_t = 0;
  int imu_count = 0;
  bool correct_now = false;
  bool initialized = false;

  // radar
  std::deque<std::shared_ptr<RIORadar>> radar_que;

  // imu settings
  std::shared_ptr<gtsam::PreintegrationType> preintegrated;
  std::deque<std::shared_ptr<rplrio::IMU>> imu_que;
  std::shared_ptr<rplrio::IMU> last_imu;
  std::shared_ptr<StaticInitializer> initializer;

  // states
  gtsam::NavState prev_state;
  gtsam::NavState prop_state;
  gtsam::imuBias::ConstantBias prev_bias;

  std::vector<std::shared_ptr<RIOKeyframe>> latest_states;
  std::tuple<std::vector<form::PlanarFeat>, std::vector<form::PointFeat>> latest_keypoints;

  void graph_init(const Eigen::Quaterniond &rotation,
                  const Eigen::Vector3d &bias_acc,
                  const Eigen::Vector3d &bias_gyro, const double &stamp);
  void update();

  std::shared_ptr<RIOKeyframe> optimize(const RIORadar &radar,
                                        const rplrio::IMU &imu);

public:
  RIOManager(rplrio::Params params,
             std::shared_ptr<RadarVelocityEstimatorBase<PointType>>
                 velocity_estimator,
             RadarEstimator::Params form_params);
  ~RIOManager();

  void feed_radar(std::shared_ptr<RIORadar> radar);

  void feed_imu(std::shared_ptr<rplrio::IMU> imu_data);

  const std::vector<std::shared_ptr<rplrio::Keyframe<PointType>>> &
  get_latest_states() const;

  const std::tuple<std::vector<form::PlanarFeat>, std::vector<form::PointFeat>> &
  get_latest_keypoints() const { return latest_keypoints; }

std::shared_ptr<RadarVelocityEstimatorBase<PointType>> velocity_estimator;

  // form components
  RadarEstimator::Params form_params;
  RadarFeatureExtractor m_extractor;
  RIOConstraintManager m_constraints;
  std::tuple<form::Matcher<form::PlanarFeat>, form::Matcher<form::PointFeat>> m_matcher;
  form::KeyScanner m_keyscanner;
  std::tuple<form::KeypointMap<form::PlanarFeat>, form::KeypointMap<form::PointFeat>> m_keypoint_map;
};
} // namespace rplrio
