#pragma once

#include <deque>
#include <fstream>
#include <memory>
#include <vector>

#include <spdlog/spdlog.h>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "optimization/ISAMOptimizer.hpp"
#include "radar/RadarVelocityEstimatorBase.hpp"
#include "core/StaticInitializer.hpp"
#include "core/params.hpp"
#include "core/rplrio_types.hpp"

namespace rplrio {
template <class PointType> class DIOManager {
private:
  typedef rplrio::Keyframe<PointType> DIOKeyframe;
  typedef rplrio::Radar<PointType> DIORadar;

  rplrio::Params params;
  std::shared_ptr<spdlog::logger> logger_;
  std::ofstream timing_csv_;

  double pre_imu_t = 0;
  bool initialized = false;

  // radar
  std::deque<std::shared_ptr<DIORadar>> radar_que;

  // imu
  std::deque<std::shared_ptr<rplrio::IMU>> imu_que;
  std::shared_ptr<rplrio::IMU> last_imu;
  std::shared_ptr<StaticInitializer> initializer;

  std::vector<std::shared_ptr<DIOKeyframe>> latest_states;

  std::unique_ptr<ISAMOptimizer<PointType>> optimizer;

  void update();

public:
  DIOManager(rplrio::Params params,
             std::shared_ptr<RadarVelocityEstimatorBase<PointType>>
                 velocity_estimator);
  ~DIOManager();

  void feed_radar(std::shared_ptr<DIORadar> radar);
  void feed_imu(std::shared_ptr<rplrio::IMU> imu_data);

  const std::vector<std::shared_ptr<rplrio::Keyframe<PointType>>> &
  get_latest_states() const;

  std::ofstream *get_timing_csv() { return timing_csv_.is_open() ? &timing_csv_ : nullptr; }

  void set_timing_csv_path(const std::string &path) {
    if (timing_csv_.is_open()) timing_csv_.close();
    timing_csv_.open(path, std::ios::out | std::ios::trunc);
    timing_csv_ << "wall_ms,label,duration_ms\n";
    velocity_estimator->set_timing_csv(&timing_csv_);
    optimizer->set_timing_csv(&timing_csv_);
  }

  std::shared_ptr<RadarVelocityEstimatorBase<PointType>> velocity_estimator;
};
} // namespace rplrio
