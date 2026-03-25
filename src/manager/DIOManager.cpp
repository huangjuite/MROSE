#include "manager/DIOManager.hpp"
#include "core/RadarPointType.hpp"

#include <spdlog/sinks/stdout_color_sinks.h>

namespace rplrio {

template <class PointType>
DIOManager<PointType>::DIOManager(
    rplrio::Params params,
    std::shared_ptr<RadarVelocityEstimatorBase<PointType>> velocity_estimator)
    : params(params), velocity_estimator(velocity_estimator) {

  logger_ = spdlog::stdout_color_mt("rplrio");
  logger_->set_pattern("[%^%l%$] %v");
  logger_->set_level(static_cast<spdlog::level::level_enum>(params.log_level));

  optimizer = std::make_unique<ISAMOptimizer<PointType>>(params, velocity_estimator);

  if (params.use_static_init) {
    initialized = false;
    initializer = std::make_unique<StaticInitializer>(
        params.init_window, params.init_thres, params.gravity);
  }
}

template <class PointType>
void DIOManager<PointType>::update() {
  // remove out of date radar measurements
  while (!radar_que.empty() && radar_que.front()->stamp < pre_imu_t - 1e-6) {
    logger_->debug("out of date: {}, {:.6f}", radar_que.front()->frame_id,
                   radar_que.front()->stamp);
    logger_->debug("imu front {:.6f}, pre: {:.6f}", imu_que.front()->stamp,
                   pre_imu_t);
    radar_que.pop_front();
  }

  if (radar_que.empty() || radar_que.front()->stamp > imu_que.back()->stamp) {
    return;
  }

  auto radar_msg = radar_que.front();
  double radar_stamp = radar_msg->stamp;
  radar_que.pop_front();

  // integrate imu measurements
  size_t imu_size = 0;
  while (!imu_que.empty() && (imu_que.front()->stamp - radar_stamp) < 1e-6) {
    auto imu_data = imu_que.front();
    double dt = imu_data->stamp - pre_imu_t;
    optimizer->preintegrated->integrateMeasurement(imu_data->acc, imu_data->gyro, dt);
    pre_imu_t = imu_data->stamp;
    last_imu = imu_data;
    imu_que.pop_front();
    imu_size++;
  }

  logger_->info("------------------------------");
  logger_->info("key frame {}", radar_msg->frame_id);
  logger_->info("imu queue size: {}", imu_que.size());
  logger_->info("radar queue size: {}", radar_que.size());
  logger_->info("imu integrated: {}", imu_size);

  if (imu_size == 0) {
    logger_->debug("only add radar.");
    latest_states.emplace_back(optimizer->optimize(*radar_msg, *last_imu));
  } else {
    auto next_imu = imu_que.front();
    rplrio::IMU interp;

    bool interp_success =
        rplrio::IMU::interpolate(interp, radar_stamp, *last_imu, *next_imu);
    if (interp_success) {
      optimizer->preintegrated->integrateMeasurement(interp.acc, interp.gyro,
                                                     interp.stamp - pre_imu_t);
      pre_imu_t = interp.stamp;
      *last_imu = interp;
    }

    optimizer->step++;
    latest_states.emplace_back(optimizer->optimize(*radar_msg, *last_imu));
  }

  logger_->info("step: {}, time: {:.6f}", optimizer->step + 1, radar_stamp);

  update();
}

template <class PointType>
void DIOManager<PointType>::feed_imu(std::shared_ptr<rplrio::IMU> imu_data) {

  if (params.use_static_init && !initialized) {
    bool init_success = initializer->init_update(imu_data);
    if (init_success) {
      Eigen::Vector3d bias_a = initializer->get_bias_acc();
      Eigen::Vector3d bias_g = initializer->get_bias_gyro();
      Eigen::Quaterniond init_r = initializer->get_rotation();

      pre_imu_t = initializer->get_first_stamp();
      optimizer->graph_init(init_r, bias_a, bias_g, pre_imu_t);
      initialized = true;
      imu_que = initializer->get_imu_queue();
      last_imu = imu_que.front();
      auto rot_mat = init_r.toRotationMatrix();

      logger_->info("static init");
      logger_->info("bias acc: {}, {}, {}", bias_a[0], bias_a[1], bias_a[2]);
      logger_->info("bias gyro: {}, {}, {}", bias_g[0], bias_g[1], bias_g[2]);
      logger_->info("rotation:");
      logger_->info("  [{:9.6f}, {:9.6f}, {:9.6f}]", rot_mat(0, 0),
                    rot_mat(0, 1), rot_mat(0, 2));
      logger_->info("  [{:9.6f}, {:9.6f}, {:9.6f}]", rot_mat(1, 0),
                    rot_mat(1, 1), rot_mat(1, 2));
      logger_->info("  [{:9.6f}, {:9.6f}, {:9.6f}]", rot_mat(2, 0),
                    rot_mat(2, 1), rot_mat(2, 2));

      update();
    }
  } else if (!params.use_static_init && !initialized) {
    Eigen::Vector3d bias_a = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias_g = Eigen::Vector3d::Zero();
    Eigen::Quaterniond init_r = Eigen::Quaterniond::Identity();

    pre_imu_t = imu_data->stamp;
    optimizer->graph_init(init_r, bias_a, bias_g, pre_imu_t);
    initialized = true;
  } else {
    imu_que.emplace_back(imu_data);
  }
}

template <class PointType>
void DIOManager<PointType>::feed_radar(std::shared_ptr<DIORadar> radar) {
  radar_que.emplace_back(radar);

  if (!initialized) {
    return;
  }

  latest_states.clear();
  update();
}

template <class PointType>
const std::vector<std::shared_ptr<rplrio::Keyframe<PointType>>> &
DIOManager<PointType>::get_latest_states() const {
  return latest_states;
}

template <class PointType> DIOManager<PointType>::~DIOManager() {}

template class DIOManager<pcl::PointXYZD>;
template class DIOManager<pcl::PointXYZSD>;
template class DIOManager<pcl::PointXYZSDW>;
template class DIOManager<pcl::PointHugin>;

} // namespace rplrio
