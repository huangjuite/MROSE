#pragma once

#include <memory>
#include <string>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rerun.hpp>

namespace rplrio {

class RerunLogger {
private:
  std::unique_ptr<rerun::RecordingStream> rec_;
  std::string app_id_;
  bool initialized_;

public:
  RerunLogger(const std::string &app_id = "rpl_rio",
              const std::string &spawn_method = "spawn");
  ~RerunLogger();

  void log_pose(const std::string &entity_path, const Eigen::Vector3d &position,
                const Eigen::Quaterniond &orientation, double timestamp = -1.0);

  template <typename PointType>
  void
  log_point_cloud(const std::string &entity_path,
                  const typename pcl::PointCloud<PointType>::ConstPtr &cloud,
                  double timestamp = -1.0);

  template <typename PointType>
  void log_point_cloud_with_colors(
      const std::string &entity_path,
      const typename pcl::PointCloud<PointType>::ConstPtr &cloud,
      const std::vector<uint32_t> &colors, double timestamp = -1.0);

  void log_transform3d(const std::string &entity_path,
                       const Eigen::Matrix4d &transform,
                       double timestamp = -1.0);

  void log_text(const std::string &entity_path, const std::string &text,
                double timestamp = -1.0);

  bool is_initialized() const { return initialized_; }

  rerun::RecordingStream &stream() { return *rec_; }
};

template <typename PointType>
void RerunLogger::log_point_cloud(
    const std::string &entity_path,
    const typename pcl::PointCloud<PointType>::ConstPtr &cloud,
    double timestamp) {
  if (!initialized_ || !cloud || cloud->empty()) {
    return;
  }

  std::vector<rerun::Position3D> positions;
  positions.reserve(cloud->size());

  for (const auto &point : cloud->points) {
    positions.push_back(rerun::Position3D(point.x, point.y, point.z));
  }

  if (timestamp > 0.0) {
    rec_->set_time_timestamp_secs_since_epoch("timestamp", timestamp);
  }

  rec_->log(entity_path, rerun::Points3D(positions));
}

template <typename PointType>
void RerunLogger::log_point_cloud_with_colors(
    const std::string &entity_path,
    const typename pcl::PointCloud<PointType>::ConstPtr &cloud,
    const std::vector<uint32_t> &colors, double timestamp) {
  if (!initialized_ || !cloud || cloud->empty()) {
    return;
  }

  std::vector<rerun::Position3D> positions;
  positions.reserve(cloud->size());

  for (const auto &point : cloud->points) {
    positions.push_back(rerun::Position3D(point.x, point.y, point.z));
  }

  if (timestamp > 0.0) {
    rec_->set_time_timestamp_secs_since_epoch("timestamp", timestamp);
  }

  if (!colors.empty() && colors.size() == cloud->size()) {
    rec_->log(entity_path, rerun::Points3D(positions).with_colors(colors));
  } else {
    rec_->log(entity_path, rerun::Points3D(positions));
  }
}

} // namespace rplrio
