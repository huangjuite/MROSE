#include "log/RerunLogger.hpp"
#include <spdlog/spdlog.h>

namespace rplrio {

RerunLogger::RerunLogger(const std::string &app_id,
                         const std::string &spawn_method)
    : app_id_(app_id), initialized_(false) {
  try {
    rec_ = std::make_unique<rerun::RecordingStream>(app_id_);

    if (spawn_method == "spawn") {
      rec_->spawn().exit_on_failure();
    } else if (spawn_method == "connect") {
      rec_->connect_grpc().exit_on_failure();
    } else if (spawn_method == "save") {
      rec_->save(app_id_ + ".rrd").exit_on_failure();
    } else if (spawn_method == "server") {
      auto result = rec_->serve_grpc();
      if (result.is_ok()) {
        spdlog::info("Rerun gRPC server started at: {}", result.value);
      } else {
        result.error.exit_on_failure();
      }
    } else {
      spdlog::error("Unknown spawn method: {}", spawn_method);
      throw std::invalid_argument("Unknown spawn method: " + spawn_method);
    }

    initialized_ = true;
  } catch (const std::exception &e) {
    spdlog::error("Failed to initialize RerunLogger: {}", e.what());
    throw;
  }
}

RerunLogger::~RerunLogger() {
  if (initialized_ && rec_) {
    rec_.reset();
  }
}

void RerunLogger::log_pose(const std::string &entity_path,
                           const Eigen::Vector3d &position,
                           const Eigen::Quaterniond &orientation,
                           double timestamp) {
  if (!initialized_) {
    return;
  }

  if (timestamp > 0.0) {
    rec_->set_time_timestamp_secs_since_epoch("timestamp", timestamp);
  }

  rerun::Transform3D rerun_transform =
      rerun::Transform3D::from_translation_rotation(
          rerun::Vec3D(static_cast<float>(position.x()),
                       static_cast<float>(position.y()),
                       static_cast<float>(position.z())),
          rerun::datatypes::Quaternion::from_xyzw(
              static_cast<float>(orientation.x()),
              static_cast<float>(orientation.y()),
              static_cast<float>(orientation.z()),
              static_cast<float>(orientation.w())));

  rec_->log(entity_path, rerun_transform);
}

void RerunLogger::log_point_cloud(const std::string &entity_path,
                                  const PointCloudPtr &cloud,
                                  double timestamp) {
  if (!initialized_ || !cloud || cloud->empty()) {
    return;
  }

  std::vector<rerun::Position3D> positions;
  positions.reserve(cloud->size());

  for (const auto &point : *cloud) {
    positions.push_back(rerun::Position3D(point.x, point.y, point.z));
  }

  if (timestamp > 0.0) {
    rec_->set_time_timestamp_secs_since_epoch("timestamp", timestamp);
  }

  rec_->log(entity_path, rerun::Points3D(positions));
}

void RerunLogger::log_point_cloud_with_colors(const std::string &entity_path,
                                              const PointCloudPtr &cloud,
                                              const std::vector<uint32_t> &colors,
                                              double timestamp) {
  if (!initialized_ || !cloud || cloud->empty()) {
    return;
  }

  std::vector<rerun::Position3D> positions;
  positions.reserve(cloud->size());

  for (const auto &point : *cloud) {
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

void RerunLogger::log_transform3d(const std::string &entity_path,
                                  const Eigen::Matrix4d &transform,
                                  double timestamp) {
  if (!initialized_) {
    return;
  }

  Eigen::Vector3d translation = transform.block<3, 1>(0, 3);
  Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
  Eigen::Quaterniond quat(rotation);

  if (timestamp > 0.0) {
    rec_->set_time_timestamp_secs_since_epoch("timestamp", timestamp);
  }

  rerun::Transform3D rerun_transform =
      rerun::Transform3D::from_translation_rotation(
          rerun::Vec3D(static_cast<float>(translation.x()),
                       static_cast<float>(translation.y()),
                       static_cast<float>(translation.z())),
          rerun::datatypes::Quaternion::from_xyzw(
              static_cast<float>(quat.x()), static_cast<float>(quat.y()),
              static_cast<float>(quat.z()), static_cast<float>(quat.w())));

  rec_->log(entity_path, rerun_transform);
}

void RerunLogger::log_text(const std::string &entity_path,
                           const std::string &text, double timestamp) {
  if (!initialized_) {
    return;
  }

  if (timestamp > 0.0) {
    rec_->set_time_timestamp_secs_since_epoch("timestamp", timestamp);
  }

  rec_->log(entity_path, rerun::TextLog(text));
}

void RerunLogger::log_scalar(const std::string &entity_path, double value,
                             double timestamp) {
  if (!initialized_) {
    return;
  }

  if (timestamp > 0.0) {
    rec_->set_time_timestamp_secs_since_epoch("timestamp", timestamp);
  }

  rec_->log(entity_path, rerun::Scalars(value));
}

void RerunLogger::log_vector3(const std::string &entity_path,
                              const Eigen::Vector3d &vec, double timestamp) {
  if (!initialized_) {
    return;
  }

  if (timestamp > 0.0) {
    rec_->set_time_timestamp_secs_since_epoch("timestamp", timestamp);
  }

  // Log each component as a separate scalar
  rec_->log(entity_path + "/x", rerun::Scalars(vec.x()));
  rec_->log(entity_path + "/y", rerun::Scalars(vec.y()));
  rec_->log(entity_path + "/z", rerun::Scalars(vec.z()));
}

} // namespace rplrio
