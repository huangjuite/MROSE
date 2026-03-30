#pragma once

#include <memory>
#include <string>

#include <Eigen/Dense>
#include <rerun.hpp>

#include "core/RadarPointType.hpp"

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

  void log_point_cloud(const std::string &entity_path,
                       const PointCloudPtr &cloud, double timestamp = -1.0);

  void log_point_cloud_with_colors(const std::string &entity_path,
                                   const PointCloudPtr &cloud,
                                   const std::vector<uint32_t> &colors,
                                   double timestamp = -1.0);

  void log_transform3d(const std::string &entity_path,
                       const Eigen::Matrix4d &transform,
                       double timestamp = -1.0);

  void log_text(const std::string &entity_path, const std::string &text,
                double timestamp = -1.0);

  void log_scalar(const std::string &entity_path, double value,
                  double timestamp = -1.0);

  void log_vector3(const std::string &entity_path, const Eigen::Vector3d &vec,
                   double timestamp = -1.0);

  bool is_initialized() const { return initialized_; }

  rerun::RecordingStream &stream() { return *rec_; }
};

} // namespace rplrio
