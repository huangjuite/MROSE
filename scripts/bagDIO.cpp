
#include <atomic>
#include <fstream>
#include <gtsam/base/Matrix.h>
#include <iomanip>
#include <memory>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <yaml-cpp/yaml.h>

#include "core/RadarPointType.hpp"
#include "DIORosNode.hpp"
#include "log/RerunLogger.hpp"
#include "core/params.hpp"
#include "core/rplrio_types.hpp"

std::atomic<bool> g_shutdown_requested(false);

void signal_handler(int signum) {
  (void)signum;
  g_shutdown_requested = true;
  std::cout << "\nShutdown requested, stopping..." << std::endl;
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <bag_file_path> [logging_method] [save_path] [velocity_save_path]" << std::endl;
    std::cerr << "  logging_method: spawn (default), serve, save, connect, or none" << std::endl;
    std::cerr << "  save_path: Path to save trajectory file (default: /tmp/rio_trajectory.txt)" << std::endl;
    std::cerr << "  velocity_save_path: Path to save velocity file (default: /tmp/rio_velocity.txt)" << std::endl;
    return 1;
  }

  // Setup signal handlers
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  std::string bag_path = argv[1];
  std::string logging_method = (argc >= 3) ? argv[2] : "none";
  std::string save_path = (argc >= 4) ? argv[3] : "/tmp/rio_trajectory.txt";
  std::string velocity_save_path = (argc >= 5) ? argv[4] : "/tmp/rio_velocity.txt";

  rclcpp::init(argc, argv);

  auto node = std::make_shared<DIORosNode>();
  auto T_i_b = gtsam::Pose3(
      gtsam::Rot3(node->params.R_i_b),
      gtsam::Point3(node->params.T_i_b.block<3, 1>(0, 3)));

  std::unique_ptr<rplrio::RerunLogger> rerun_logger = nullptr;
  if (logging_method != "none") {
    rerun_logger =
        std::make_unique<rplrio::RerunLogger>("bagDIO", logging_method);
  }

  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path;
  storage_options.storage_id = "mcap";

  rosbag2_cpp::ConverterOptions converter_options{};
  converter_options.input_serialization_format = "cdr";

  try {
    reader.open(storage_options, converter_options);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to open bag file: %s", e.what());
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Reading from bag: %s", bag_path.c_str());

  rosbag2_storage::StorageFilter storage_filter;
  storage_filter.topics.push_back(node->params.imu_topic);
  for (auto r : node->params.radar_list) {
    storage_filter.topics.push_back(node->params.radars[r].topic);
  }
  reader.set_filter(storage_filter);

  RCLCPP_INFO(node->get_logger(), "Filtering topics:");
  for (const auto &topic : storage_filter.topics) {
    RCLCPP_INFO(node->get_logger(), "  - %s", topic.c_str());
  }

  auto topics = reader.get_all_topics_and_types();
  RCLCPP_INFO(node->get_logger(), "Bag contains %zu topics:", topics.size());
  for (const auto &topic : topics) {
    RCLCPP_INFO(node->get_logger(), "  - %s [%s]", topic.name.c_str(),
                topic.type.c_str());
  }

  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_serialization;

  std::vector<std::string> radar_topics;
  for (auto r : node->params.radar_list) {
    radar_topics.push_back(node->params.radars[r].topic);
  }

  std::vector<Eigen::Vector3d> path_positions;
  std::vector<Eigen::Quaterniond> path_orientations;
  std::vector<double> path_timestamps;
  std::vector<Eigen::Vector3d> path_velocities;

  std::vector<double> velocity_timestamps;
  std::vector<Eigen::Vector3d> state_velocities;
  std::vector<Eigen::Vector3d> radar_velocities;

  // Accumulate path as PointCloud for rerun logging
  auto path_cloud = std::make_shared<rplrio::PointCloud>();

  while (reader.has_next() && !g_shutdown_requested) {
    auto bag_message = reader.read_next();

    try {
      if (bag_message->topic_name == node->params.imu_topic) {
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        imu_serialization.deserialize_message(&serialized_msg, imu_msg.get());
        node->cb_imu(imu_msg);
      } else if (std::find(radar_topics.begin(), radar_topics.end(),
                           bag_message->topic_name) != radar_topics.end()) {
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pc_serialization.deserialize_message(&serialized_msg, pc_msg.get());

        node->cb_radar(pc_msg);

        auto &states = node->dio_manager->get_latest_states();
        for (const std::shared_ptr<rplrio::Keyframe> &kf : states) {
          double timestamp = kf->stamp;
          auto current_pose = kf->state.pose().compose(T_i_b);

          Eigen::Vector3d position = current_pose.translation();
          Eigen::Quaterniond orientation(current_pose.rotation().toQuaternion());
          Eigen::Vector3d velocity = kf->state.velocity();

          path_positions.push_back(position);
          path_orientations.push_back(orientation);
          path_timestamps.push_back(timestamp);
          path_velocities.push_back(velocity);

          velocity_timestamps.push_back(timestamp);
          Eigen::Vector3d state_vel_body =
              current_pose.rotation().matrix().transpose() *
              Eigen::Vector3d(kf->state.velocity());
          state_velocities.push_back(state_vel_body);
          radar_velocities.push_back(Eigen::Vector3d(kf->radar_velocity));

          if (rerun_logger) {
            rerun_logger->log_pose("world/base_link", position, orientation,
                                   timestamp);
            rerun_logger->log_pose("world/base_link/imu", T_i_b.translation(),
                                   T_i_b.rotation().toQuaternion(), timestamp);

            path_cloud->push_back(
                {static_cast<float>(position.x()),
                 static_cast<float>(position.y()),
                 static_cast<float>(position.z()), 0.0f});
            rerun_logger->log_point_cloud("world/path", path_cloud, timestamp);

            if (kf->radar_points && !kf->radar_points->empty()) {
              rerun_logger->log_point_cloud(
                  "world/base_link/imu/" + kf->name, kf->radar_points,
                  timestamp);
            }

            rerun_logger->log_vector3("plots/radar_velocity",
                                      Eigen::Vector3d(kf->radar_velocity),
                                      timestamp);

            gtsam::Vector3 v_rot =
                gtsam::skewSymmetric(
                    kf->imu_bias.correctGyroscope(kf->imu.gyro)) *
                node->params.radars[kf->name].T_r_i.block(0, 3, 3, 1);

            rerun_logger->log_vector3("plots/v_rot", v_rot, timestamp);
            rerun_logger->log_vector3("plots/v_err", kf->v_err, timestamp);

            rerun_logger->log_vector3("plots/imu/gyro",
                                      Eigen::Vector3d(kf->imu.gyro), timestamp);

            rerun_logger->log_vector3(
                "plots/imu_bias/accel",
                Eigen::Vector3d(kf->imu_bias.accelerometer()), timestamp);
            rerun_logger->log_vector3("plots/imu_bias/gyro",
                                      kf->imu_bias.gyroscope(), timestamp);

            Eigen::Vector3d velocity_cov = kf->velocity_cov.diagonal().array();
            rerun_logger->log_vector3("plots/velocity_cov", velocity_cov,
                                      timestamp);

            Eigen::Vector3d radar_velocity_cov =
                kf->radar_velocity_cov.diagonal().array();
            rerun_logger->log_vector3("plots/radar_velocity_cov",
                                      radar_velocity_cov, timestamp);

            Eigen::Vector3d position_cov =
                kf->pose_cov.block<3, 3>(3, 3).diagonal().array();
            rerun_logger->log_vector3("plots/position_cov", position_cov,
                                      timestamp);

            Eigen::Vector3d orientation_cov =
                kf->pose_cov.block<3, 3>(0, 0).diagonal().array();
            rerun_logger->log_vector3("plots/orientation_cov", orientation_cov,
                                      timestamp);
          }
        }
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node->get_logger(), "Failed to process topic '%s': %s",
                   bag_message->topic_name.c_str(), e.what());
      continue;
    }
  }

  if (g_shutdown_requested) {
    RCLCPP_INFO(node->get_logger(), "Processing interrupted by user");
  } else {
    RCLCPP_INFO(node->get_logger(), "Finished processing bag file");
  }

  if (!path_positions.empty()) {
    std::ofstream traj_file(save_path);
    if (traj_file.is_open()) {
      traj_file << std::fixed << std::setprecision(9);
      for (size_t i = 0; i < path_positions.size(); ++i) {
        const auto &pos = path_positions[i];
        const auto &quat = path_orientations[i];
        const auto &vel = path_velocities[i];
        traj_file << path_timestamps[i] << " " << pos.x() << " " << pos.y()
                  << " " << pos.z() << " " << quat.x() << " " << quat.y() << " "
                  << quat.z() << " " << quat.w() << " " << vel.x() << " "
                  << vel.y() << " " << vel.z() << "\n";
      }
      traj_file.close();
      RCLCPP_INFO(node->get_logger(), "Trajectory saved to: %s (%zu poses)",
                  save_path.c_str(), path_positions.size());
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to open trajectory file: %s",
                   save_path.c_str());
    }
  }

  if (!velocity_timestamps.empty()) {
    std::ofstream vel_file(velocity_save_path);
    if (vel_file.is_open()) {
      vel_file << std::fixed << std::setprecision(9);
      vel_file << "# timestamp state_vx state_vy state_vz radar_vx radar_vy radar_vz\n";
      for (size_t i = 0; i < velocity_timestamps.size(); ++i) {
        const auto &state_vel = state_velocities[i];
        const auto &radar_vel = radar_velocities[i];
        vel_file << velocity_timestamps[i] << " "
                 << state_vel.x() << " " << state_vel.y() << " " << state_vel.z() << " "
                 << radar_vel.x() << " " << radar_vel.y() << " " << radar_vel.z() << "\n";
      }
      vel_file.close();
      RCLCPP_INFO(node->get_logger(), "Velocity data saved to: %s (%zu frames)",
                  velocity_save_path.c_str(), velocity_timestamps.size());
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to open velocity file: %s",
                   velocity_save_path.c_str());
    }
  }

  rclcpp::shutdown();
  return 0;
}
