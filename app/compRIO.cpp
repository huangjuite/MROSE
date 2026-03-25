
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

#include <pcl/common/transforms.h>

#include "core/RadarPointType.hpp"
#include "node/RerunLogger.hpp"
#include "node/RIORosNode.hpp"
#include "core/rplrio_types.hpp"

// Signal handler for clean shutdown
std::atomic<bool> g_shutdown_requested(false);

void signal_handler(int signum) {
  (void)signum;
  g_shutdown_requested = true;
  std::cout << "\nShutdown requested, stopping..." << std::endl;
}

std::string getPointTypeFromConfig(const std::string &config_path) {
  try {
    YAML::Node config = YAML::LoadFile(config_path);
    if (config["point_type"]) {
      return config["point_type"].as<std::string>();
    }
  } catch (const YAML::Exception &e) {
    std::cerr << "Error reading config file: " << e.what() << std::endl;
  }
  return "";
}

template <typename PointType> int runBagDIO(int argc, char *argv[]) {
  // Setup signal handlers
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  std::string bag_path = argv[1];
  std::string logging_method = (argc >= 3) ? argv[2] : "none";
  std::string save_path = (argc >= 4) ? argv[3] : "/tmp/rio_trajectory.txt";
  std::string velocity_save_path =
      (argc >= 5) ? argv[4] : "/tmp/rio_velocity.txt";

  // Create the node
  auto node = std::make_shared<RIORosNode<PointType>>();
  auto T_i_b = gtsam::Pose3(
      gtsam::Rot3(node->params.R_i_b),
      gtsam::Point3(node->params.T_i_b.template block<3, 1>(0, 3)));

  // Initialize Rerun logger only if logging is enabled
  std::unique_ptr<rplrio::RerunLogger> rerun_logger = nullptr;
  if (logging_method != "none") {
    rerun_logger =
        std::make_unique<rplrio::RerunLogger>("bagRIO", logging_method);

    // Set up RGB color coding (R=x, G=y, B=z) for all vector3 plot series
    rerun_logger->setup_vector3_series("plots/radar_velocity");
    rerun_logger->setup_vector3_series("plots/v_rot");
    rerun_logger->setup_vector3_series("plots/v_err");
    rerun_logger->setup_vector3_series("plots/imu/gyro");
    rerun_logger->setup_vector3_series("plots/imu_bias/accel");
    rerun_logger->setup_vector3_series("plots/imu_bias/gyro");
    rerun_logger->setup_vector3_series("plots/velocity_cov");
    rerun_logger->setup_vector3_series("plots/radar_velocity_cov");
    rerun_logger->setup_vector3_series("plots/position_cov");
    rerun_logger->setup_vector3_series("plots/orientation_cov");
  }

  // Setup bag reader
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

  // Set up topic filter to only read topics we need
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

  // Print topic metadata
  auto topics = reader.get_all_topics_and_types();
  RCLCPP_INFO(node->get_logger(), "Bag contains %zu topics:", topics.size());
  for (const auto &topic : topics) {
    RCLCPP_INFO(node->get_logger(), "  - %s [%s]", topic.name.c_str(),
                topic.type.c_str());
  }

  // Serialization objects
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_serialization;

  std::vector<std::string> radar_topics;
  for (auto r : node->params.radar_list) {
    radar_topics.push_back(node->params.radars[r].topic);
  }

  // Store all poses for trajectory logging
  std::vector<Eigen::Vector3d> path_positions;
  std::vector<Eigen::Quaterniond> path_orientations;
  std::vector<double> path_timestamps;
  std::vector<Eigen::Vector3d> path_velocities;

  // Store velocity data
  std::vector<double> velocity_timestamps;
  std::vector<Eigen::Vector3d> state_velocities;
  std::vector<Eigen::Vector3d> radar_velocities;

  // Point cloud for incrementally growing trajectory
  pcl::PointCloud<pcl::PointXYZ>::Ptr path_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  while (reader.has_next() && !g_shutdown_requested) {
    auto bag_message = reader.read_next();

    try {
      // Process IMU messages
      if (bag_message->topic_name == node->params.imu_topic) {
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        imu_serialization.deserialize_message(&serialized_msg, imu_msg.get());
        node->cb_imu(imu_msg);
      }
      // Process radar/pointcloud messages
      else if (std::find(radar_topics.begin(), radar_topics.end(),
                         bag_message->topic_name) != radar_topics.end()) {
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pc_serialization.deserialize_message(&serialized_msg, pc_msg.get());

        node->cb_radar(pc_msg);

        // Process keyframes for trajectory and logging
        auto &states = node->rio_manager->get_latest_states();
        for (std::shared_ptr<rplrio::Keyframe<PointType>> kf : states) {
          double timestamp = kf->stamp;
          auto current_pose = kf->state.pose();

          current_pose = current_pose.compose(T_i_b);

          Eigen::Vector3d position = current_pose.translation();
          Eigen::Quaterniond orientation(
              current_pose.rotation().toQuaternion());
          Eigen::Vector3d velocity = kf->state.velocity();

          // save trajectory data
          path_positions.push_back(position);
          path_orientations.push_back(orientation);
          path_timestamps.push_back(timestamp);
          path_velocities.push_back(velocity);
          // Save velocity data (in body frame)
          velocity_timestamps.push_back(timestamp);
          // Rotate state velocity from global to body frame
          Eigen::Vector3d state_vel_body =
              current_pose.rotation().matrix().transpose() *
              Eigen::Vector3d(kf->state.velocity());
          state_velocities.push_back(state_vel_body);
          // Radar velocity is already in body frame
          radar_velocities.push_back(Eigen::Vector3d(kf->radar_velocity));


          // Log to Rerun only if logger is enabled
          if (rerun_logger) {
            // Log pose
            // position.z() = 0.0;
            rerun_logger->log_pose("world/base_link", position, orientation,
                                   timestamp);
            rerun_logger->log_pose("world/base_link/imu", T_i_b.translation(),
                                   T_i_b.rotation().toQuaternion(), timestamp);

            // Log the trajectory
            pcl::PointXYZ point;
            point.x = position.x();
            point.y = position.y();
            point.z = position.z();
            path_cloud->push_back(point);

            rerun_logger->log_point_cloud<pcl::PointXYZ>("world/path",
                                                         path_cloud, timestamp);

            // Log radar inlier points and features
            if (kf->radar_points && !kf->radar_points->empty()) {
              // Transform to world frame (radar_points are in IMU frame)
              Eigen::Matrix4f T_w_i =
                  kf->state.pose().matrix().template cast<float>();
              auto world_cloud = std::make_shared<pcl::PointCloud<PointType>>();
              pcl::transformPointCloud(*kf->radar_points, *world_cloud, T_w_i);
              rerun_logger->log_point_cloud<PointType>(
                  "world/radar/" + kf->name, world_cloud, timestamp);

            }

            // Log radar velocity
            rerun_logger->log_vector3("plots/radar_velocity",
                                      Eigen::Vector3d(kf->radar_velocity),
                                      timestamp);

            // Log velocity error
            gtsam::Vector3 v_rot =
                gtsam::skewSymmetric(
                    kf->imu_bias.correctGyroscope(kf->imu.gyro)) *
                node->params.radars[kf->name].T_r_i.block(0, 3, 3, 1);

            // v_err += v_rot;
            rerun_logger->log_vector3("plots/v_rot", v_rot, timestamp);
            rerun_logger->log_vector3("plots/v_err", kf->v_err, timestamp);

            // Log IMU raw data
            rerun_logger->log_vector3("plots/imu/gyro",
                                      Eigen::Vector3d(kf->imu.gyro), timestamp);

            // Log IMU bias
            rerun_logger->log_vector3(
                "plots/imu_bias/accel",
                Eigen::Vector3d(kf->imu_bias.accelerometer()), timestamp);
            rerun_logger->log_vector3("plots/imu_bias/gyro",
                                      kf->imu_bias.gyroscope(), timestamp);

            // state velocity covariance
            Eigen::Vector3d velocity_cov = kf->velocity_cov.diagonal().array();
            rerun_logger->log_vector3("plots/velocity_cov", velocity_cov,
                                      timestamp);

            // radar velocity covariance
            Eigen::Vector3d radar_velocity_cov =
                kf->radar_velocity_cov.diagonal().array();
            rerun_logger->log_vector3("plots/radar_velocity_cov",
                                      radar_velocity_cov, timestamp);

            // position covariance (last 3 elements of pose_cov diagonal)
            Eigen::Vector3d position_cov =
                kf->pose_cov.template block<3, 3>(3, 3).diagonal().array();
            rerun_logger->log_vector3("plots/position_cov", position_cov,
                                      timestamp);

            // orientation covariance (first 3 elements of pose_cov diagonal)
            Eigen::Vector3d orientation_cov =
                kf->pose_cov.template block<3, 3>(0, 0).diagonal().array();
            rerun_logger->log_vector3("plots/orientation_cov", orientation_cov,
                                      timestamp);

            // Log keypoint map in world frame
            {
              const auto &values = node->rio_manager->m_constraints.get_values();
              const double voxel_width =
                  node->rio_manager->form_params.matcher.max_dist_matching;
              const auto planar_world_map =
                  std::get<0>(node->rio_manager->m_keypoint_map)
                      .to_voxel_map(values, voxel_width);
              std::vector<rerun::Position3D> map_positions;
              for (const auto &[coords, points] : planar_world_map) {
                for (const auto &p : points) {
                  map_positions.push_back(rerun::Position3D(
                      static_cast<float>(p.x), static_cast<float>(p.y),
                      static_cast<float>(p.z)));
                }
              }
              if (!map_positions.empty()) {
                rerun_logger->stream().set_time_timestamp_secs_since_epoch(
                    "timestamp", timestamp);
                rerun_logger->stream().log("world/keypoint_map",
                                           rerun::Points3D(map_positions));
              }
            }

            // Log keypoints in world frame
            const auto &kps = node->rio_manager->get_latest_keypoints();
            const gtsam::Pose3 T_w_i = kf->state.pose();
            {
              const auto &planar = std::get<0>(kps);
              if (!planar.empty()) {
                std::vector<rerun::Position3D> positions;
                positions.reserve(planar.size());
                for (const auto &p : planar) {
                  const Eigen::Vector3d pw = T_w_i * p.vec3();
                  positions.push_back(rerun::Position3D(
                      static_cast<float>(pw.x()), static_cast<float>(pw.y()),
                      static_cast<float>(pw.z())));
                }
                rerun_logger->stream().set_time_timestamp_secs_since_epoch(
                    "timestamp", timestamp);
                rerun_logger->stream().log("world/keypoints/planar",
                                           rerun::Points3D(positions));
              }
            }
            {
              const auto &pts = std::get<1>(kps);
              if (!pts.empty()) {
                std::vector<rerun::Position3D> positions;
                positions.reserve(pts.size());
                for (const auto &p : pts) {
                  const Eigen::Vector3d pw = T_w_i * p.vec3();
                  positions.push_back(rerun::Position3D(
                      static_cast<float>(pw.x()), static_cast<float>(pw.y()),
                      static_cast<float>(pw.z())));
                }
                rerun_logger->stream().set_time_timestamp_secs_since_epoch(
                    "timestamp", timestamp);
                rerun_logger->stream().log("world/keypoints/point",
                                           rerun::Points3D(positions));
              }
            }
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

  // Save trajectory to file (with velocity in global frame)
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

  // Save velocity data to file (in body frame)
  if (!velocity_timestamps.empty()) {
    std::ofstream vel_file(velocity_save_path);
    if (vel_file.is_open()) {
      vel_file << std::fixed << std::setprecision(9);
      // Write header
      vel_file << "# timestamp state_vx state_vy state_vz radar_vx radar_vy "
                  "radar_vz\n";
      for (size_t i = 0; i < velocity_timestamps.size(); ++i) {
        const auto &state_vel = state_velocities[i];
        const auto &radar_vel = radar_velocities[i];
        vel_file << velocity_timestamps[i] << " " << state_vel.x() << " "
                 << state_vel.y() << " " << state_vel.z() << " "
                 << radar_vel.x() << " " << radar_vel.y() << " "
                 << radar_vel.z() << "\n";
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

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr
        << "Usage: " << argv[0]
        << " <bag_file_path> [logging_method] [save_path] [velocity_save_path]"
        << std::endl;
    std::cerr
        << "  logging_method: spawn (default), serve, save, connect, or none"
        << std::endl;
    std::cerr << "  save_path: Path to save trajectory file (default: "
                 "/tmp/rio_trajectory.txt)"
              << std::endl;
    std::cerr << "  velocity_save_path: Path to save velocity file (default: "
                 "/tmp/rio_velocity.txt)"
              << std::endl;
    return 1;
  }

  // Initialize ROS first to get parameters
  rclcpp::init(argc, argv);

  // Create temporary node to read config parameter
  auto temp_node = rclcpp::Node::make_shared(
      "temp_node", rclcpp::NodeOptions()
                       .allow_undeclared_parameters(true)
                       .automatically_declare_parameters_from_overrides(true));

  std::string config_path;
  if (temp_node->has_parameter("config")) {
    config_path = temp_node->get_parameter("config").as_string();
  } else {
    std::cerr << "Error: config parameter not specified" << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  temp_node.reset();

  std::string point_type = getPointTypeFromConfig(config_path);

  if (point_type.empty()) {
    std::cerr << "Error: point_type not found in config file: " << config_path
              << std::endl;
    std::cerr << "Available types: PointXYZD, PointXYZSD, PointHugin"
              << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  if (point_type == "PointXYZD") {
    return runBagDIO<pcl::PointXYZD>(argc, argv);
  } else if (point_type == "PointXYZSD") {
    return runBagDIO<pcl::PointXYZSD>(argc, argv);
  } else if (point_type == "PointHugin") {
    return runBagDIO<pcl::PointHugin>(argc, argv);
  } else {
    std::cerr << "Error: Unknown point type '" << point_type << "'"
              << std::endl;
    std::cerr << "Available types: PointXYZD, PointXYZSD, PointHugin"
              << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}