#define PCL_NO_PRECOMPILE

#include <atomic>
#include <fstream>
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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <yaml-cpp/yaml.h>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "core/RadarPointType.hpp"
#include "node/RerunLogger.hpp"
#include "form/formRadar.hpp"
#include "core/utils.hpp"

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

template <typename PointType>
int runBagForm(const std::string &config_path, int argc, char *argv[]) {
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  std::string bag_path = argv[1];
  std::string logging_method = (argc >= 3) ? argv[2] : "none";
  std::string save_path = (argc >= 4) ? argv[3] : "/tmp/form_trajectory.txt";

  rplrio::Params params = rplrio::get_params(config_path);

  auto T_i_b = gtsam::Pose3(
      gtsam::Rot3(params.R_i_b),
      gtsam::Point3(params.T_i_b.template block<3, 1>(0, 3)));

  // Initialize RadarEstimator with default params
  rplrio::RadarEstimator::Params est_params;
  rplrio::RadarEstimator estimator(est_params);

  // Initialize Rerun logger
  std::unique_ptr<rplrio::RerunLogger> rerun_logger = nullptr;
  if (logging_method != "none") {
    rerun_logger =
        std::make_unique<rplrio::RerunLogger>("runform", logging_method);
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
    std::cerr << "Failed to open bag file: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "Reading from bag: " << bag_path << std::endl;

  // Filter to radar topics only; build topic -> radar_name lookup
  rosbag2_storage::StorageFilter storage_filter;
  std::vector<std::string> radar_topics;
  std::unordered_map<std::string, std::string> topic_to_radar;
  for (const auto &r : params.radar_list) {
    const auto &topic = params.radars[r].topic;
    storage_filter.topics.push_back(topic);
    radar_topics.push_back(topic);
    topic_to_radar[topic] = r;
  }
  reader.set_filter(storage_filter);

  std::cout << "Filtering topics:" << std::endl;
  for (const auto &t : radar_topics) {
    std::cout << "  - " << t << std::endl;
  }

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_serialization;

  // Trajectory storage
  std::vector<Eigen::Vector3d> path_positions;
  std::vector<Eigen::Quaterniond> path_orientations;
  std::vector<double> path_timestamps;

  // Incrementally growing path cloud for Rerun
  pcl::PointCloud<pcl::PointXYZ>::Ptr path_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  while (reader.has_next() && !g_shutdown_requested) {
    auto bag_message = reader.read_next();

    if (std::find(radar_topics.begin(), radar_topics.end(),
                  bag_message->topic_name) == radar_topics.end()) {
      continue;
    }

    try {
      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
      auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pc_serialization.deserialize_message(&serialized_msg, pc_msg.get());

      double timestamp = rclcpp::Time(pc_msg->header.stamp).seconds();

      auto cloud = std::make_shared<pcl::PointCloud<PointType>>();
      pcl::fromROSMsg(*pc_msg, *cloud);

      if (cloud->empty()) {
        std::cerr << "Empty cloud at t=" << timestamp << std::endl;
        continue;
      }

      // Transform radar points to IMU frame before registration
      const std::string &radar_name = topic_to_radar.at(bag_message->topic_name);
      const Eigen::Matrix4f T_i_r =
          params.radars[radar_name].T_r_i.inverse().cast<float>();
      auto imu_cloud = std::make_shared<pcl::PointCloud<PointType>>();
      pcl::transformPointCloud(*cloud, *imu_cloud, T_i_r);

      // Run FORM registration in IMU frame
      auto [planar_feats, point_feats] =
          estimator.register_scan<PointType>(imu_cloud);

      // Get current pose estimate and convert IMU frame -> base_link
      gtsam::Pose3 current_pose =
          estimator.current_lidar_estimate().compose(T_i_b);
      Eigen::Vector3d position = current_pose.translation();
      Eigen::Quaterniond orientation(current_pose.rotation().toQuaternion());

      std::cout << "t=" << std::fixed << std::setprecision(3) << timestamp
                << "  pos=(" << position.x() << ", " << position.y() << ", "
                << position.z() << ")"
                << "  planar=" << planar_feats.size()
                << "  point=" << point_feats.size() << std::endl;

      path_positions.push_back(position);
      path_orientations.push_back(orientation);
      path_timestamps.push_back(timestamp);

      if (rerun_logger) {
        // Log pose
        rerun_logger->log_pose("world/sensor", position, orientation,
                               timestamp);

        // Log growing path
        pcl::PointXYZ pt;
        pt.x = position.x();
        pt.y = position.y();
        pt.z = position.z();
        path_cloud->push_back(pt);
        rerun_logger->log_point_cloud<pcl::PointXYZ>("world/path", path_cloud,
                                                     timestamp);

        const Eigen::Matrix4f T_w_s = current_pose.matrix().cast<float>();
        const Eigen::Matrix3f R_w_s = T_w_s.block<3, 3>(0, 0);
        const Eigen::Vector3f t_w_s = T_w_s.block<3, 1>(0, 3);

        // Log raw radar cloud in world frame
        auto world_cloud = std::make_shared<pcl::PointCloud<PointType>>();
        pcl::transformPointCloud(*imu_cloud, *world_cloud, T_w_s);
        rerun_logger->log_point_cloud<PointType>("world/radar", world_cloud,
                                                 timestamp);

        // Log planar features with normals
        if (!planar_feats.empty()) {
          std::vector<rerun::Position3D> planar_positions;
          std::vector<rerun::Position3D> arrow_origins;
          std::vector<rerun::Vector3D> arrow_vecs;
          planar_positions.reserve(planar_feats.size());
          arrow_origins.reserve(planar_feats.size());
          arrow_vecs.reserve(planar_feats.size());

          for (const auto &pf : planar_feats) {
            Eigen::Vector3f p_w =
                R_w_s * Eigen::Vector3f(pf.x, pf.y, pf.z) + t_w_s;
            Eigen::Vector3f n_w =
                R_w_s * Eigen::Vector3f(pf.nx, pf.ny, pf.nz);
            planar_positions.emplace_back(p_w.x(), p_w.y(), p_w.z());
            arrow_origins.emplace_back(p_w.x(), p_w.y(), p_w.z());
            arrow_vecs.emplace_back(n_w.x() * 0.3f, n_w.y() * 0.3f,
                                    n_w.z() * 0.3f);
          }

          rerun_logger->stream().set_time_timestamp_secs_since_epoch(
              "timestamp", timestamp);
          rerun_logger->stream().log(
              "world/features/planar",
              rerun::Points3D(planar_positions)
                  .with_colors({{255, 255, 0, 255}})
                  .with_radii({0.05f}));
          rerun_logger->stream().log(
              "world/features/planar_normals",
              rerun::Arrows3D::from_vectors(arrow_vecs)
                  .with_origins(arrow_origins)
                  .with_colors({{255, 255, 0, 255}}));
        }

        // Log point features
        if (!point_feats.empty()) {
          std::vector<rerun::Position3D> point_positions;
          point_positions.reserve(point_feats.size());

          for (const auto &pf : point_feats) {
            Eigen::Vector3f p_w =
                R_w_s * Eigen::Vector3f(pf.x, pf.y, pf.z) + t_w_s;
            point_positions.emplace_back(p_w.x(), p_w.y(), p_w.z());
          }

          rerun_logger->stream().set_time_timestamp_secs_since_epoch(
              "timestamp", timestamp);
          rerun_logger->stream().log(
              "world/features/points",
              rerun::Points3D(point_positions)
                  .with_colors({{255, 0, 0, 255}})
                  .with_radii({0.05f}));
        }
      }
    } catch (const std::exception &e) {
      std::cerr << "Error processing '" << bag_message->topic_name
                << "': " << e.what() << std::endl;
      continue;
    }
  }

  if (g_shutdown_requested) {
    std::cout << "Processing interrupted by user" << std::endl;
  } else {
    std::cout << "Finished processing bag file" << std::endl;
  }

  // Save trajectory to file
  if (!path_positions.empty()) {
    std::ofstream traj_file(save_path);
    if (traj_file.is_open()) {
      traj_file << std::fixed << std::setprecision(9);
      for (size_t i = 0; i < path_positions.size(); ++i) {
        const auto &pos = path_positions[i];
        const auto &q = path_orientations[i];
        traj_file << path_timestamps[i] << " " << pos.x() << " " << pos.y()
                  << " " << pos.z() << " " << q.x() << " " << q.y() << " "
                  << q.z() << " " << q.w() << "\n";
      }
      traj_file.close();
      std::cout << "Trajectory saved to: " << save_path << " ("
                << path_positions.size() << " poses)" << std::endl;
    } else {
      std::cerr << "Failed to open trajectory file: " << save_path << std::endl;
    }
  }

  rclcpp::shutdown();
  return 0;
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <bag_file_path> [logging_method] [save_path]" << std::endl;
    std::cerr
        << "  logging_method: spawn (default), serve, save, connect, or none"
        << std::endl;
    std::cerr << "  save_path: Path to save trajectory file (default: "
                 "/tmp/form_trajectory.txt)"
              << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);

  auto temp_node = rclcpp::Node::make_shared(
      "temp_node",
      rclcpp::NodeOptions()
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
    return runBagForm<pcl::PointXYZD>(config_path, argc, argv);
  } else if (point_type == "PointXYZSD") {
    return runBagForm<pcl::PointXYZSD>(config_path, argc, argv);
  } else if (point_type == "PointHugin") {
    return runBagForm<pcl::PointHugin>(config_path, argc, argv);
  } else {
    std::cerr << "Error: Unknown point type '" << point_type << "'"
              << std::endl;
    std::cerr << "Available types: PointXYZD, PointXYZSD, PointHugin"
              << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
