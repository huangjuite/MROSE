
#include "RosNode/DIORosNode.h"
#include "RosNode/RerunLogger.h"
#include <atomic>
#include <chrono>
#include <errno.h>
#include <fcntl.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <signal.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "RadarPointType.h"

typedef pcl::PointHugin PointType;

// Signal handler for clean shutdown
std::atomic<bool> g_shutdown_requested(false);
std::atomic<bool> g_paused(false);

void signal_handler(int signum) {
  (void)signum;
  g_shutdown_requested = true;
  std::cout << "\nShutdown requested, stopping..." << std::endl;
}

// Set terminal to non-blocking mode
void set_nonblocking_input(bool enable) {
  static struct termios old_term, new_term;
  static int old_flags;

  if (enable) {
    // Get current terminal settings
    tcgetattr(STDIN_FILENO, &old_term);
    new_term = old_term;

    // Disable canonical mode and echo
    new_term.c_lflag &= ~(ICANON | ECHO);

    // Set read to return immediately with whatever is available
    new_term.c_cc[VMIN] = 0;
    new_term.c_cc[VTIME] = 0;

    tcsetattr(STDIN_FILENO, TCSANOW, &new_term);

    // Set non-blocking
    old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);
  } else {
    // Restore original settings
    tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
    fcntl(STDIN_FILENO, F_SETFL, old_flags);
  }
}

// Check for space key press
void check_keyboard_input() {
  char c;
  ssize_t n = read(STDIN_FILENO, &c, 1);
  if (n > 0) {
    if (c == ' ') {
      g_paused = !g_paused;
      if (g_paused) {
        std::cout << "\n[PAUSED] Press space to resume..." << std::endl;
        std::cout.flush();
      } else {
        std::cout << "[RESUMED] Processing bag..." << std::endl;
        std::cout.flush();
      }
    } else if (c == 'q' || c == 'Q') {
      g_shutdown_requested = true;
      std::cout << "\nQuit requested..." << std::endl;
      std::cout.flush();
    }
  }
}

int main(int argc, char *argv[]) {
  // Setup signal handlers
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <bag_file_path> [logging_method]"
              << std::endl;
    std::cerr
        << "  logging_method: spawn (default), serve, save, connect, or none"
        << std::endl;
    return 1;
  }

  std::string bag_path = argv[1];
  std::string logging_method = (argc >= 3) ? argv[2] : "none";

  // Create the node
  auto node = std::make_shared<DIORosNode<PointType>>();
  auto T_i_b =
      gtsam::Pose3(gtsam::Rot3(node->params.R_i_b),
                   gtsam::Point3(node->params.T_i_b.block<3, 1>(0, 3)));

  // Initialize Rerun logger only if logging is enabled
  std::unique_ptr<rplrio::RerunLogger> rerun_logger = nullptr;
  if (logging_method != "none") {
    rerun_logger =
        std::make_unique<rplrio::RerunLogger>("bagDIO", logging_method);
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

  // Print topic metadata
  auto topics = reader.get_all_topics_and_types();
  RCLCPP_INFO(node->get_logger(), "Bag contains %zu topics:", topics.size());
  for (const auto &topic : topics) {
    RCLCPP_INFO(node->get_logger(), "  - %s [%s]", topic.name.c_str(),
                topic.type.c_str());
  }

  // Enable non-blocking keyboard input
  if (isatty(STDIN_FILENO)) {
    set_nonblocking_input(true);
    std::cout << "\n=== Controls ===" << std::endl;
    std::cout << "  SPACE: Pause/Resume playback" << std::endl;
    std::cout << "  Q: Quit" << std::endl;
    std::cout << "  Ctrl+C: Emergency stop" << std::endl;
    std::cout << "================\n" << std::endl;
    std::cout.flush();
  } else {
    std::cout
        << "\n[WARNING] Keyboard input disabled (not running in a terminal)"
        << std::endl;
    std::cout << "  Use Ctrl+C to stop" << std::endl;
    std::cout.flush();
  }

  // Serialization objects
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_serialization;

  std::vector<std::string> radar_topics;
  for (auto r : node->params.radar_list) {
    radar_topics.push_back(node->params.radars[r].topic);
  }

  // Read and process messages

  // Store all poses for trajectory logging
  std::vector<Eigen::Vector3d> trajectory_positions;
  std::vector<double> trajectory_timestamps;

  // Point cloud for incrementally growing trajectory
  pcl::PointCloud<pcl::PointXYZ>::Ptr path_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  bool keyboard_enabled = isatty(STDIN_FILENO);
  while (reader.has_next() && !g_shutdown_requested) {
    // Check for keyboard input only if running in a terminal
    if (keyboard_enabled) {
      check_keyboard_input();
    }

    // If paused, wait and continue checking for input
    if (g_paused) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

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

        // Log keyframes to Rerun
        if (rerun_logger) {
          auto states = node->dio_manager->get_latest_states();
          for (auto kf : states) {
            double timestamp = kf->stamp;
            auto current_pose = kf->state.pose();

            current_pose = current_pose.compose(T_i_b);

            // Log pose
            Eigen::Vector3d position = current_pose.translation();
            Eigen::Quaterniond orientation(
                current_pose.rotation().toQuaternion());
            rerun_logger->log_pose("world/base_link", position, orientation,
                                   timestamp);
            rerun_logger->log_pose("world/base_link/imu", T_i_b.translation(),
                                   T_i_b.rotation().toQuaternion(), timestamp);

            // trajectory
            trajectory_positions.push_back(position);
            trajectory_timestamps.push_back(timestamp);

            // Log the trajectory
            pcl::PointXYZ point;
            point.x = position.x();
            point.y = position.y();
            point.z = position.z();
            path_cloud->push_back(point);

            rerun_logger->log_point_cloud<pcl::PointXYZ>("world/path",
                                                         path_cloud, timestamp);

            // Log radar inlier points
            if (kf->radar_points && !kf->radar_points->empty()) {
              rerun_logger->log_point_cloud<PointType>(
                  "world/base_link/imu/radar_inliers", kf->radar_points,
                  timestamp);
            }

            // Log radar velocity
            rerun_logger->log_vector3("plots/radar_velocity",
                                      Eigen::Vector3d(kf->radar_velocity),
                                      timestamp);

            // Log velocity error (state velocity - radar velocity)
            gtsam::Vector3 v_err =
                kf->state.bodyVelocity() - kf->radar_velocity;
            rerun_logger->log_vector3("plots/v_err", Eigen::Vector3d(v_err),
                                      timestamp);

            // Log IMU bias
            rerun_logger->log_vector3(
                "plots/imu_bias/accel",
                Eigen::Vector3d(kf->imu_bias.accelerometer()), timestamp);
            rerun_logger->log_vector3("plots/imu_bias/gyro",
                                      Eigen::Vector3d(kf->imu_bias.gyroscope()),
                                      timestamp);
          }
        }
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node->get_logger(),
                   "Failed to deserialize message from topic '%s': %s",
                   bag_message->topic_name.c_str(), e.what());
      continue;
    }
  }

  // Restore terminal settings
  if (keyboard_enabled) {
    set_nonblocking_input(false);
  }

  if (g_shutdown_requested) {
    RCLCPP_INFO(node->get_logger(), "Processing interrupted by user");
  } else {
    RCLCPP_INFO(node->get_logger(), "Finished processing bag file");
  }

  rclcpp::shutdown();
  return 0;
}