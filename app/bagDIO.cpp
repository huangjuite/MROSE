
#include "RosNode/DIORosNode.h"
#include "RosNode/RerunLogger.h"
#include <atomic>
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
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <chrono>

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
  if (read(STDIN_FILENO, &c, 1) == 1) {
    if (c == ' ') {
      g_paused = !g_paused;
      if (g_paused) {
        std::cout << "\n[PAUSED] Press space to resume..." << std::endl;
      } else {
        std::cout << "[RESUMED] Processing bag..." << std::endl;
      }
    }
  }
}

int main(int argc, char *argv[]) {
  // Setup signal handlers
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <bag_file_path> [logging_method]" << std::endl;
    std::cerr << "  logging_method: spawn (default), serve, save, or connect" << std::endl;
    return 1;
  }

  std::string bag_path = argv[1];
  std::string logging_method = (argc >= 3) ? argv[2] : "spawn";

  // Create the node
  auto node = std::make_shared<DIORosNode<PointType>>();

  // Initialize Rerun logger
  auto rerun_logger = std::make_unique<rplrio::RerunLogger>("bagDIO", logging_method);

  // Setup bag reader
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path;
  storage_options.storage_id = "mcap";

  rosbag2_cpp::ConverterOptions converter_options{};
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

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
  set_nonblocking_input(true);
  std::cout << "\nPress SPACE to pause/resume playback" << std::endl;

  // Serialization objects
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_serialization;

  // Read and process messages
  while (reader.has_next() && !g_shutdown_requested) {
    // Check for keyboard input
    check_keyboard_input();

    // If paused, wait and continue checking for input
    if (g_paused) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    auto bag_message = reader.read_next();

    try {
      // Process IMU messages
      if (bag_message->topic_name.find("imu") != std::string::npos) {
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        imu_serialization.deserialize_message(&serialized_msg, imu_msg.get());

        node->cb_imu(imu_msg);
      }
      // Process radar/pointcloud messages
      else if (bag_message->topic_name.find("radar_data") !=
               std::string::npos) {
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
        auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pc_serialization.deserialize_message(&serialized_msg, pc_msg.get());

        node->cb_radar(pc_msg);

        // Log keyframes to Rerun 
        auto states = node->dio_manager->get_latest_states();
        for (auto kf : states) {
          double timestamp = kf->stamp;

          // Log pose (position + orientation)
          auto current_pose = kf->state.pose();
          Eigen::Vector3d position = current_pose.translation();
          Eigen::Quaterniond orientation(current_pose.rotation().toQuaternion());
          rerun_logger->log_pose("world/base_link", position, orientation, timestamp);

          // Log radar inlier points
          if (kf->radar_points && !kf->radar_points->empty()) {
            rerun_logger->log_point_cloud<PointType>("world/radar_inliers",
                                                      kf->radar_points, timestamp);
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
  set_nonblocking_input(false);

  if (g_shutdown_requested) {
    RCLCPP_INFO(node->get_logger(), "Processing interrupted by user");
  } else {
    RCLCPP_INFO(node->get_logger(), "Finished processing bag file");
  }

  rclcpp::shutdown();
  return 0;
}