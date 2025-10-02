#include <ceres/types.h>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "RadarPointType.h"
#include "RadarVelocityEstimatorCeres.h"
#include "utils.h"

typedef pcl::PointHugin PointType;

class Vestimate : public rclcpp::Node {
public:
  Vestimate() : Node("velociy_analysis") {
    velocity_estimator =
        std::make_shared<RadarVelocityEstimatorCeres<PointType>>();
    velocity_estimator->set_inlier_thresh(0.5);
    velocity_estimator->set_initial_velocity(Eigen::Vector3d(0, 1, 0));

    // Open a CSV file in append mode
    csv_file.open("/root/radar_stream/velocity_data.csv", std::ios::out);
    if (!csv_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file for writing.");
      return;
    }

    publisher_ =
        create_publisher<visualization_msgs::msg::Marker>("velocity", 100);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "radar/points", 100,
        std::bind(&Vestimate::topic_callback, this, std::placeholders::_1));
  }

  void on_shutdown() {
    csv_file.flush();
    csv_file.close();
  }

private:
  std::ofstream csv_file;
  uint count = 0;
  std::shared_ptr<RadarVelocityEstimatorCeres<PointType>> velocity_estimator;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<PointType>::Ptr pcl_cloud(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    auto vest = velocity_estimator->optimize(pcl_cloud);
    auto marker = rplrio::velocityMarker(vest, Eigen::Vector3d(0, 0, 0),
                                         msg->header.frame_id, "velocity");
    publisher_->publish(marker);
    RCLCPP_INFO(this->get_logger(), "%u #pc %zu v: [%f, %f, %f]", count++,
                pcl_cloud->size(), vest[0], vest[1], vest[2]);

    // Get the current time
    auto timestamp = rclcpp::Time(msg->header.stamp).seconds();

    // Write the timestamp and velocity to the CSV file
    csv_file << std::fixed << timestamp << "," << vest[0] << "," << vest[1]
             << "," << vest[2] << "\n";
    csv_file.flush();
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Vestimate>();
  rclcpp::spin(node);
  rclcpp::on_shutdown(std::bind(&Vestimate::on_shutdown, node));
  rclcpp::shutdown();
  return 0;
}