#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

class Mid360ScanProjector final : public rclcpp::Node {
 public:
  Mid360ScanProjector() : Node("g1_mid360_scan_projector") {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/mid360/points");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    min_height_ = declare_parameter<double>("min_height", 0.12);
    max_height_ = declare_parameter<double>("max_height", 1.60);
    self_min_x_ = declare_parameter<double>("self_min_x", -0.45);
    self_max_x_ = declare_parameter<double>("self_max_x", 0.45);
    self_min_y_ = declare_parameter<double>("self_min_y", -0.40);
    self_max_y_ = declare_parameter<double>("self_max_y", 0.40);
    self_min_z_ = declare_parameter<double>("self_min_z", 0.0);
    self_max_z_ = declare_parameter<double>("self_max_z", 1.80);

    range_min_ = declare_parameter<double>("range_min", 0.55);
    range_max_ = declare_parameter<double>("range_max", 10.0);
    angle_increment_ = declare_parameter<double>("angle_increment", 0.00872664626);
    transform_timeout_s_ = declare_parameter<double>("transform_timeout_s", 0.10);
    frame_stride_ = declare_parameter<int64_t>("frame_stride", 2);

    if (min_height_ >= max_height_ || range_min_ <= 0.0 || range_min_ >= range_max_ ||
        angle_increment_ <= 0.0 || angle_increment_ > 2.0 * kPi || frame_stride_ < 1) {
      throw std::runtime_error("Invalid Mid-360 scan projector parameters");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::SensorDataQoS());
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&Mid360ScanProjector::on_cloud, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Passive Mid-360 projection: %s -> %s in %s; height [%.2f, %.2f] m, "
        "self-mask x=[%.2f, %.2f], y=[%.2f, %.2f], every %ld frame",
        cloud_topic_.c_str(), scan_topic_.c_str(), base_frame_.c_str(), min_height_, max_height_,
        self_min_x_, self_max_x_, self_min_y_, self_max_y_, static_cast<long>(frame_stride_));
  }

 private:
  bool inside_self_mask(double x, double y, double z) const {
    return x >= self_min_x_ && x <= self_max_x_ && y >= self_min_y_ && y <= self_max_y_ &&
           z >= self_min_z_ && z <= self_max_z_;
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    const uint64_t frame_index = frame_index_++;
    if (frame_index % static_cast<uint64_t>(frame_stride_) != 0U) {
      return;
    }

    if (cloud->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring cloud without frame_id");
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
          base_frame_, cloud->header.frame_id, rclcpp::Time(cloud->header.stamp),
          rclcpp::Duration::from_seconds(transform_timeout_s_));
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Skipping cloud: %s", error.what());
      return;
    }

    const auto bins = static_cast<std::size_t>(std::floor((2.0 * kPi) / angle_increment_)) + 1U;
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = cloud->header.stamp;
    if (scan.header.stamp.sec == 0 && scan.header.stamp.nanosec == 0) {
      scan.header.stamp = now();
    }
    scan.header.frame_id = base_frame_;
    scan.angle_min = static_cast<float>(-kPi);
    scan.angle_max = static_cast<float>(kPi);
    scan.angle_increment = static_cast<float>(angle_increment_);
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.ranges.assign(bins, std::numeric_limits<float>::infinity());

    tf2::Quaternion rotation(
        transform.transform.rotation.x, transform.transform.rotation.y,
        transform.transform.rotation.z, transform.transform.rotation.w);
    if (rotation.length2() < 1e-12) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring cloud with invalid TF rotation");
      return;
    }
    rotation.normalize();
    const tf2::Vector3 translation(
        transform.transform.translation.x, transform.transform.translation.y,
        transform.transform.translation.z);

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x_iter(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_iter(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_iter(*cloud, "z");
      for (; x_iter != x_iter.end(); ++x_iter, ++y_iter, ++z_iter) {
        const auto x = static_cast<double>(*x_iter);
        const auto y = static_cast<double>(*y_iter);
        const auto z = static_cast<double>(*z_iter);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }

        const tf2::Vector3 point = tf2::quatRotate(rotation, tf2::Vector3(x, y, z)) + translation;
        const auto point_x = point.x();
        const auto point_y = point.y();
        const auto point_z = point.z();
        if (point_z < min_height_ || point_z > max_height_ ||
            inside_self_mask(point_x, point_y, point_z)) {
          continue;
        }

        const auto range = std::hypot(point_x, point_y);
        if (range < range_min_ || range > range_max_) {
          continue;
        }
        const auto angle = std::atan2(point_y, point_x);
        const auto index = static_cast<std::size_t>(
            std::llround((angle - static_cast<double>(scan.angle_min)) / angle_increment_));
        if (index < scan.ranges.size() && range < scan.ranges[index]) {
          scan.ranges[index] = static_cast<float>(range);
        }
      }
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "Cannot read PointCloud2 x/y/z fields: %s", error.what());
      return;
    }

    scan_pub_->publish(scan);
  }

  std::string cloud_topic_;
  std::string scan_topic_;
  std::string base_frame_;
  double min_height_{0.12};
  double max_height_{1.60};
  double self_min_x_{-0.45};
  double self_max_x_{0.45};
  double self_min_y_{-0.40};
  double self_max_y_{0.40};
  double self_min_z_{0.0};
  double self_max_z_{1.80};
  double range_min_{0.55};
  double range_max_{10.0};
  double angle_increment_{0.00872664626};
  double transform_timeout_s_{0.10};
  int64_t frame_stride_{2};
  uint64_t frame_index_{0U};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Mid360ScanProjector>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("g1_mid360_scan_projector"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
