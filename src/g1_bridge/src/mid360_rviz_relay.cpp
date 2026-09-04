#include <algorithm>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class Mid360RvizRelay final : public rclcpp::Node {
 public:
  Mid360RvizRelay() : Node("g1_mid360_rviz_relay") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/mid360/points");
    output_topic_ =
        declare_parameter<std::string>("output_topic", "/mid360/points_rviz");
    frame_stride_ = declare_parameter<int64_t>("frame_stride", 2);
    point_stride_ = declare_parameter<int64_t>("point_stride", 4);

    if (frame_stride_ < 1 || point_stride_ < 1) {
      throw std::runtime_error("frame_stride and point_stride must be positive");
    }

    // The raw cloud stays local to the robot. SensorDataQoS prevents this relay
    // from applying back-pressure to the Livox driver.
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS().keep_last(2),
        std::bind(&Mid360RvizRelay::on_cloud, this, std::placeholders::_1));

    // The reduced cloud is small enough for reliable delivery over Wi-Fi.
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_, rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile());

    RCLCPP_INFO(
        get_logger(), "RViz cloud relay: %s -> %s, every %ld frame, every %ld point",
        input_topic_.c_str(), output_topic_.c_str(), static_cast<long>(frame_stride_),
        static_cast<long>(point_stride_));
  }

 private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
    const uint64_t frame_index = frame_index_++;
    if (frame_index % static_cast<uint64_t>(frame_stride_) != 0U) {
      return;
    }

    if (cloud->point_step == 0U || cloud->width == 0U || cloud->height == 0U) {
      return;
    }
    if (cloud->width > std::numeric_limits<std::size_t>::max() / cloud->height) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Cloud dimensions overflow");
      return;
    }

    const std::size_t total_points =
        static_cast<std::size_t>(cloud->width) * static_cast<std::size_t>(cloud->height);
    const std::size_t stride = static_cast<std::size_t>(point_stride_);
    const std::size_t selected_points = (total_points + stride - 1U) / stride;
    if (selected_points > std::numeric_limits<std::size_t>::max() / cloud->point_step) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Reduced cloud size overflow");
      return;
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header = cloud->header;
    output.height = 1U;
    output.width = static_cast<uint32_t>(selected_points);
    output.fields = cloud->fields;
    output.is_bigendian = cloud->is_bigendian;
    output.point_step = cloud->point_step;
    output.row_step = output.width * output.point_step;
    output.is_dense = cloud->is_dense;
    output.data.resize(static_cast<std::size_t>(output.row_step));

    std::size_t destination_index = 0U;
    for (std::size_t source_index = 0U; source_index < total_points;
         source_index += stride) {
      const std::size_t row = source_index / cloud->width;
      const std::size_t column = source_index % cloud->width;
      const std::size_t source_offset =
          row * static_cast<std::size_t>(cloud->row_step) +
          column * static_cast<std::size_t>(cloud->point_step);
      if (source_offset > cloud->data.size() ||
          cloud->data.size() - source_offset < cloud->point_step) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "Ignoring malformed PointCloud2 payload");
        return;
      }
      std::memcpy(
          output.data.data() + destination_index * output.point_step,
          cloud->data.data() + source_offset, output.point_step);
      ++destination_index;
    }

    output.width = static_cast<uint32_t>(destination_index);
    output.row_step = output.width * output.point_step;
    output.data.resize(static_cast<std::size_t>(output.row_step));
    cloud_pub_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  int64_t frame_stride_{2};
  int64_t point_stride_{4};
  uint64_t frame_index_{0U};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mid360RvizRelay>());
  rclcpp::shutdown();
  return 0;
}
