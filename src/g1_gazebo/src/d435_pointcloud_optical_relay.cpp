#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{
class D435PointcloudOpticalRelay : public rclcpp::Node
{
public:
  D435PointcloudOpticalRelay()
  : Node("d435_pointcloud_optical_relay")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/camera/camera/points_gz");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/camera/camera/depth/color/points");
    const auto optical_frame = declare_parameter<std::string>(
      "optical_frame", "d435_color_optical_frame");

    optical_frame_ = optical_frame;
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        this->relay(*cloud);
      });
  }

private:
  void relay(const sensor_msgs::msg::PointCloud2 & input)
  {
    sensor_msgs::msg::PointCloud2 output = input;

    try {
      sensor_msgs::PointCloud2Iterator<float> x(output, "x");
      sensor_msgs::PointCloud2Iterator<float> y(output, "y");
      sensor_msgs::PointCloud2Iterator<float> z(output, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        const float gazebo_x = *x;
        const float gazebo_y = *y;
        const float gazebo_z = *z;
        // Gazebo camera: x forward, y left, z up.
        // ROS optical frame: x right, y down, z forward.
        *x = -gazebo_y;
        *y = -gazebo_z;
        *z = gazebo_x;
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "D435 point cloud does not contain float32 x/y/z fields: %s", error.what());
      return;
    }

    output.header.frame_id = optical_frame_;
    publisher_->publish(output);
  }

  std::string optical_frame_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<D435PointcloudOpticalRelay>());
  rclcpp::shutdown();
  return 0;
}
