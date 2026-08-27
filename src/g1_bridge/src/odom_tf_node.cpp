#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class OdomTfNode final : public rclcpp::Node {
 public:
  OdomTfNode() : Node("g1_odom_tf") {
    input_topic_ = declare_parameter<std::string>(
        "input_odom_topic", "/state_estimator/odom_pelvis");
    output_topic_ = declare_parameter<std::string>("output_odom_topic", "/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    pelvis_frame_ = declare_parameter<std::string>("pelvis_frame", "pelvis");
    pelvis_height_ = declare_parameter<double>("pelvis_height", 0.793);
    use_input_pelvis_z_ = declare_parameter<bool>("use_input_pelvis_z", false);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    if (pelvis_height_ <= 0.0) {
      throw std::runtime_error("pelvis_height must be positive");
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&OdomTfNode::on_odom, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Adapting %s to %s and TF %s -> %s -> %s (pelvis height %.3f m%s)",
        input_topic_.c_str(), output_topic_.c_str(), odom_frame_.c_str(),
        base_frame_.c_str(), pelvis_frame_.c_str(), pelvis_height_,
        use_input_pelvis_z_ ? ", using input z" : ", fixed z");
  }

 private:
  static geometry_msgs::msg::Quaternion to_msg(const tf2::Quaternion &q) {
    geometry_msgs::msg::Quaternion result;
    result.x = q.x();
    result.y = q.y();
    result.z = q.z();
    result.w = q.w();
    return result;
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr input) {
    const auto &position = input->pose.pose.position;
    const auto &orientation = input->pose.pose.orientation;

    tf2::Quaternion full_orientation(
        orientation.x, orientation.y, orientation.z, orientation.w);
    if (full_orientation.length2() < 1e-12) {
      full_orientation.setValue(0.0, 0.0, 0.0, 1.0);
    } else {
      full_orientation.normalize();
    }

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(full_orientation).getRPY(roll, pitch, yaw);
    (void)roll;
    (void)pitch;

    tf2::Quaternion yaw_orientation;
    yaw_orientation.setRPY(0.0, 0.0, yaw);
    yaw_orientation.normalize();
    tf2::Quaternion pelvis_tilt = yaw_orientation.inverse() * full_orientation;
    pelvis_tilt.normalize();

    auto stamp = input->header.stamp;
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      stamp = now();
    }

    nav_msgs::msg::Odometry output = *input;
    output.header.stamp = stamp;
    output.header.frame_id = odom_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = position.x;
    output.pose.pose.position.y = position.y;
    output.pose.pose.position.z = 0.0;
    output.pose.pose.orientation = to_msg(yaw_orientation);
    output.twist.twist.linear.z = 0.0;
    output.twist.twist.angular.x = 0.0;
    output.twist.twist.angular.y = 0.0;
    odom_pub_->publish(output);

    if (!tf_broadcaster_) {
      return;
    }

    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header.stamp = stamp;
    odom_to_base.header.frame_id = odom_frame_;
    odom_to_base.child_frame_id = base_frame_;
    odom_to_base.transform.translation.x = position.x;
    odom_to_base.transform.translation.y = position.y;
    odom_to_base.transform.translation.z = 0.0;
    odom_to_base.transform.rotation = to_msg(yaw_orientation);

    geometry_msgs::msg::TransformStamped base_to_pelvis;
    base_to_pelvis.header.stamp = stamp;
    base_to_pelvis.header.frame_id = base_frame_;
    base_to_pelvis.child_frame_id = pelvis_frame_;
    base_to_pelvis.transform.translation.x = 0.0;
    base_to_pelvis.transform.translation.y = 0.0;
    base_to_pelvis.transform.translation.z =
        use_input_pelvis_z_ && std::isfinite(position.z) && position.z > 0.0
            ? position.z
            : pelvis_height_;
    base_to_pelvis.transform.rotation = to_msg(pelvis_tilt);

    tf_broadcaster_->sendTransform({odom_to_base, base_to_pelvis});
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string pelvis_frame_;
  double pelvis_height_{0.793};
  bool use_input_pelvis_z_{false};
  bool publish_tf_{true};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<OdomTfNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("g1_odom_tf"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
