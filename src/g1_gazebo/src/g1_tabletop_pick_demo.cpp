#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

namespace
{
struct Vec3
{
  double x{};
  double y{};
  double z{};
};

struct Mat3
{
  double m[3][3]{};
};

struct Transform
{
  Mat3 r{};
  Vec3 t{};
};

enum JointIndex : std::size_t
{
  kWaistYaw,
  kWaistRoll,
  kWaistPitch,
  kLeftShoulderPitch,
  kLeftShoulderRoll,
  kLeftShoulderYaw,
  kLeftElbow,
  kLeftWristRoll,
  kLeftWristPitch,
  kLeftWristYaw,
  kRightShoulderPitch,
  kRightShoulderRoll,
  kRightShoulderYaw,
  kRightElbow,
  kRightWristRoll,
  kRightWristPitch,
  kRightWristYaw,
  kJointCount
};

using Pose = std::array<double, kJointCount>;

const std::array<const char *, kJointCount> kJointNames{{
  "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
  "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
  "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
  "left_wrist_yaw_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
  "right_wrist_pitch_joint", "right_wrist_yaw_joint",
}};

const Pose kDownPose{};

const std::array<double, kJointCount> kLowerLimits{{
  -2.618, -0.52, -0.52,
  -3.0892, -1.5882, -2.618, -1.0472, -1.972222054, -1.614429558, -1.614429558,
  -3.0892, -2.2515, -2.618, -1.0472, -1.972222054, -1.614429558, -1.614429558,
}};

const std::array<double, kJointCount> kUpperLimits{{
  2.618, 0.52, 0.52,
  2.6704, 2.2515, 2.618, 2.0944, 1.972222054, 1.614429558, 1.614429558,
  2.6704, 1.5882, 2.618, 2.0944, 1.972222054, 1.614429558, 1.614429558,
}};

Vec3 operator+(const Vec3 & a, const Vec3 & b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3 & a, const Vec3 & b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(const Vec3 & a, const double s)
{
  return {a.x * s, a.y * s, a.z * s};
}

double dot(const Vec3 & a, const Vec3 & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

double norm(const Vec3 & a)
{
  return std::sqrt(dot(a, a));
}

Mat3 identity_rotation()
{
  return {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
}

Mat3 multiply(const Mat3 & a, const Mat3 & b)
{
  Mat3 result{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        result.m[i][j] += a.m[i][k] * b.m[k][j];
      }
    }
  }
  return result;
}

Vec3 rotate(const Mat3 & r, const Vec3 & v)
{
  return {
    r.m[0][0] * v.x + r.m[0][1] * v.y + r.m[0][2] * v.z,
    r.m[1][0] * v.x + r.m[1][1] * v.y + r.m[1][2] * v.z,
    r.m[2][0] * v.x + r.m[2][1] * v.y + r.m[2][2] * v.z};
}

Mat3 rotation_x(const double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return {{{1.0, 0.0, 0.0}, {0.0, c, -s}, {0.0, s, c}}};
}

Mat3 rotation_y(const double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return {{{c, 0.0, s}, {0.0, 1.0, 0.0}, {-s, 0.0, c}}};
}

Mat3 rotation_z(const double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return {{{c, -s, 0.0}, {s, c, 0.0}, {0.0, 0.0, 1.0}}};
}

Mat3 rotation_rpy(const Vec3 & rpy)
{
  return multiply(multiply(rotation_z(rpy.z), rotation_y(rpy.y)), rotation_x(rpy.x));
}

Mat3 rotation_axis(const Vec3 & axis, const double angle)
{
  const double axis_norm = norm(axis);
  if (axis_norm < 1e-9) {
    return identity_rotation();
  }
  const double x = axis.x / axis_norm;
  const double y = axis.y / axis_norm;
  const double z = axis.z / axis_norm;
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  const double one_minus_c = 1.0 - c;
  return {{
    {c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s},
    {y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s},
    {z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c},
  }};
}

Transform identity_transform()
{
  return {identity_rotation(), {0.0, 0.0, 0.0}};
}

Transform make_transform(const Vec3 & xyz, const Vec3 & rpy)
{
  return {rotation_rpy(rpy), xyz};
}

Transform compose(const Transform & a, const Transform & b)
{
  return {multiply(a.r, b.r), a.t + rotate(a.r, b.t)};
}

Vec3 apply(const Transform & t, const Vec3 & p)
{
  return t.t + rotate(t.r, p);
}

struct JointSpec
{
  JointIndex index{};
  Vec3 origin{};
  Vec3 rpy{};
  Vec3 axis{};
  bool revolute{true};
};

Transform apply_joint(const Transform & input, const JointSpec & joint, const Pose & pose)
{
  Transform output = compose(input, make_transform(joint.origin, joint.rpy));
  if (joint.revolute) {
    output = compose(output, {rotation_axis(joint.axis, pose[joint.index]), {0.0, 0.0, 0.0}});
  }
  return output;
}

std::vector<JointSpec> arm_chain(const bool left)
{
  const double side = left ? 1.0 : -1.0;
  const JointIndex shoulder_pitch = left ? kLeftShoulderPitch : kRightShoulderPitch;
  const JointIndex shoulder_roll = left ? kLeftShoulderRoll : kRightShoulderRoll;
  const JointIndex shoulder_yaw = left ? kLeftShoulderYaw : kRightShoulderYaw;
  const JointIndex elbow = left ? kLeftElbow : kRightElbow;
  const JointIndex wrist_roll = left ? kLeftWristRoll : kRightWristRoll;
  const JointIndex wrist_pitch = left ? kLeftWristPitch : kRightWristPitch;
  const JointIndex wrist_yaw = left ? kLeftWristYaw : kRightWristYaw;

  return {
    {kWaistYaw, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, true},
    {kWaistRoll, {-0.0039635, 0.0, 0.044}, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, true},
    {kWaistPitch, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, true},
    {shoulder_pitch, {0.0039563, side * 0.100215, 0.24778},
      {side * 0.27931, 5.4949e-05, side * -0.00019159}, {0.0, 1.0, 0.0}, true},
    {shoulder_roll, {0.0, side * 0.038, -0.013831},
      {side * -0.27925, 0.0, 0.0}, {1.0, 0.0, 0.0}, true},
    {shoulder_yaw, {0.0, side * 0.00624, -0.1032}, {0.0, 0.0, 0.0},
      {0.0, 0.0, 1.0}, true},
    {elbow, {0.015783, 0.0, -0.080518}, {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, true},
    {wrist_roll, {0.100, side * 0.00188791, -0.010}, {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0}, true},
    {wrist_pitch, {0.038, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, true},
    {wrist_yaw, {0.046, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, true},
    {wrist_yaw, {0.0415, side * 0.003, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, false},
  };
}

Vec3 hand_position(const bool left, const Pose & pose)
{
  Transform transform = identity_transform();
  for (const auto & joint : arm_chain(left)) {
    transform = apply_joint(transform, joint, pose);
  }
  return transform.t;
}

Transform camera_optical_to_pelvis_transform()
{
  Transform transform = identity_transform();
  transform = compose(transform, make_transform({-0.0039635, 0.0, 0.044}, {0.0, 0.0, 0.0}));
  transform = compose(
    transform,
    make_transform({0.0576235, 0.01753, 0.42987}, {0.0, 0.8307767239493009, 0.0}));
  transform = compose(
    transform,
    make_transform({0.0, 0.0, 0.0}, {-M_PI / 2.0, 0.0, -M_PI / 2.0}));
  return transform;
}

Vec3 camera_optical_to_pelvis(const Vec3 & point)
{
  static const Transform transform = camera_optical_to_pelvis_transform();
  return apply(transform, point);
}

std::array<JointIndex, 7> arm_indices(const bool left)
{
  if (left) {
    return {kLeftShoulderPitch, kLeftShoulderRoll, kLeftShoulderYaw, kLeftElbow,
      kLeftWristRoll, kLeftWristPitch, kLeftWristYaw};
  }
  return {kRightShoulderPitch, kRightShoulderRoll, kRightShoulderYaw, kRightElbow,
    kRightWristRoll, kRightWristPitch, kRightWristYaw};
}

Pose seed_pose(const bool left, const double shoulder_pitch, const double shoulder_roll,
               const double shoulder_yaw, const double elbow, const double wrist_pitch)
{
  Pose pose = kDownPose;
  const auto indices = arm_indices(left);
  pose[indices[0]] = shoulder_pitch;
  pose[indices[1]] = shoulder_roll;
  pose[indices[2]] = shoulder_yaw;
  pose[indices[3]] = elbow;
  pose[indices[5]] = wrist_pitch;
  return pose;
}

struct IkResult
{
  Pose pose{};
  double error{std::numeric_limits<double>::infinity()};
};

IkResult solve_arm_ik(const bool left, const Vec3 & target, const Pose & warm_start)
{
  const auto indices = arm_indices(left);
  std::vector<Pose> candidates;
  candidates.emplace_back(warm_start);
  candidates.emplace_back(seed_pose(left, -0.65, left ? 0.10 : -0.10, left ? -0.20 : 0.20, 0.87, 0.02));
  candidates.emplace_back(seed_pose(left, -0.95, left ? 0.35 : -0.35, left ? -0.15 : 0.15, 1.05, -0.10));
  candidates.emplace_back(seed_pose(left, -0.45, left ? 0.20 : -0.20, left ? -0.35 : 0.35, 0.65, 0.10));

  IkResult best;
  for (auto candidate : candidates) {
    for (int iteration = 0; iteration < 120; ++iteration) {
      const Vec3 current = hand_position(left, candidate);
      const Vec3 error = target - current;
      if (norm(error) < 0.018) {
        break;
      }

      std::array<double, 7> gradients{};
      constexpr double kFiniteDifferenceStep = 1e-4;
      for (std::size_t i = 0; i < indices.size(); ++i) {
        Pose perturbed = candidate;
        perturbed[indices[i]] += kFiniteDifferenceStep;
        const Vec3 moved = hand_position(left, perturbed);
        const Vec3 column = (moved - current) * (1.0 / kFiniteDifferenceStep);
        gradients[i] = dot(column, error) - 0.008 * candidate[indices[i]];
      }

      const double gain = 0.42;
      for (std::size_t i = 0; i < indices.size(); ++i) {
        const JointIndex joint = indices[i];
        candidate[joint] = std::clamp(
          candidate[joint] + gain * gradients[i], kLowerLimits[joint], kUpperLimits[joint]);
      }
    }

    const double final_error = norm(target - hand_position(left, candidate));
    if (final_error < best.error) {
      best = {candidate, final_error};
    }
  }
  return best;
}

Pose merge_arm(const Pose & base, const Pose & arm_pose, const bool left)
{
  Pose result = base;
  for (const auto joint : arm_indices(left)) {
    result[joint] = arm_pose[joint];
  }
  return result;
}

Pose solve_bimanual_pose(const Vec3 & left_target, const Vec3 & right_target, const Pose & seed,
                         double * left_error, double * right_error)
{
  Pose result = seed;
  result[kWaistYaw] = 0.0;
  result[kWaistRoll] = 0.0;
  result[kWaistPitch] = 0.0;

  const IkResult left = solve_arm_ik(true, left_target, result);
  result = merge_arm(result, left.pose, true);
  const IkResult right = solve_arm_ik(false, right_target, result);
  result = merge_arm(result, right.pose, false);

  *left_error = left.error;
  *right_error = right.error;
  return result;
}

Pose interpolate(const Pose & from, const Pose & to, const double alpha)
{
  Pose result = from;
  const double t = std::clamp(alpha, 0.0, 1.0);
  for (std::size_t i = 0; i < result.size(); ++i) {
    result[i] = from[i] + (to[i] - from[i]) * t;
  }
  return result;
}
}  // namespace

class TabletopPickDemo final : public rclcpp::Node
{
public:
  TabletopPickDemo()
  : Node("g1_tabletop_pick_demo")
  {
    auto_start_ = declare_parameter<bool>("auto_start", true);
    for (const auto joint : kJointNames) {
      joint_publishers_.emplace(
        joint, create_publisher<std_msgs::msg::Float64>(
          "/g1/task/joints/" + std::string(joint) + "/command", rclcpp::QoS(1)));
    }
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/g1/task/status", rclcpp::QoS(1).transient_local());
    box_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/g1/task/box_pose", rclcpp::QoS(1).transient_local());
    box_pose_pelvis_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/g1/task/box_pose_pelvis", rclcpp::QoS(1).transient_local());
    trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/g1/task/bimanual_trajectory", rclcpp::QoS(1).transient_local());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/g1/task/markers", rclcpp::QoS(1).transient_local());
    attach_pub_ = create_publisher<std_msgs::msg::Empty>("/g1/task/grasp/attach", rclcpp::QoS(1));
    detach_pub_ = create_publisher<std_msgs::msg::Empty>("/g1/task/grasp/detach", rclcpp::QoS(1));
    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/g1/task/start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        handle_start(response);
      });
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "/g1/task/reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        handle_reset(response);
      });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/camera/depth/color/points", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) { detect_box(*cloud); });
    tick_timer_ = create_wall_timer(100ms, [this] { tick(); });

    plan_ = {kDownPose, kDownPose, kDownPose, kDownPose};
    state_ = auto_start_ ? State::kWaitingForBox : State::kWaitingForStart;
    publish_pose(kDownPose);
    publish_status(
      auto_start_ ? "WAITING_FOR_DEPTH_BOX: both arms are down" :
      "WAITING_FOR_START: both arms are down; call /g1/task/start");
  }

private:
  enum class State { kWaitingForStart, kWaitingForBox, kExecuting, kLifted, kFailed };

  struct BoxObservation
  {
    Vec3 center_optical{};
    Vec3 min_optical{};
    Vec3 max_optical{};
    std::size_t points{};
  };

  void publish_pose(const Pose & pose)
  {
    for (std::size_t i = 0; i < kJointNames.size(); ++i) {
      std_msgs::msg::Float64 command;
      command.data = pose[i];
      joint_publishers_.at(kJointNames[i])->publish(command);
    }
  }

  void publish_status(const std::string & text)
  {
    std_msgs::msg::String status;
    status.data = text;
    status_pub_->publish(status);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  void publish_box_pose(const std::string & frame_id, const builtin_interfaces::msg::Time & stamp,
                        const BoxObservation & box)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = stamp;
    pose.pose.position.x = box.center_optical.x;
    pose.pose.position.y = box.center_optical.y;
    pose.pose.position.z = box.center_optical.z;
    pose.pose.orientation.w = 1.0;
    box_pose_pub_->publish(pose);

    geometry_msgs::msg::PoseStamped pelvis_pose;
    pelvis_pose.header.frame_id = "pelvis";
    pelvis_pose.header.stamp = stamp;
    const Vec3 pelvis = camera_optical_to_pelvis(box.center_optical);
    pelvis_pose.pose.position.x = pelvis.x;
    pelvis_pose.pose.position.y = pelvis.y;
    pelvis_pose.pose.position.z = pelvis.z;
    pelvis_pose.pose.orientation.w = 1.0;
    box_pose_pelvis_pub_->publish(pelvis_pose);
  }

  void detect_box(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    if (state_ != State::kWaitingForBox && state_ != State::kWaitingForStart) {
      return;
    }

    double closest_depth = std::numeric_limits<double>::infinity();
    std::size_t candidates = 0;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!in_workspace(*x, *y, *z)) {
          continue;
        }
        closest_depth = std::min(closest_depth, static_cast<double>(*z));
        ++candidates;
      }
      if (candidates < 100 || !std::isfinite(closest_depth)) {
        stable_frames_ = 0;
        return;
      }

      Vec3 min_point{std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
      Vec3 max_point{-std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
      std::size_t cluster_size = 0;
      x = sensor_msgs::PointCloud2ConstIterator<float>(cloud, "x");
      y = sensor_msgs::PointCloud2ConstIterator<float>(cloud, "y");
      z = sensor_msgs::PointCloud2ConstIterator<float>(cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!in_workspace(*x, *y, *z) || *z < closest_depth || *z > closest_depth + 0.13) {
          continue;
        }
        min_point.x = std::min(min_point.x, static_cast<double>(*x));
        min_point.y = std::min(min_point.y, static_cast<double>(*y));
        min_point.z = std::min(min_point.z, static_cast<double>(*z));
        max_point.x = std::max(max_point.x, static_cast<double>(*x));
        max_point.y = std::max(max_point.y, static_cast<double>(*y));
        max_point.z = std::max(max_point.z, static_cast<double>(*z));
        ++cluster_size;
      }
      if (cluster_size < 80) {
        stable_frames_ = 0;
        return;
      }

      const double observed_depth = max_point.z - min_point.z;
      const double center_depth = closest_depth + std::clamp(observed_depth * 0.5, 0.04, 0.08);
      last_box_ = BoxObservation{
        {(min_point.x + max_point.x) * 0.5, (min_point.y + max_point.y) * 0.5, center_depth},
        min_point, max_point, cluster_size};
      publish_box_pose(cloud.header.frame_id, cloud.header.stamp, last_box_);
      publish_markers();
      ++stable_frames_;
    } catch (const std::runtime_error & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Invalid depth point cloud: %s", error.what());
    }
  }

  bool in_workspace(const float x, const float y, const float z) const
  {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
           x > -0.14F && x < 0.18F && y > -0.16F && y < 0.20F && z > 0.40F && z < 0.75F;
  }

  bool build_plan_from_depth()
  {
    Vec3 box = camera_optical_to_pelvis(last_box_.center_optical);
    box.x = std::clamp(box.x, 0.24, 0.40);
    box.y = std::clamp(box.y, -0.16, 0.16);
    box.z = std::clamp(box.z, 0.04, 0.13);

    const double visible_width = last_box_.max_optical.x - last_box_.min_optical.x;
    const double side_clearance = std::clamp(visible_width * 0.5 + 0.055, 0.085, 0.13);
    const double pre_clearance = side_clearance + 0.075;
    const double pre_x = box.x - 0.055;

    const Vec3 left_pre{pre_x, box.y + pre_clearance, box.z + 0.035};
    const Vec3 right_pre{pre_x, box.y - pre_clearance, box.z + 0.035};
    const Vec3 left_grasp{box.x, box.y + side_clearance, box.z};
    const Vec3 right_grasp{box.x, box.y - side_clearance, box.z};
    const Vec3 left_lift{box.x, box.y + side_clearance, box.z + 0.18};
    const Vec3 right_lift{box.x, box.y - side_clearance, box.z + 0.18};

    plan_targets_ = {left_pre, right_pre, left_grasp, right_grasp, left_lift, right_lift};
    has_plan_targets_ = true;

    double left_error = 0.0;
    double right_error = 0.0;
    const Pose pre = solve_bimanual_pose(left_pre, right_pre, kDownPose, &left_error, &right_error);
    max_ik_error_ = std::max(left_error, right_error);
    const Pose grasp = solve_bimanual_pose(left_grasp, right_grasp, pre, &left_error, &right_error);
    max_ik_error_ = std::max(max_ik_error_, std::max(left_error, right_error));
    const Pose lift = solve_bimanual_pose(left_lift, right_lift, grasp, &left_error, &right_error);
    max_ik_error_ = std::max(max_ik_error_, std::max(left_error, right_error));

    if (max_ik_error_ > 0.11) {
      publish_status(
        "IK_FAILED: detected box is outside the current bimanual reach envelope");
      return false;
    }

    plan_ = {kDownPose, pre, grasp, lift};
    planned_box_pelvis_ = box;
    publish_markers();
    return true;
  }

  visualization_msgs::msg::Marker make_marker(
    const int id, const std::string & ns, const int type, const Vec3 & position,
    const Vec3 & scale, const float r, const float g, const float b, const float a)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "pelvis";
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = position.x;
    marker.pose.position.y = position.y;
    marker.pose.position.z = position.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale.x;
    marker.scale.y = scale.y;
    marker.scale.z = scale.z;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 0;
    return marker;
  }

  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray markers;
    if (last_box_.points > 0) {
      Vec3 box = camera_optical_to_pelvis(last_box_.center_optical);
      box.x = std::clamp(box.x, 0.24, 0.40);
      box.y = std::clamp(box.y, -0.16, 0.16);
      box.z = std::clamp(box.z, 0.04, 0.13);
      markers.markers.emplace_back(make_marker(
        1, "detected_box", visualization_msgs::msg::Marker::CUBE, box,
        {0.16, 0.16, 0.16}, 1.0F, 0.45F, 0.05F, 0.45F));
      markers.markers.emplace_back(make_marker(
        2, "detected_box_center", visualization_msgs::msg::Marker::SPHERE, box,
        {0.035, 0.035, 0.035}, 0.1F, 0.7F, 1.0F, 0.9F));
    }

    if (has_plan_targets_) {
      const std::array<std::string, 6> names{{
        "left_pre", "right_pre", "left_grasp", "right_grasp", "left_lift", "right_lift"}};
      for (std::size_t i = 0; i < plan_targets_.size(); ++i) {
        const bool left = (i % 2) == 0;
        markers.markers.emplace_back(make_marker(
          static_cast<int>(10 + i), names[i], visualization_msgs::msg::Marker::SPHERE,
          plan_targets_[i], {0.028, 0.028, 0.028},
          left ? 0.1F : 0.9F, left ? 0.9F : 0.1F, 0.25F, 0.95F));
      }

      auto line = make_marker(
        30, "palm_target_path", visualization_msgs::msg::Marker::LINE_LIST,
        {0.0, 0.0, 0.0}, {0.012, 0.0, 0.0}, 0.2F, 0.9F, 1.0F, 0.9F);
      for (std::size_t i = 0; i < 2; ++i) {
        geometry_msgs::msg::Point pre;
        pre.x = plan_targets_[i].x;
        pre.y = plan_targets_[i].y;
        pre.z = plan_targets_[i].z;
        geometry_msgs::msg::Point grasp;
        grasp.x = plan_targets_[i + 2].x;
        grasp.y = plan_targets_[i + 2].y;
        grasp.z = plan_targets_[i + 2].z;
        geometry_msgs::msg::Point lift;
        lift.x = plan_targets_[i + 4].x;
        lift.y = plan_targets_[i + 4].y;
        lift.z = plan_targets_[i + 4].z;
        line.points.emplace_back(pre);
        line.points.emplace_back(grasp);
        line.points.emplace_back(grasp);
        line.points.emplace_back(lift);
      }
      markers.markers.emplace_back(std::move(line));
    }

    marker_pub_->publish(markers);
  }

  void clear_markers()
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.emplace_back(marker);
    marker_pub_->publish(markers);
  }

  void handle_start(std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (state_ == State::kExecuting) {
      response->success = false;
      response->message = "pick trajectory is already executing";
      return;
    }
    if (state_ == State::kLifted) {
      response->success = false;
      response->message = "box is already lifted; call /g1/task/reset first";
      return;
    }
    attach_sent_ = false;
    state_ = State::kWaitingForBox;
    publish_status("WAITING_FOR_DEPTH_BOX: start accepted; looking for a stable box cluster");
    response->success = true;
    response->message = "started tabletop pick pipeline";
  }

  void handle_reset(std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    detach_pub_->publish(std_msgs::msg::Empty{});
    state_ = auto_start_ ? State::kWaitingForBox : State::kWaitingForStart;
    stable_frames_ = 0;
    attach_sent_ = false;
    has_plan_targets_ = false;
    max_ik_error_ = 0.0;
    plan_ = {kDownPose, kDownPose, kDownPose, kDownPose};
    publish_pose(kDownPose);
    clear_markers();
    publish_status(
      auto_start_ ? "WAITING_FOR_DEPTH_BOX: reset complete; both arms are down" :
      "WAITING_FOR_START: reset complete; both arms are down");
    response->success = true;
    response->message = "reset tabletop pick pipeline";
  }

  void publish_plan()
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = now();
    for (const auto joint : kJointNames) {
      trajectory.joint_names.emplace_back(joint);
    }
    const std::array<int, 4> seconds{{0, 2, 5, 8}};
    for (std::size_t i = 0; i < plan_.size(); ++i) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.time_from_start.sec = seconds[i];
      for (const auto position : plan_[i]) {
        point.positions.emplace_back(position);
      }
      trajectory.points.emplace_back(std::move(point));
    }
    trajectory_pub_->publish(trajectory);
  }

  void tick()
  {
    if (state_ == State::kWaitingForStart) {
      publish_pose(kDownPose);
      return;
    }

    if (state_ == State::kWaitingForBox) {
      publish_pose(kDownPose);
      if (stable_frames_ >= 8) {
        if (!build_plan_from_depth()) {
          state_ = State::kFailed;
          return;
        }
        publish_plan();
        state_ = State::kExecuting;
        execution_started_at_ = std::chrono::steady_clock::now();
        publish_status(
          "IK_PLANNED_FROM_DEPTH: box center in pelvis=(" +
          std::to_string(planned_box_pelvis_.x) + ", " + std::to_string(planned_box_pelvis_.y) +
          ", " + std::to_string(planned_box_pelvis_.z) + "), max IK error=" +
          std::to_string(max_ik_error_));
      }
      return;
    }

    if (state_ == State::kFailed) {
      publish_pose(kDownPose);
      return;
    }

    if (state_ == State::kLifted) {
      publish_pose(plan_[3]);
      return;
    }

    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - execution_started_at_).count();
    if (elapsed < 2.0) {
      publish_pose(interpolate(plan_[0], plan_[1], elapsed / 2.0));
    } else if (elapsed < 5.0) {
      publish_pose(interpolate(plan_[1], plan_[2], (elapsed - 2.0) / 3.0));
    } else if (elapsed < 8.0) {
      if (!attach_sent_) {
        attach_pub_->publish(std_msgs::msg::Empty{});
        attach_sent_ = true;
        publish_status("BIMANUAL_GRASP: virtual attachment created; lifting box");
      }
      publish_pose(interpolate(plan_[2], plan_[3], (elapsed - 5.0) / 3.0));
    } else {
      state_ = State::kLifted;
      publish_pose(plan_[3]);
      publish_status("BOX_LIFTED: bimanual IK trajectory complete");
    }
  }

  State state_{State::kWaitingForStart};
  bool auto_start_{true};
  std::size_t stable_frames_{0};
  bool attach_sent_{false};
  bool has_plan_targets_{false};
  double max_ik_error_{0.0};
  Vec3 planned_box_pelvis_{};
  BoxObservation last_box_{};
  std::array<Vec3, 6> plan_targets_{};
  std::array<Pose, 4> plan_{};
  std::chrono::steady_clock::time_point execution_started_at_{};
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>
    joint_publishers_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr box_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr box_pose_pelvis_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr detach_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TabletopPickDemo>());
  rclcpp::shutdown();
  return 0;
}
