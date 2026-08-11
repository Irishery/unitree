#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "unitree_hg/msg/hand_cmd.hpp"
#include "unitree_hg/msg/hand_state.hpp"

using namespace std::chrono_literals;

namespace {
constexpr std::size_t kMotorCount = 7;
constexpr uint8_t kServoStatus = 0x01;

struct Hand {
  std::string side;
  std::array<double, kMotorCount> minimum{};
  std::array<double, kMotorCount> maximum{};
  std::array<double, kMotorCount> target{};
  std::array<double, kMotorCount> commanded{};
  std::array<double, kMotorCount> measured{};
  bool state_received{false};
  bool command_received{false};
  rclcpp::Time state_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Time command_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<unitree_hg::msg::HandCmd>::SharedPtr native_command;
  rclcpp::Subscription<unitree_hg::msg::HandState>::SharedPtr native_state;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ros_command;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ros_state;
};

uint8_t motor_mode(std::size_t motor, bool timeout) {
  return static_cast<uint8_t>((motor & 0x0fU) | (kServoStatus << 4U) |
                              (timeout ? 0x80U : 0x00U));
}
}  // namespace

class Dex3Bridge final : public rclcpp::Node {
 public:
  Dex3Bridge() : Node("dex3_bridge") {
    enabled_ = declare_parameter("start_control_enabled", false);
    require_both_hands_ = declare_parameter("require_both_hands", true);
    state_timeout_s_ = declare_parameter("state_timeout_s", 0.5);
    command_timeout_s_ = declare_parameter("command_timeout_s", 0.35);
    publish_rate_hz_ = declare_parameter("publish_rate_hz", 50.0);
    max_velocity_ = declare_parameter("max_velocity_rad_s", 0.5);
    kp_ = declare_parameter("kp", 0.5);
    kd_ = declare_parameter("kd", 0.1);

    left_.side = "left";
    left_.minimum = {-1.05, -0.724, 0.0, -1.57, -1.75, -1.57, -1.75};
    left_.maximum = {1.05, 1.05, 1.75, 0.0, 0.0, 0.0, 0.0};
    right_.side = "right";
    right_.minimum = {-1.05, -1.05, -1.75, 0.0, 0.0, 0.0, 0.0};
    right_.maximum = {1.05, 0.742, 0.0, 1.57, 1.75, 1.57, 1.75};
    setup_hand(left_);
    setup_hand(right_);

    enabled_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/g1/dex3/control_enabled", rclcpp::QoS(1).transient_local());
    enable_service_ = create_service<std_srvs::srv::SetBool>(
        "/g1/dex3/enable_control",
        std::bind(&Dex3Bridge::enable, this, std::placeholders::_1,
                  std::placeholders::_2));
    stop_service_ = create_service<std_srvs::srv::Trigger>(
        "/g1/dex3/stop",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          disarm("stop service called");
          response->success = true;
          response->message = "DEX3 control disabled and timeout command sent";
        });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&Dex3Bridge::tick, this));
    publish_enabled();
    RCLCPP_WARN(get_logger(),
                "Real DEX3-1 bridge ready but DISARMED; inspect both hand states "
                "before calling /g1/dex3/enable_control");
  }

  ~Dex3Bridge() override {
    if (rclcpp::ok()) {
      publish_stop(left_);
      publish_stop(right_);
    }
  }

 private:
  void setup_hand(Hand &hand) {
    const std::string native_base = "/dex3/" + hand.side;
    const std::string ros_base = "/g1/dex3/" + hand.side;
    hand.native_command = create_publisher<unitree_hg::msg::HandCmd>(
        native_base + "/cmd", 10);
    hand.native_state = create_subscription<unitree_hg::msg::HandState>(
        "/lf" + native_base + "/state", rclcpp::QoS(1).best_effort(),
        [this, &hand](unitree_hg::msg::HandState::ConstSharedPtr message) {
          on_state(hand, *message);
        });
    hand.ros_command = create_subscription<sensor_msgs::msg::JointState>(
        ros_base + "/command", 10,
        [this, &hand](sensor_msgs::msg::JointState::ConstSharedPtr message) {
          on_command(hand, *message);
        });
    hand.ros_state =
        create_publisher<sensor_msgs::msg::JointState>(ros_base + "/joint_states", 10);
  }

  void on_state(Hand &hand, const unitree_hg::msg::HandState &message) {
    if (message.motor_state.size() < kMotorCount) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "%s DEX3 state has %zu motors, expected 7",
                            hand.side.c_str(), message.motor_state.size());
      return;
    }
    sensor_msgs::msg::JointState output;
    output.header.stamp = now();
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      output.name.push_back(hand.side + "_dex3_motor_" + std::to_string(i));
      output.position.push_back(message.motor_state[i].q);
      output.velocity.push_back(message.motor_state[i].dq);
      output.effort.push_back(message.motor_state[i].tau_est);
      hand.measured[i] = message.motor_state[i].q;
    }
    hand.state_received = true;
    hand.state_stamp = now();
    hand.ros_state->publish(output);
  }

  void on_command(Hand &hand, const sensor_msgs::msg::JointState &message) {
    if (!enabled_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Ignoring %s DEX3 command while disarmed",
                           hand.side.c_str());
      return;
    }
    if (message.position.size() != kMotorCount) {
      RCLCPP_ERROR(get_logger(), "%s DEX3 command must contain 7 positions",
                   hand.side.c_str());
      return;
    }
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      if (!std::isfinite(message.position[i])) {
        RCLCPP_ERROR(get_logger(), "Rejected non-finite %s DEX3 command",
                     hand.side.c_str());
        return;
      }
      hand.target[i] = std::clamp(message.position[i], hand.minimum[i], hand.maximum[i]);
    }
    hand.command_received = true;
    hand.command_stamp = now();
  }

  bool state_is_fresh(const Hand &hand) const {
    return hand.state_received && (now() - hand.state_stamp).seconds() <= state_timeout_s_;
  }

  void enable(const std_srvs::srv::SetBool::Request::SharedPtr request,
              std_srvs::srv::SetBool::Response::SharedPtr response) {
    if (!request->data) {
      disarm("control disabled by service");
      response->success = true;
      response->message = "DEX3 control disabled";
      return;
    }
    const bool ready = require_both_hands_
                           ? state_is_fresh(left_) && state_is_fresh(right_)
                           : state_is_fresh(left_) || state_is_fresh(right_);
    if (!ready) {
      response->success = false;
      response->message = "fresh DEX3 state not available";
      return;
    }
    enabled_ = true;
    left_.commanded = left_.measured;
    left_.target = left_.measured;
    right_.commanded = right_.measured;
    right_.target = right_.measured;
    left_.command_received = false;
    right_.command_received = false;
    publish_enabled();
    response->success = true;
    response->message = "DEX3 armed; waiting for fresh 7-position commands";
  }

  void tick() {
    if (!enabled_) return;
    if ((require_both_hands_ && (!state_is_fresh(left_) || !state_is_fresh(right_))) ||
        (!require_both_hands_ && !state_is_fresh(left_) && !state_is_fresh(right_))) {
      disarm("DEX3 state watchdog expired");
      return;
    }
    command_hand(left_);
    command_hand(right_);
  }

  void command_hand(Hand &hand) {
    if (!state_is_fresh(hand)) return;
    if (!hand.command_received ||
        (now() - hand.command_stamp).seconds() > command_timeout_s_) {
      publish_stop(hand);
      hand.command_received = false;
      return;
    }
    unitree_hg::msg::HandCmd output;
    output.motor_cmd.resize(kMotorCount);
    const double max_step = max_velocity_ / publish_rate_hz_;
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      const double delta = std::clamp(hand.target[i] - hand.commanded[i], -max_step, max_step);
      hand.commanded[i] += delta;
      output.motor_cmd[i].mode = motor_mode(i, false);
      output.motor_cmd[i].q = static_cast<float>(hand.commanded[i]);
      output.motor_cmd[i].dq = 0.0F;
      output.motor_cmd[i].tau = 0.0F;
      output.motor_cmd[i].kp = static_cast<float>(kp_);
      output.motor_cmd[i].kd = static_cast<float>(kd_);
    }
    hand.native_command->publish(output);
  }

  void publish_stop(Hand &hand) {
    unitree_hg::msg::HandCmd output;
    output.motor_cmd.resize(kMotorCount);
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      output.motor_cmd[i].mode = motor_mode(i, true);
      output.motor_cmd[i].q = 0.0F;
      output.motor_cmd[i].dq = 0.0F;
      output.motor_cmd[i].tau = 0.0F;
      output.motor_cmd[i].kp = 0.0F;
      output.motor_cmd[i].kd = 0.0F;
    }
    hand.native_command->publish(output);
  }

  void disarm(const std::string &reason) {
    enabled_ = false;
    left_.command_received = false;
    right_.command_received = false;
    publish_stop(left_);
    publish_stop(right_);
    publish_enabled();
    RCLCPP_WARN(get_logger(), "%s", reason.c_str());
  }

  void publish_enabled() {
    std_msgs::msg::Bool message;
    message.data = enabled_;
    enabled_pub_->publish(message);
  }

  Hand left_;
  Hand right_;
  bool enabled_{false};
  bool require_both_hands_{true};
  double state_timeout_s_{0.5};
  double command_timeout_s_{0.35};
  double publish_rate_hz_{50.0};
  double max_velocity_{0.5};
  double kp_{0.5};
  double kd_{0.1};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Dex3Bridge>());
  rclcpp::shutdown();
  return 0;
}
