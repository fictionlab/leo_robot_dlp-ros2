// Copyright 2025 Fictionlab sp. z o.o.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once

#include <optional>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "MD.hpp"
#include "mab_types.hpp"

#include "leo_bldc/robot_controller.hpp"
#include "leo_bldc/bldc_manager_parameters.hpp"

namespace leo_bldc
{
class BLDCManager : public rclcpp::Node
{
public:
  explicit BLDCManager(rclcpp::NodeOptions options);

private:
  void init_controller();
  void fini_controller();
  void check_dynamic_parameters();
  RobotParams parse_parameters();
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void wheel_cmd_vel_callback(const void * msgin, void * context);
  void reset_odom_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
  void clear_errors_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
  void update();
  void publish_msgs();

  // ROS entities
  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::array<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr, 4> wheel_cmd_vel_subs_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_odom_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_motor_errors_srv_;

  // Timers
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr loop_timer_;

  // Parameters
  bldc_manager::ParamListener param_listener_;
  bldc_manager::Params params_;

  // Time
  std::optional<rclcpp::Time> last_update_time_ {std::nullopt};

  // Flags and variables
  RobotController * controller_;
  bool controller_initialized_{};
  bool mecanum_wheels_{};
  std::array<WheelID, 4> wheel_ids_;
};

} // namespace leo_bldc
