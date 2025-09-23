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

#include "bldc_manager.hpp"

#include <chrono>
#include <cmath>

#include "leo_bldc/diff_drive_controller.hpp"
#include "leo_bldc/mecanum_controller.hpp"
#include "leo_bldc/wheel_controller.hpp"

#include "configuration.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;


namespace leo_bldc
{
double clamp(double val, double min, double max)
{
  return std::min(std::max(val, min), max);
}

BLDCManager::BLDCManager(rclcpp::NodeOptions options)
: Node("bldc_manager", options),
  param_listener_(get_node_parameters_interface())
{
  params_ = param_listener_.get_params();
  init_controller();

  reset_odom_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/reset_odometry",
    std::bind(&BLDCManager::reset_odom_callback, this, _1, _2));

  wheel_odom_pub_ =
    create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
  joint_states_pub_ =
    create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(5).best_effort(),
    std::bind(&BLDCManager::cmd_vel_callback, this, _1));

  std::array<std::string, 4> wheel_cmd_vel_topics = {
    "~/wheel_RL/cmd_velocity",
    "~/wheel_RR/cmd_velocity",
    "~/wheel_FL/cmd_velocity",
    "~/wheel_FR/cmd_velocity"
  };
  wheel_ids_ = {WheelID::RL, WheelID::RR, WheelID::FL, WheelID::FR};

  for (int i = 0; i < 4; ++i) {
    wheel_cmd_vel_subs_[i] = create_subscription<std_msgs::msg::Float32>(
        wheel_cmd_vel_topics[i],
        rclcpp::QoS(5),
      [this, i](std_msgs::msg::Float32::ConstSharedPtr msg) {
        wheel_cmd_vel_callback(msg.get(), (void *)&wheel_ids_[i]);
        }
    );
  }

  update_timer_ = create_wall_timer(
    10ms, std::bind(&BLDCManager::update, this));

  loop_timer_ = create_wall_timer(
    1s/30.0, std::bind(&BLDCManager::publish_msgs, this));
  RCLCPP_INFO(get_logger(), "Started node");
}

void BLDCManager::init_controller()
{
  if (params_.mecanum_wheels) {
    controller_ = new MecanumController(ROBOT_CONFIG);
  } else {
    controller_ = new DiffDriveController(ROBOT_CONFIG);
  }
  mecanum_wheels_ = params_.mecanum_wheels;

  RobotParams rp = parse_parameters();
  controller_->init(rp);
  controller_initialized_ = true;

  std::array<bool, 4> connections = controller_->getwheelsConnections();
  for (int i = 0; i < 4; i++) {
    if (!connections[i]) {
      controller_initialized_ = false;
      RCLCPP_ERROR_STREAM(get_logger(), "Wheel " << wheel_ids_[i] << " not connected!");
    }
  }
  if (controller_initialized_) {
    RCLCPP_INFO(get_logger(), "All wheels connected. Controller initialized.");
  }
}

void BLDCManager::fini_controller()
{
  controller_initialized_ = false;
  controller_->~RobotController();
  delete controller_;
}

void BLDCManager::check_dynamic_parameters()
{
  if (param_listener_.is_old(params_)) {
    param_listener_.refresh_dynamic_parameters();
    params_ = param_listener_.get_params();
  }
}

RobotParams BLDCManager::parse_parameters()
{
  RobotParams rp;
  // wheel controller params
  rp.max_torque = params_.max_torque;
  rp.wheel_pid_p = params_.pid_constants[0];
  rp.wheel_pid_i = params_.pid_constants[1];
  rp.wheel_pid_d = params_.pid_constants[2];
  rp.wheel_pid_int_max = params_.pid_integral_max;
  rp.profile_velocity = params_.profile_velocity;
  rp.op_mode =
    params_.velocity_profile ? WheelOperationMode::VELOCITY_PROFILE :
    WheelOperationMode::VELOCITY_PID;

  // robot controller params
  rp.robot_wheel_radius = params_.wheel_radius;
  rp.robot_wheel_separation = params_.wheel_separation;
  rp.robot_wheel_base = params_.wheel_base;
  rp.robot_angular_velocity_multiplier = params_.angular_velocity_multiplier;
  rp.robot_input_timeout = params_.input_timeout;

  return rp;
}

void BLDCManager::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (controller_initialized_) {
    double lin_x = clamp(msg->linear.x, -params_.max_linear_velocity, params_.max_linear_velocity);
    double lin_y = clamp(msg->linear.y, -params_.max_linear_velocity, params_.max_linear_velocity);
    double ang_z = clamp(msg->angular.z, -params_.max_angular_velocity,
        params_.max_angular_velocity);
    controller_->setSpeed(lin_x, lin_y, ang_z);
  }
}

void BLDCManager::wheel_cmd_vel_callback(const void * msgin, void *context)
{
  const std_msgs::msg::Float32 * msg = static_cast<const std_msgs::msg::Float32 *>(msgin);
  auto wheel_id = *static_cast<const WheelID *>(context);

  WheelController * wheel = nullptr;

  if (controller_initialized_) {
    switch (wheel_id) {
      case WheelID::FL:
        wheel = &controller_->wheel_FL;
        break;
      case WheelID::FR:
        wheel = &controller_->wheel_FR;
        break;
      case WheelID::RL:
        wheel = &controller_->wheel_RL;
        break;
      case WheelID::RR:
        wheel = &controller_->wheel_RR;
        break;
      default:
        break;
    }

    if (wheel) {
      wheel->enable();
      wheel->setTargetVelocity(msg->data);
    }
  }
}

void BLDCManager::reset_odom_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  if (controller_initialized_) {
    controller_->resetOdom();

    res->success = true;
    res->message = "Odometry reset successful.";
  } else {
    res->success = false;
    res->message = "Controller not initialized.";
  }
}

void BLDCManager::update()
{
  rclcpp::Time current_time = get_clock()->now();
  if (!last_update_time_.has_value()) {
    last_update_time_ = current_time;
    return;
  }

  check_dynamic_parameters();
  if (!controller_initialized_) {
    return;
  }

  if (mecanum_wheels_ != params_.mecanum_wheels) {
    fini_controller();
    init_controller();
  } else {
    controller_->updateParams(parse_parameters());
  }

  uint32_t dt = static_cast<uint32_t>((current_time - *last_update_time_).seconds() * 1000.0);

  controller_->update(dt, current_time);


  last_update_time_ = current_time;
}

void BLDCManager::publish_msgs()
{
  if (controller_initialized_) {
    nav_msgs::msg::Odometry odom_msg = controller_->getOdom();
    sensor_msgs::msg::JointState joint_states_msg = controller_->getJointState();

    rclcpp::Time stamp = get_clock()->now();
    odom_msg.header.stamp = stamp;
    joint_states_msg.header.stamp = stamp;

    joint_states_msg.name = params_.wheel_joint_names;

    odom_msg.header.frame_id = params_.tf_frame_prefix + params_.odom_frame_id;
    odom_msg.child_frame_id = params_.tf_frame_prefix + params_.robot_frame_id;

    std::vector<double> covariance_matrix;
    if (mecanum_wheels_) {
      covariance_matrix = params_.wheel_odom_mecanum_twist_covariance_diagonal;
    } else {
      covariance_matrix = params_.wheel_odom_twist_covariance_diagonal;
    }
    for (int i = 0; i < 6; i++) {
      odom_msg.twist.covariance[i * 7] =
        covariance_matrix[i];
    }

    wheel_odom_pub_->publish(odom_msg);
    joint_states_pub_->publish(joint_states_msg);
  }
}

} // namespace leo_bldc

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(leo_bldc::BLDCManager)
