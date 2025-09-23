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

#include <vector>

#include "leo_bldc/robot_controller.hpp"

namespace leo_bldc
{

class MecanumController : public RobotController {
public:
  using RobotController::RobotController;

  void setSpeed(float linear_x, float linear_y, float angular) override
  {
    if (this->params_.robot_input_timeout > 0) {
      this->last_command_time_remaining_ = this->params_.robot_input_timeout;
    }
    if (!this->enabled_) {this->enable();}

    const float wheel_geometry =
      (this->params_.robot_wheel_base + this->params_.robot_wheel_separation) / 2.0F;

    const float angular_multiplied =
      angular * wheel_geometry * this->params_.robot_angular_velocity_multiplier;
    const float sum_xy = linear_x + linear_y;
    const float diff_xy = linear_x - linear_y;

    const float wheel_FL_vel = (diff_xy - angular_multiplied) / this->params_.robot_wheel_radius;
    const float wheel_FR_vel = (sum_xy + angular_multiplied) / this->params_.robot_wheel_radius;
    const float wheel_RL_vel = (sum_xy - angular_multiplied) / this->params_.robot_wheel_radius;
    const float wheel_RR_vel = (diff_xy + angular_multiplied) / this->params_.robot_wheel_radius;

    this->wheel_FL.setTargetVelocity(wheel_FL_vel);
    this->wheel_RL.setTargetVelocity(wheel_RL_vel);
    this->wheel_FR.setTargetVelocity(wheel_FR_vel);
    this->wheel_RR.setTargetVelocity(wheel_RR_vel);
  }

  void update(uint32_t dt_ms, rclcpp::Time current_time) override
  {
    if (this->enabled_ && this->params_.robot_input_timeout > 0) {
      this->last_command_time_remaining_ -= dt_ms;
      if (this->last_command_time_remaining_ < 0) {this->disable();}
    }

    for (auto& wheel : wheels_) {
      wheel.update(current_time);
    }


    // velocity in radians per second
    const float FL_ang_vel = this->wheel_FL.getVelocity();
    const float RL_ang_vel = this->wheel_RL.getVelocity();
    const float FR_ang_vel = this->wheel_FR.getVelocity();
    const float RR_ang_vel = this->wheel_RR.getVelocity();

    const float dt_s = static_cast<float>(dt_ms) * 0.001F;

    const float wheel_geometry =
      (this->params_.robot_wheel_base + this->params_.robot_wheel_separation) / 2.0F;

    const float velocity_lin_x = (FL_ang_vel + FR_ang_vel + RL_ang_vel + RR_ang_vel) *
      this->params_.robot_wheel_radius / 4.0F;
    const float velocity_lin_y = (-FL_ang_vel + FR_ang_vel + RL_ang_vel - RR_ang_vel) *
      this->params_.robot_wheel_radius / 4.0F;
    const float z_rotation =
      (-FL_ang_vel + FR_ang_vel - RL_ang_vel + RR_ang_vel) * this->params_.robot_wheel_radius *
      this->params_.robot_angular_velocity_multiplier / (wheel_geometry * 4.0F);

    const float x_move = velocity_lin_x * std::cos(this->pose_yaw_) -
      velocity_lin_y * std::sin(this->pose_yaw_);
    const float y_move = velocity_lin_x * std::sin(this->pose_yaw_) +
      velocity_lin_y * std::cos(this->pose_yaw_);

    this->odom_.pose.pose.position.x += (x_move * dt_s);
    this->odom_.pose.pose.position.y += (y_move * dt_s);

    this->pose_yaw_ += (z_rotation * dt_s);

    if (this->pose_yaw_ > 2.0F * PI) {
      this->pose_yaw_ -= 2.0F * PI;
    } else if (this->pose_yaw_ < 0.0F) {
      this->pose_yaw_ += 2.0F * PI;
    }

    this->odom_.pose.pose.orientation.z = std::sin(this->pose_yaw_ * 0.5F);
    this->odom_.pose.pose.orientation.w = std::cos(this->pose_yaw_ * 0.5F);

    this->odom_.twist.twist.angular.z = z_rotation;
    this->odom_.twist.twist.linear.x = velocity_lin_x;
    this->odom_.twist.twist.linear.y = velocity_lin_y;
  }

private:
  static constexpr float PI = 3.141592653F;
};

} // namespace leo_bldc
