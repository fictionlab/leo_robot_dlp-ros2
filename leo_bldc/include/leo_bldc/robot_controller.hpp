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

#include <array>

#include "leo_bldc/wheel_controller.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/time.hpp"

namespace leo_bldc
{

struct RobotConfiguration
{
  WheelConfiguration wheel_FL_conf;
  WheelConfiguration wheel_RL_conf;
  WheelConfiguration wheel_FR_conf;
  WheelConfiguration wheel_RR_conf;
};

struct RobotParams : WheelParams
{
  // The radius of the wheel in meters.
  float robot_wheel_radius;

  // The distance (in meters) between the centers of the left and right wheels.
  float robot_wheel_separation;

  // The distance (in meters) between the centers of the rear and front wheels.
  float robot_wheel_base;

  // The angular velocity in setSpeed command is multiplied by this parameter
  // and the calculated odometry has its angular velocity divided by this
  // parameter.
  float robot_angular_velocity_multiplier;

  // The timeout (in milliseconds) for the setSpeed commands. The controller
  // will be disabled if it does not receive a command within the specified
  // time. If set to 0, the timeout is disabled.
  int robot_input_timeout;
};

class RobotController {
public:
  RobotController(const RobotConfiguration & robot_conf)
  : candle_(mab::attachCandle(mab::CANdleDatarate_E::CAN_DATARATE_1M,
      mab::candleTypes::busTypes_t::USB)),
    wheels_{{WheelController(robot_conf.wheel_RL_conf, candle_),
        WheelController(robot_conf.wheel_RR_conf, candle_),
        WheelController(robot_conf.wheel_FL_conf, candle_),
        WheelController(robot_conf.wheel_FR_conf, candle_)}},
    wheel_RL(wheels_[0]),
    wheel_RR(wheels_[1]),
    wheel_FL(wheels_[2]),
    wheel_FR(wheels_[3])
  {
    if (!candle_) {
      throw std::runtime_error("Failed to attach CANdle!");
    }
  }
  ~RobotController()
  {
    for(auto &wheel : wheels_) {
      wheel.~WheelController();
    }
    mab::detachCandle(candle_);
    candle_ = nullptr;
  }
  /**
   * Initialize the Robot Controller and all Wheel Controllers.
   * @param params Parameter values to use.
   */
  void init(const RobotParams & params)
  {
    for(auto &wheel : wheels_) {
      wheel.init(params);
    }
    params_ = params;
  }

  /**
   * Update parameters of Robot Controller and all Wheel Controllers
   * @param params Parameter values to use.
   */
  void updateParams(const RobotParams & params)
  {
    for(auto &wheel : wheels_) {
      wheel.updateParams(params);
    }
    params_ = params;
  }

  /**
   * Set the target speed of the robot.
   * Automatically enables the controller.
   * @param linear_x The linear speed of the robot in x axis in m/s
   * @param linear_y The linear speed of the robot in y axis in m/s
   * @param angular The angular speed of the robot in rad/s
   */
  virtual void setSpeed(float linear_x, float linear_y, float angular) = 0;

  /**
   * Get the current odometry.
   */
  // RobotOdom getOdom() { return odom_; }
  nav_msgs::msg::Odometry getOdom() const {return odom_;}

  /**
   * Get the current wheel states.
   */
  sensor_msgs::msg::JointState getJointState() const
  {
    sensor_msgs::msg::JointState js;
    js.velocity.resize(4);
    js.position.resize(4);
    js.effort.resize(4);

    for (int i = 0; i < 4; i++) {
      js.velocity[i] = wheels_[i].getVelocity();
      js.position[i] = wheels_[i].getPosition();
      js.effort[i] = wheels_[i].getTorque();
    }

    return js;
  }

  /**
   * Reset the odometry position.
   */
  void resetOdom()
  {
    odom_.pose.pose.position.x = 0.0;
    odom_.pose.pose.position.y = 0.0;
    pose_yaw_ = 0;
  }

  /**
   * Perform an update routine.
   * @param dt_ms Time elapsed since the last call to update function.
   */
  virtual void update(uint32_t dt_ms, rclcpp::Time current_time) = 0;

  /**
   * Enable the controller.
   * Enabling the controller enables all wheel controllers.
   */
  void enable()
  {
    for(auto &wheel : wheels_) {
      wheel.enable();
    }
    enabled_ = true;
  }

  /**
   * Disable the controller.
   * Disabling the controller disables all wheel controllers.
   */
  void disable()
  {
    for(auto &wheel : wheels_) {
      wheel.disable();
    }
    enabled_ = false;
  }

  /**
   * Get statuses of all wheel motor connections.
   */
  std::array<bool, 4> getwheelsConnections() const
  {
    return {{
      wheels_[0].isConnected(),
      wheels_[1].isConnected(),
      wheels_[2].isConnected(),
      wheels_[3].isConnected()
    }};

  }

private:
  mab::Candle * candle_;

protected:
  nav_msgs::msg::Odometry odom_{};
  double pose_yaw_;
  bool enabled_ = false;
  int last_command_time_remaining_;
  RobotParams params_;

  // Wheel controllers
  std::array<WheelController, 4> wheels_;

public:
  WheelController & wheel_FL;
  WheelController & wheel_RL;
  WheelController & wheel_FR;
  WheelController & wheel_RR;
};

}
