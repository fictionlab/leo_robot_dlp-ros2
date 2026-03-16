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

// Number of wheels the robot is using
constexpr uint8_t NUMBER_OF_WHEELS = 4;

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

  // The time (in milliseconds) the robot must remain still before motor
  // unnecessary efforts are reset.
  int effort_reset_timeout;
};

class RobotController {
public:
  RobotController(const RobotConfiguration & robot_conf, rclcpp::Logger logger)
  : candle_(mab::attachCandle(mab::CANdleDatarate_E::CAN_DATARATE_1M,
      mab::candleTypes::busTypes_t::USB)),
    wheels_{{WheelController(robot_conf.wheel_RL_conf, candle_, WheelID::RL),
        WheelController(robot_conf.wheel_RR_conf, candle_, WheelID::RR),
        WheelController(robot_conf.wheel_FL_conf, candle_, WheelID::FL),
        WheelController(robot_conf.wheel_FR_conf, candle_, WheelID::FR)}},
    wheel_RL(wheels_[0]),
    wheel_RR(wheels_[1]),
    wheel_FL(wheels_[2]),
    wheel_FR(wheels_[3]),
    logger_(logger)
  {
    if (!candle_) {
      throw std::runtime_error("Failed to attach CANdle!");
    }
  }
  ~RobotController()
  {
    for(auto & wheel : wheels_) {
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
    for(auto & wheel : wheels_) {
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
    for(auto & wheel : wheels_) {
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
    js.velocity.resize(NUMBER_OF_WHEELS);
    js.position.resize(NUMBER_OF_WHEELS);
    js.effort.resize(NUMBER_OF_WHEELS);

    for (int i = 0; i < NUMBER_OF_WHEELS; i++) {
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
    for(auto & wheel : wheels_) {
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
    for(auto & wheel : wheels_) {
      wheel.disable();
    }
    enabled_ = false;
  }

  /**
   * Clear errors present in all wheel motor drivers.
   */
  void clearErrors()
  {
    for(auto & wheel : wheels_) {
      wheel.clearErrors();
    }
  }

  /**
   * Get statuses of all wheel motor connections.
   */
  bool wheelsConnected() const
  {
    bool connected = true;

    for (auto & wheel : wheels_) {
      if (!wheel.isConnected()) {
        RCLCPP_ERROR_STREAM(logger_, "Wheel " << wheel.getID() << " not connected!");
        connected = false;
      }
    }

    return connected;
  }

private:
  mab::Candle * candle_;

protected:
  /**
   * Reset unnecessary efforts on the motors.
   * Checs if the motors hold effort from previous movement
   * and resets them if those are unnecessary.
   */
  void resetEffort()
  {
    std::vector<double> efforts;
    std::vector<int> signs;

    for(auto & wheel : wheels_) {
      double effort = wheel.getTorque();
      efforts.push_back(effort);
      signs.push_back(effort > 0 ? 1 : -1);
    }

    int sum = 0;
    for(int i = 0; i < NUMBER_OF_WHEELS; i++) {
      sum += signs[i];
    }

    bool has_effort = std::all_of(efforts.begin(), efforts.end(), [](double e){
          return std::abs(e) > 0.01;
                                                                                                         });

    if (std::abs(sum) != NUMBER_OF_WHEELS && has_effort) {
      for(auto & wheel : wheels_) {
        wheel.resetEffort();
      }
    }
  }

  /**
   * Check if robot is not moving.
   * Checks if all motors are stopped.
   */
  bool robotNotMoving()
  {
    for(auto & wheel : wheels_) {
      if (wheel.isMoving()) {
        return false;
      }
    }

    return true;
  }


  nav_msgs::msg::Odometry odom_{};
  double pose_yaw_;
  bool enabled_ = false;
  bool target_vel_zero_ = true;
  int last_command_time_remaining_;
  int motors_loosen_time_remaining_;
  RobotParams params_;
  rclcpp::Logger logger_;

  // Wheel controllers
  std::array<WheelController, 4> wheels_;

public:
  WheelController & wheel_FL;
  WheelController & wheel_RL;
  WheelController & wheel_FR;
  WheelController & wheel_RR;
};

}
