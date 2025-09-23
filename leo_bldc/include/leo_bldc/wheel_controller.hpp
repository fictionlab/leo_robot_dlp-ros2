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
#include "mab_types.hpp"
#include "MD.hpp"

#include "leo_bldc/circular_buffer.hpp"

namespace leo_bldc
{

enum class WheelOperationMode { VELOCITY_PROFILE, VELOCITY_PID };

enum class WheelID {FL, RL, FR, RR};
inline std::string wheelID_to_string(WheelID id)
{
  switch (id) {
    case WheelID::FL: return "FL";
    case WheelID::RL: return "RL";
    case WheelID::FR: return "FR";
    case WheelID::RR: return "RR";
  }
  return "Unknown";
}

inline std::ostream & operator<<(std::ostream & os, WheelID id)
{
  return os << wheelID_to_string(id);
}

struct WheelConfiguration
{
  // Id of the motor to connect to (belongs in [10, 2000]).
  mab::canId_t id;

  // Whether to reverse the direction of the wheel.
  bool reversed = false;
};

struct WheelParams
{
  // Max torque value to be output by the motor (in Nm).
  float max_torque;

  // P constant of the PID regulator.
  float wheel_pid_p;

  // I constant of the PID regulator.
  float wheel_pid_i;

  // D constant of the PID regulator.
  float wheel_pid_d;

  // Maximum limit for the integral component of PID regulator.
  float wheel_pid_int_max;

  // Target velocity of the profile movement (in s^-1).
  float profile_velocity;

  // The operation mode to use.
  WheelOperationMode op_mode = WheelOperationMode::VELOCITY_PID;
};

class WheelController {
public:
  WheelController(const WheelConfiguration & wheel_conf, mab::Candle * candle)
  : reversed_(wheel_conf.reversed)
  {
    motor_ = std::make_unique<mab::MD>(wheel_conf.id, candle);
    connected_ = motor_->init() == mab::MD::Error_t::OK;
    // Logger::g_m_verbosity = Logger::Verbosity_E::SILENT;
    // Logger::m_optionalLevel = Logger::LogLevel_E::WARN;
    // motor_->m_log.m_optionalLevel = Logger::LogLevel_E::WARN;

  }
  ~WheelController()
  {
    disable();
  }

  /**
   * Initialize the Wheel Controller.
   * Should be called after all ROS parameters are loaded.
   * Sets the motor parameters and initializes it.
   * @param params Parameter values to use.
   */
  void init(const WheelParams & params)
  {
    updateParams(params, true);
    motor_->zero();
    enable();
    setTargetVelocity(0);
  }


  /**
   * Enable the controller.
   * Enables sending PWM commands to the motor.
  */
  void enable()
  {
    if (!enabled_) {
      setMotionMode(params_);
      motor_->enable();
      enabled_ = true;
    }
  }

  /**
   * Disable the controller.
   * Disables sending PWM commands to the motor.
  */
  void disable()
  {
    if (enabled_) {
      motor_->disable();
      enabled_ = false;
    }
  }

  /**
   * Check if Wheel Controller connected to its motor.
   */
  bool isConnected() const {return connected_;}

  /**
   * Get the current position of the motor (in rad).
   */
  double getPosition() const
  {
    std::pair<float, mab::MD::Error_t> position = motor_->getPosition();
    if (position.second == mab::MD::Error_t::OK) {
      return reversed_ ? -position.first : position.first;
    }
    return -1.0;
  }

  /**
   * Get the current velocity of the motor (in rad/s).
   */
  double getVelocity() const
  {
    if (position_buffer_.filled_) {
      std::pair<double, rclcpp::Time> recent_positon = position_buffer_.get_recent();
      std::pair<double, rclcpp::Time> oldest_postion = position_buffer_.get_oldest();

      double distance = recent_positon.first - oldest_postion.first;
      double time = (recent_positon.second - oldest_postion.second).seconds();
      return distance / time;
    }
    
    std::pair<float, mab::MD::Error_t> velocity = motor_->getVelocity();
    if (velocity.second == mab::MD::Error_t::OK) {
      return reversed_ ? -velocity.first : velocity.first;
    }
    return -1.0;
  }

  /**
   * Get the current torque of the motor (in Nm).
   */
  double getTorque() const
  {
    std::pair<float, mab::MD::Error_t> torque = motor_->getTorque();
    return torque.second == mab::MD::Error_t::OK ? torque.first : -1.0;
  }

  /**
   * Set target velocity of the motor
   * @param velocity target velocity (in rad/s).
   */
  void setTargetVelocity(float velocity)
  {
    if (reversed_) {
      velocity *= -1;
    }
    motor_->setTargetVelocity(velocity);
  }

  /**
   * Update parameters of the Wheel Controller and its motor
   * @param params Parameter values to use.
   * @param init Flag specifying if this function is called during
   * initialization of Wheel Controller.
   */
  void updateParams(const WheelParams & params, bool init = false)
  {
    if (params_.max_torque != params.max_torque || init) {
      motor_->setMaxTorque(params.max_torque);
    }

    if (params_.op_mode != params.op_mode) {
      setMotionMode(params);
    }

    if (params_.profile_velocity != params.profile_velocity || init) {
      motor_->setProfileVelocity(params.profile_velocity);
    }

    updatePid(params.wheel_pid_p, params.wheel_pid_i, params.wheel_pid_d, params.wheel_pid_int_max);
    params_ = params;
  }

  /**
   * Perform an update routine.
   * @param timestamp Time at which call to update function happened
   */
  void update(rclcpp::Time timestamp) {
    double position = getPosition();
    position_buffer_.push_back({position, timestamp});
  }

private:
  /**
   * Update PID constants and maximum limit for I component
   * @param p P constant of PID regulator
   * @param i I constant of PID regulator
   * @param d D constant of PID regulator
   * @param int_max Maximum output of the integral component of PID regulator (in [rad/s]).
   */
  void updatePid(float p, float i, float d, float int_max)
  {
    if (p != params_.wheel_pid_p ||
      i != params_.wheel_pid_i ||
      d != params_.wheel_pid_d ||
      int_max != params_.wheel_pid_int_max)
    {
      motor_->setVelocityPIDparam(p, i, d, int_max);
    }
  }
  /**
   * Set motion mode of the motor and responding properties from parameters.
   * The motion mode to be set is also determined from parameters.
   * @param params Parameters with the properties to be set.
   */
  void setMotionMode(const WheelParams & params)
  {
    switch (params.op_mode) {
      case WheelOperationMode::VELOCITY_PID:
        motor_->setMotionMode(mab::MdMode_E::VELOCITY_PID);
        motor_->setVelocityPIDparam(params.wheel_pid_p, params.wheel_pid_i, params.wheel_pid_d,
            params.wheel_pid_int_max);
        break;
      case WheelOperationMode::VELOCITY_PROFILE:
        motor_->setMotionMode(mab::MdMode_E::VELOCITY_PROFILE);
        motor_->setProfileVelocity(params.profile_velocity);
        break;
      default:
        break;
    }
  }

  std::unique_ptr<mab::MD> motor_;
  WheelParams params_;
  CircularBuffer<std::pair<double, rclcpp::Time>, 100> position_buffer_;
  bool enabled_{};
  bool connected_{};
  bool reversed_{};
};

} // namespace leo_bldc
