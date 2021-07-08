/*
 * robot_interface.hpp
 *
 * Created on: Jul 08, 2021 11:48
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef ROBOT_INTERFACE_HPP
#define ROBOT_INTERFACE_HPP

#include <string>

#include "ugv_sdk/interface/agilex_message.h"

namespace westonrobot {
struct RobotInterface {
  // functions to be implemented by class AgilexBase
  virtual void EnableCommandedMode() = 0;

  virtual void SetMotionMode(uint8_t mode){};

  virtual void SendMotionCommand(double linear_vel, double angular_vel,
                                 double lateral_velocity,
                                 double steering_angle) = 0;
  virtual void SendLightCommand(LightMode front_mode,
                                uint8_t front_custom_value, LightMode rear_mode,
                                uint8_t rear_custom_value) = 0;
  virtual void DisableLightControl() = 0;

  // functions to be implemented by each robot class
  virtual void Connect(std::string can_name) = 0;
  virtual void Connect(std::string uart_name, uint32_t baudrate) = 0;

  virtual void ResetRobotState() = 0;
};
}  // namespace westonrobot

#endif /* ROBOT_INTERFACE_HPP */
