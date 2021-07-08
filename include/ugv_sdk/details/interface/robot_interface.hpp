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

#include "ugv_sdk/details/interface/agilex_message.h"

namespace westonrobot {
enum class ProtocolType { AGX_V1, AGX_V2 };

class RobotInterface {
 public:
  ~RobotInterface() = default;

  // functions to be implemented by class AgilexBase
  virtual void EnableCommandedMode() = 0;
  virtual void DisableLightControl() = 0;

  // functions to be implemented by each robot class
  virtual void Connect(std::string can_name) = 0;
  // functions to be implemented by each robot class
  virtual void Connect(std::string uart_name, uint32_t baudrate){
      // use derived version
  };
  virtual void ResetRobotState() = 0;

 protected:
  /****** functions not available/valid to all robots ******/
  // functions to be implemented by class AgilexBase
  virtual void SetMotionMode(uint8_t mode){};

    // any specific robot will use a specialized version of the two functions
  virtual void SendMotionCommand(double linear_vel, double angular_vel,
                                 double lateral_velocity,
                                 double steering_angle){
      // use derived version
  };

  virtual void SendLightCommand(LightMode front_mode,
                                uint8_t front_custom_value, LightMode rear_mode,
                                uint8_t rear_custom_value){
      // use derived version
  };
};
}  // namespace westonrobot

#endif /* ROBOT_INTERFACE_HPP */
