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
#include <chrono>

#include "ugv_sdk/details/interface/agilex_message.h"
#include "ugv_sdk/details/interface/parser_interface.hpp"

#define AGX_MAX_ACTUATOR_NUM 8

namespace westonrobot {
using AgxMsgRefClock = std::chrono::steady_clock;
using AgxMsgTimeStamp = std::chrono::time_point<AgxMsgRefClock>;

struct CoreStateMsgGroup {
  AgxMsgTimeStamp time_stamp;

  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  LightStateMessage light_state;
  MotionModeStateMessage motion_mode_state;
  RcStateMessage rc_state;
};

struct ActuatorStateMsgGroup {
  AgxMsgTimeStamp time_stamp;

  ActuatorHSStateMessage actuator_hs_state[AGX_MAX_ACTUATOR_NUM];  // v2 only
  ActuatorLSStateMessage actuator_ls_state[AGX_MAX_ACTUATOR_NUM];  // v2 only
  ActuatorStateMessageV1 actuator_state[AGX_MAX_ACTUATOR_NUM];     // v1 only
};

struct CommonSensorStateMsgGroup {};

class RobotCommonInterface {
 public:
  ~RobotCommonInterface() = default;

  // functions to be implemented by class AgilexBase
  virtual void EnableCommandedMode() = 0;

  // functions to be implemented by each robot class
  virtual void Connect(std::string can_name) = 0;

  virtual void ResetRobotState() = 0;

  virtual void DisableLightControl() {
    // do nothing if no light on robot
  }

  virtual ProtocolVersion GetParserProtocolVersion() = 0;

 protected:
  /****** functions not available/valid to all robots ******/
  // functions to be implemented by class AgilexBase
  virtual void SetMotionMode(uint8_t mode){};
  virtual void SetBrakedMode(BrakeMode mode){};

  virtual CoreStateMsgGroup GetRobotCoreStateMsgGroup() {
    throw std::runtime_error(
        "Only a derived version of this function with actual implementation "
        "is supposed to be used.");
    return CoreStateMsgGroup{};
  };
  virtual ActuatorStateMsgGroup GetActuatorStateMsgGroup() {
    throw std::runtime_error(
        "Only a derived version of this function with actual implementation "
        "is supposed to be used.");
    return ActuatorStateMsgGroup{};
  };

  // any specific robot will use a specialized version of the two functions
  virtual void SendMotionCommand(double linear_vel, double angular_vel,
                                 double lateral_velocity,
                                 double steering_angle) {
    throw std::runtime_error(
        "Only a derived version of this function with actual implementation "
        "is supposed to be used.");
  };

  virtual void SendLightCommand(LightMode front_mode,
                                uint8_t front_custom_value, LightMode rear_mode,
                                uint8_t rear_custom_value) {
    throw std::runtime_error(
        "Only a derived version of this function with actual implementation "
        "is supposed to be used.");
  };
};
}  // namespace westonrobot

#endif /* ROBOT_INTERFACE_HPP */
