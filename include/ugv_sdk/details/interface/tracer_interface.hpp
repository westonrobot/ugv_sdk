/*
 * tracer_interface.hpp
 *
 * Created on: Jul 08, 2021 09:36
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef TRACER_INTERFACE_HPP
#define TRACER_INTERFACE_HPP

#include <string>

#include "ugv_sdk/details/interface/agilex_message.h"
#include "ugv_sdk/details/interface/robot_common_interface.hpp"

namespace westonrobot {
struct TracerCoreState {
  AgxMsgTimeStamp time_stamp;

  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  LightStateMessage light_state;
  RcStateMessage rc_state;
};

struct TracerActuatorState {
  AgxMsgTimeStamp time_stamp;

  // actuator state
  ActuatorHSStateMessage actuator_hs_state[2];
  ActuatorLSStateMessage actuator_ls_state[2];
};

struct TracerInterface {
  virtual ~TracerInterface() = default;

  virtual void Connect(std::string uart_name, uint32_t baudrate){
      // use derived version
  };

  virtual void SetMotionCommand(double linear_vel, double angular_vel) = 0;
  virtual void SetLightCommand(LightMode f_mode, uint8_t f_value) = 0;

  // get robot state
  virtual TracerCoreState GetRobotState() = 0;
  virtual TracerActuatorState GetActuatorState() = 0;
};
}  // namespace westonrobot

#endif /* TRACER_INTERFACE_HPP */
