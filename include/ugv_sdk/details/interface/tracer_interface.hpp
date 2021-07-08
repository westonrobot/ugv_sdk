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

#include "ugv_sdk/interface/agilex_message.h"

namespace westonrobot {
struct TracerState {
  // system state
  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  LightStateMessage light_state;

  RcStateMessage rc_state;

  ActuatorHSStateMessage actuator_hs_state[2];
  ActuatorLSStateMessage actuator_ls_state[2];

  // sensor data
  OdometryMessage odometry;
};

struct TracerMotionCmd {
  double linear_velocity;
  double angular_velocity;
};

struct TracerLightCmd {
  TracerLightCmd() = default;
  TracerLightCmd(LightMode f_mode, uint8_t f_value)
      : cmd_ctrl_allowed(true),
        front_mode(f_mode),
        front_custom_value(f_value) {}

  bool cmd_ctrl_allowed = false;
  LightMode front_mode;
  uint8_t front_custom_value;
};

/////////////////////////////////////////////////////////////////////////

struct TracerInterface {
  // set up connection
  virtual void Connect(std::string can_name) = 0;
  virtual void Connect(std::string uart_name, uint32_t baudrate) = 0;

  // robot control
  virtual void SetMotionCommand(double linear_vel, double angular_vel) = 0;
  virtual void SetLightCommand(const TracerLightCmd &cmd) = 0;

  // get robot state
  virtual TracerState GetTracerState() = 0;
};
}  // namespace westonrobot

#endif /* TRACER_INTERFACE_HPP */
