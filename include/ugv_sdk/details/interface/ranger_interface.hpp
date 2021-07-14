/*
 * ranger_interface.hpp
 *
 * Created on: Jul 08, 2021 09:40
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef RANGER_INTERFACE_HPP
#define RANGER_INTERFACE_HPP

#include <string>

#include "ugv_sdk/interface/agilex_message.h"

namespace westonrobot {
struct RangerState {
  // system state
  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  LightStateMessage light_state;

  RcStateMessage rc_state;

  ActuatorHSStateMessage actuator_hs_state[8];
  ActuatorLSStateMessage actuator_ls_state[8];
  MotionModeStateMessage current_motion_mode;

  // sensor data
  OdometryMessage odometry;
};

struct RangerMotionCmd {
  double linear_velocity;
  double angular_velocity;
};

struct RangerLightCmd {
  RangerLightCmd() = default;
  RangerLightCmd(LightMode f_mode, uint8_t f_value, LightMode r_mode,
                 uint8_t r_value)
      : enable_cmd_ctrl(true),
        front_mode(f_mode),
        front_custom_value(f_value),
        rear_mode(r_mode),
        rear_custom_value(r_value) {}

  bool enable_cmd_ctrl = false;
  LightMode front_mode;
  uint8_t front_custom_value;
  LightMode rear_mode;
  uint8_t rear_custom_value;
};

/////////////////////////////////////////////////////////////////////////

struct RangerInterface {
  virtual void Connect(std::string dev_name) = 0;

  // robot control
  virtual void SetMotionCommand(double linear_vel, double steer_angle,
                                double lateral_vel, double angular_vel) = 0;
  virtual void SetLightCommand(const RangerLightCmd &cmd) = 0;
  virtual void SetMotionMode(uint8_t mode) = 0;

  // get robot state
  virtual RangerState GetRangerState() = 0;
};
}  // namespace westonrobot

#endif /* RANGER_INTERFACE_HPP */
