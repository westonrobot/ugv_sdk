/*
 * scout_interface.hpp
 *
 * Created on: Jul 08, 2021 12:02
 * Description: Scout-specific interface
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef SCOUT_INTERFACE_HPP
#define SCOUT_INTERFACE_HPP

#include <string>

#include "ugv_sdk/details/interface/agilex_message.h"
#include "ugv_sdk/details/interface/robot_common_interface.hpp"

namespace westonrobot {
struct ScoutCoreState {
  SdkTimePoint time_stamp;

  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  LightStateMessage light_state;
  RcStateMessage rc_state;
};

struct ScoutActuatorState {
  SdkTimePoint time_stamp;

  // actuator state
  // - for v2 robots only
  ActuatorHSStateMessage actuator_hs_state[4];
  ActuatorLSStateMessage actuator_ls_state[4];
  // - for v1 robots only
  ActuatorStateMessageV1 actuator_state[4];
};

struct ScoutCommonSensorState {
  SdkTimePoint time_stamp;

  BmsBasicMessage bms_basic_state;
};

struct ScoutInterface {
  virtual ~ScoutInterface() = default;

  virtual void SetMotionCommand(double linear_vel, double angular_vel) = 0;
  virtual void SetLightCommand(AgxLightMode f_mode, uint8_t f_value,
                               AgxLightMode r_mode, uint8_t r_value) = 0;
  virtual void DisableLightControl() = 0;

  // get robot state
  virtual ScoutCoreState GetRobotState() = 0;
  virtual ScoutActuatorState GetActuatorState() = 0;
  virtual ScoutCommonSensorState GetCommonSensorState() = 0;
};

struct ScoutOmniInterface {
  virtual ~ScoutOmniInterface() = default;

  virtual void SetMotionCommand(double linear_vel, double angular_vel,
                                double lateral_velocity) = 0;
};
}  // namespace westonrobot

#endif /* SCOUT_INTERFACE_HPP */
