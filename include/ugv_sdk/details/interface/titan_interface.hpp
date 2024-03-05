/*
 * ranger_interface.hpp
 *
 * Created on: Jul 08, 2021 09:40
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef TITAN_INTERFACE_HPP
#define TITAN_INTERFACE_HPP

#include <string>

#include "ugv_sdk/details/interface/agilex_message.h"
#include "ugv_sdk/details/interface/robot_common_interface.hpp"

namespace westonrobot {
struct TitanCoreState {
  // system state
  SdkTimePoint time_stamp;

  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  MotionModeStateMessage motion_mode_state;

  RcStateMessage rc_state;
  OdometryMessage odometry;
};

struct TitanActuatorState {
  SdkTimePoint time_stamp;

  ActuatorHSStateMessage actuator_hs_state[4];
  ActuatorLSStateMessage actuator_ls_state[4];
};

struct TitanCommonSensorState {
  SdkTimePoint time_stamp;

  BmsBasicMessage bms_basic_state;
};

/////////////////////////////////////////////////////////////////////////

struct TitanInterface {

  virtual ~TitanInterface() = default;

  // robot control
  virtual void SetMotionCommand(double linear_vel, double steer_angle) = 0;
  virtual void ActivateBrake() = 0;
  virtual void ReleaseBrake() = 0;

  // get robot state
  virtual TitanCoreState GetRobotState() = 0;
  virtual TitanActuatorState GetActuatorState() = 0;
  virtual TitanCommonSensorState GetCommonSensorState() = 0;
};
}  // namespace westonrobot

#endif /* TITAN_INTERFACE_HPP */
