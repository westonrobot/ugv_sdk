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

#include "ugv_sdk/details/interface/agilex_message.h"
#include "ugv_sdk/details/interface/robot_common_interface.hpp"

namespace westonrobot {
struct RangerCoreState {
  // system state
  SdkTimePoint time_stamp;

  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  LightStateMessage light_state;
  MotionModeStateMessage motion_mode_state;

  RcStateMessage rc_state;
  OdometryMessage odometry;
};

struct RangerActuatorState {
  SdkTimePoint time_stamp;

  MotorAngleMessage motor_angles;
  MotorSpeedMessage motor_speeds;

  ActuatorHSStateMessage actuator_hs_state[8];
  ActuatorLSStateMessage actuator_ls_state[8];
};

struct RangerCommonSensorState {
  SdkTimePoint time_stamp;

  BmsBasicMessage bms_basic_state;
};

/////////////////////////////////////////////////////////////////////////

struct RangerInterface {
  enum MotionMode {
    kDualAckerman = 0,
    kParallel = 1,
    kSpinning = 2,
    kPark = 3,
    kSideSlip = 4
  };

  virtual ~RangerInterface() = default;

  // robot control
  virtual void SetMotionMode(uint8_t mode) = 0;
  virtual void SetMotionCommand(double linear_vel, double steer_angle,
                                double angular_vel) = 0;
  virtual void SetLightCommand(AgxLightMode f_mode, uint8_t f_value,
                               AgxLightMode r_mode, uint8_t r_value) = 0;
  virtual void DisableLightControl() = 0;

  // get robot state
  virtual RangerCoreState GetRobotState() = 0;
  virtual RangerActuatorState GetActuatorState() = 0;
  virtual RangerCommonSensorState GetCommonSensorState() = 0;
};
}  // namespace westonrobot

#endif /* RANGER_INTERFACE_HPP */
