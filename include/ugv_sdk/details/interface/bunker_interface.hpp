/**
 * @file bunker_interface.hpp
 * @brief Bunker robot common interface
 * @date 06-01-2025
 * 
 * @copyright Copyright (c) 2025 Weston Robot Pte. Ltd.
 */
#ifndef BUNKER_INTERFACE_HPP
#define BUNKER_INTERFACE_HPP

#include <string>

#include "ugv_sdk/details/interface/agilex_message.h"
#include "ugv_sdk/details/interface/robot_common_interface.hpp"

namespace westonrobot {
struct BunkerCoreState {
  SdkTimePoint time_stamp;

  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  RcStateMessage rc_state;
};

struct BunkerActuatorState {
  SdkTimePoint time_stamp;

  // actuator state
  ActuatorHSStateMessage actuator_hs_state[2];
  ActuatorLSStateMessage actuator_ls_state[2];
  // - for v1 robots only
  ActuatorStateMessageV1 actuator_state[2];
};

struct BunkerCommonSensorState {
  SdkTimePoint time_stamp;

  BmsBasicMessage bms_basic_state;
};

struct BunkerInterface {
  virtual ~BunkerInterface() = default;

  virtual void SetMotionCommand(double linear_vel, double angular_vel) = 0;

  // get robot state
  virtual BunkerCoreState GetRobotState() = 0;
  virtual BunkerActuatorState GetActuatorState() = 0;
  virtual BunkerCommonSensorState GetCommonSensorState() = 0;
};
}  // namespace westonrobot

#endif /* BUNKER_INTERFACE_HPP */
