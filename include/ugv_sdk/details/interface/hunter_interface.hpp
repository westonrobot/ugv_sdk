/*
 * hunter_interface.hpp
 *
 * Created on: Jul 14, 2021 23:21
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef HUNTER_INTERFACE_HPP
#define HUNTER_INTERFACE_HPP

#include <string>

#include "ugv_sdk/details/interface/agilex_message.h"

namespace westonrobot {
struct HunterCoreState {
  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  LightStateMessage light_state;
  RcStateMessage rc_state;
};

struct HunterActuatorState {
  // actuator state
  ActuatorHSStateMessage actuator_hs_state[3];
  ActuatorLSStateMessage actuator_ls_state[3];
  // - for v1 robots only
  ActuatorStateMessageV1 actuator_state[3];
};

struct HunterInterface {
  virtual ~HunterInterface() = default;

  virtual void SetMotionCommand(double linear_vel, double steering_angle) = 0;

  // get robot state
  virtual HunterCoreState GetRobotState() = 0;
  virtual HunterActuatorState GetActuatorState() = 0;
};
}  // namespace westonrobot

#endif /* HUNTER_INTERFACE_HPP */
