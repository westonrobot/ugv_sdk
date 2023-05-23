/*
 * ranger_robot.hpp
 *
 * Created on: Jul 14, 2021 21:45
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef RANGER_ROBOT_HPP
#define RANGER_ROBOT_HPP

#include <cmath>

#include "ugv_sdk/details/robot_base/ranger_base.hpp"

namespace westonrobot {
using RangerRobot = RangerBaseV2;
using RangerMiniV2Robot = RangerBaseV2;

// Note: Ranger Mini V1 is using a modified AgileX V2 protocol
// Here we provide a work-around fix as no new firmware will be provided from
// AgileX to properly fix the issue.
class RangerMiniV1Robot : public RangerBaseV2 {
 public:
  RangerMiniV1Robot() : RangerBaseV2(){};
  ~RangerMiniV1Robot() = default;

  // robot control
  void SetMotionCommand(double linear_vel, double steer_angle,
                        double angular_vel = 0.0) override;
  RangerCoreState GetRobotState() override;
  RangerActuatorState GetActuatorState() override;
};
}  // namespace westonrobot

#endif /* RANGER_ROBOT_HPP */
