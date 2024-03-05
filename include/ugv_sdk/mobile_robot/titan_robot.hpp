/*
 * ranger_robot.hpp
 *
 * Created on: Jul 14, 2021 21:45
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef TITAN_ROBOT_HPP
#define TITAN_ROBOT_HPP

#include <cmath>

#include "ugv_sdk/details/robot_base/titan_base.hpp"

namespace westonrobot {
class TitanRobot : public RobotCommonInterface, public TitanInterface {
 public:
  TitanRobot();
  ~TitanRobot();

  bool Connect(std::string can_name) override;

  void EnableCommandedMode() override;
  std::string RequestVersion(int timeout_sec) override;

  void ActivateBrake() override;
  void ReleaseBrake() override;

  // functions to be implemented by each robot class
  void ResetRobotState() override;

  ProtocolVersion GetParserProtocolVersion() override;

  // robot control
  void SetMotionCommand(double linear_vel, double steer_angle) override;

  // get robot state
  TitanCoreState GetRobotState() override;
  TitanActuatorState GetActuatorState() override;
  TitanCommonSensorState GetCommonSensorState() override;

 private:
  RobotCommonInterface* robot_;
};
}  // namespace westonrobot

#endif /* TITAN_ROBOT_HPP */
