/*
 * hunter_robot.hpp
 *
 * Created on: Jul 08, 2021 10:59
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef SCOUT_ROBOT_HPP
#define SCOUT_ROBOT_HPP

#include <memory>

#include "ugv_sdk/details/interface/robot_interface.hpp"
#include "ugv_sdk/details/interface/hunter_interface.hpp"

namespace westonrobot {
class HunterRobot : public RobotInterface, public HunterInterface {
 public:
  HunterRobot(ProtocolVersion protocol = ProtocolVersion::AGX_V2);
  ~HunterRobot();

  void Connect(std::string can_name) override;

  void EnableCommandedMode() override;

  void SetMotionCommand(double linear_vel, double angular_vel) override;

  void ResetRobotState() override;

  ProtocolVersion GetProtocolVersion() override;

  // get robot state
  HunterCoreState GetRobotState() override;
  HunterActuatorState GetActuatorState() override;

 private:
  RobotInterface* robot_;
};
}  // namespace westonrobot

#endif /* SCOUT_ROBOT_HPP */
