/*
 * bunker_robot.cpp
 *
 * Created on: Jul 14, 2021 23:14
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "ugv_sdk/mobile_robot/bunker_robot.hpp"
#include "ugv_sdk/details/robot_base/bunker_base.hpp"

namespace westonrobot {
BunkerRobot::BunkerRobot(ProtocolVersion protocol) {
  if (protocol == ProtocolVersion::AGX_V1) {
    robot_ = new BunkerBaseV1();
  } else if (protocol == ProtocolVersion::AGX_V2) {
    robot_ = new BunkerBaseV2();
  }
}

BunkerRobot::~BunkerRobot() {
  if (robot_) delete robot_;
}

void BunkerRobot::EnableCommandedMode() { robot_->EnableCommandedMode(); }

void BunkerRobot::Connect(std::string can_name) { robot_->Connect(can_name); }

void BunkerRobot::ResetRobotState() { robot_->ResetRobotState(); }

ProtocolVersion BunkerRobot::GetParserProtocolVersion() {
  return robot_->GetParserProtocolVersion();
}

void BunkerRobot::SetMotionCommand(double linear_vel, double angular_vel) {
  auto bunker = dynamic_cast<BunkerInterface*>(robot_);
  bunker->SetMotionCommand(linear_vel, angular_vel);
}

BunkerCoreState BunkerRobot::GetRobotState() {
  auto bunker = dynamic_cast<BunkerInterface*>(robot_);
  return bunker->GetRobotState();
}

BunkerActuatorState BunkerRobot::GetActuatorState() {
  auto bunker = dynamic_cast<BunkerInterface*>(robot_);
  return bunker->GetActuatorState();
}
}  // namespace westonrobot