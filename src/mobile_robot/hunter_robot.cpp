/*
 * hunter_robot.cpp
 *
 * Created on: Jul 14, 2021 23:30
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "ugv_sdk/mobile_robot/hunter_robot.hpp"
#include "ugv_sdk/details/robot_base/hunter_base.hpp"

namespace westonrobot {
HunterRobot::HunterRobot(ProtocolVersion protocol) {
  if (protocol == ProtocolVersion::AGX_V1) {
    robot_ = new HunterBaseV1();
  } else if (protocol == ProtocolVersion::AGX_V2) {
    robot_ = new HunterBaseV2();
  }
}

HunterRobot::~HunterRobot() {
  if (robot_) delete robot_;
}

void HunterRobot::EnableCommandedMode() { robot_->EnableCommandedMode(); }

void HunterRobot::Connect(std::string can_name) { robot_->Connect(can_name); }

void HunterRobot::ResetRobotState() { robot_->ResetRobotState(); }

ProtocolVersion HunterRobot::GetParserProtocolVersion() {
  return robot_->GetParserProtocolVersion();
}

void HunterRobot::SetMotionCommand(double linear_vel, double angular_vel) {
  auto hunter = dynamic_cast<HunterInterface*>(robot_);
  hunter->SetMotionCommand(linear_vel, angular_vel);
}

void HunterRobot::ActivateBrake() {
  auto hunter = dynamic_cast<HunterInterface*>(robot_);
  hunter->ActivateBrake();
}

void HunterRobot::ReleaseBrake() {
  auto hunter = dynamic_cast<HunterInterface*>(robot_);
  hunter->ReleaseBrake();
}

HunterCoreState HunterRobot::GetRobotState() {
  auto hunter = dynamic_cast<HunterInterface*>(robot_);
  return hunter->GetRobotState();
}

HunterActuatorState HunterRobot::GetActuatorState() {
  auto hunter = dynamic_cast<HunterInterface*>(robot_);
  return hunter->GetActuatorState();
}
}  // namespace westonrobot