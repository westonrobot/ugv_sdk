/*
 * titan_robot.cpp
 *
 * Created on: Feb 23, 2023 17:10
 * Description:
 *
 * Copyright (c) 2023 Weston Robot Pte. Ltd.
 */
#include "ugv_sdk/mobile_robot/titan_robot.hpp"
#include "ugv_sdk/details/robot_base/titan_base.hpp"

namespace westonrobot {
TitanRobot::TitanRobot() {
    robot_ = new TitanBase();
}

TitanRobot::~TitanRobot() {
  if (robot_) delete robot_;
}

bool TitanRobot::Connect(std::string can_name) {
  return robot_->Connect(can_name);
}

void TitanRobot::EnableCommandedMode() { robot_->EnableCommandedMode(); }

std::string TitanRobot::RequestVersion(int timeout_sec) {
  return robot_->RequestVersion(timeout_sec);
}

// functions to be implemented by each robot class
void TitanRobot::ResetRobotState() { robot_->ResetRobotState(); }

ProtocolVersion TitanRobot::GetParserProtocolVersion() {
  return robot_->GetParserProtocolVersion();
}

// robot control
void TitanRobot::SetMotionCommand(double linear_vel, double steer_angle) {
  auto titan = dynamic_cast<TitanInterface*>(robot_);
  return titan->SetMotionCommand(linear_vel, steer_angle);
}

void TitanRobot::ActivateBrake() {
  auto hunter = dynamic_cast<TitanInterface*>(robot_);
  hunter->ActivateBrake();
}

void TitanRobot::ReleaseBrake() {
  auto hunter = dynamic_cast<TitanInterface*>(robot_);
  hunter->ReleaseBrake();
}

// get robot state
TitanCoreState TitanRobot::GetRobotState() {
  auto titan = dynamic_cast<TitanInterface*>(robot_);
  return titan->GetRobotState();
}

TitanActuatorState TitanRobot::GetActuatorState() {
  auto titan = dynamic_cast<TitanInterface*>(robot_);
  return titan->GetActuatorState();
}
TitanCommonSensorState TitanRobot::GetCommonSensorState() {
  auto titan = dynamic_cast<TitanInterface*>(robot_);
  return titan->GetCommonSensorState();
}
}  // namespace westonrobot