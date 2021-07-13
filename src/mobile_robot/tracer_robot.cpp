/*
 * tracer_robot.cpp
 *
 * Created on: Jul 13, 2021 22:13
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "ugv_sdk/mobile_robot/tracer_robot.hpp"
#include "ugv_sdk/details/robot_base/tracer_base.hpp"

namespace westonrobot {
TracerRobot::TracerRobot() {
  robot_ = new TracerBaseV2();
}

TracerRobot::~TracerRobot() {
  if (robot_) delete robot_;
}

void TracerRobot::EnableCommandedMode() { robot_->EnableCommandedMode(); }

void TracerRobot::Connect(std::string can_name) { robot_->Connect(can_name); }

void TracerRobot::Connect(std::string uart_name, uint32_t baudrate) {
  robot_->Connect(uart_name, baudrate);
}

void TracerRobot::ResetRobotState() { robot_->ResetRobotState(); }

ProtocolVersion TracerRobot::GetProtocolVersion() {
  return robot_->GetProtocolVersion();
}

void TracerRobot::SetMotionCommand(double linear_vel, double angular_vel) {
  auto tracer = dynamic_cast<TracerInterface*>(robot_);
  tracer->SetMotionCommand(linear_vel, angular_vel);
}

void TracerRobot::DisableLightControl() { robot_->DisableLightControl(); }

void TracerRobot::SetLightCommand(LightMode f_mode, uint8_t f_value) {
  auto tracer = dynamic_cast<TracerInterface*>(robot_);
  tracer->SetLightCommand(f_mode, f_value);
}

TracerState TracerRobot::GetRobotState() {
  auto tracer = dynamic_cast<TracerInterface*>(robot_);
  return tracer->GetRobotState();
}
}  // namespace westonrobot