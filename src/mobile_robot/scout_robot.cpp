/*
 * scout_robot.cpp
 *
 * Created on: Jul 08, 2021 11:13
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "ugv_sdk/mobile_robot/scout_robot.hpp"
#include "ugv_sdk/details/robot_base/scout_base.hpp"

namespace westonrobot {
ScoutRobot::ScoutRobot(ProtocolVersion protocol, bool is_mini_model) {
  if (!is_mini_model) {
    if (protocol == ProtocolVersion::AGX_V1) {
      robot_ = new ScoutBaseV1();
    } else if (protocol == ProtocolVersion::AGX_V2) {
      robot_ = new ScoutBaseV2();
    }
  } else {
    if (protocol == ProtocolVersion::AGX_V1) {
      robot_ = new ScoutMiniBaseV1();
    } else if (protocol == ProtocolVersion::AGX_V2) {
      robot_ = new ScoutMiniBaseV2();
    }
  }
}

ScoutRobot::~ScoutRobot() {
  if (robot_) delete robot_;
}

void ScoutRobot::EnableCommandedMode() { robot_->EnableCommandedMode(); }

bool ScoutRobot::Connect(std::string can_name) {
  return robot_->Connect(can_name);
}

std::string ScoutRobot::RequestVersion(int timeout_sec) {
  return robot_->RequestVersion(timeout_sec);
}

void ScoutRobot::ResetRobotState() { robot_->ResetRobotState(); }

ProtocolVersion ScoutRobot::GetParserProtocolVersion() {
  return robot_->GetParserProtocolVersion();
}

void ScoutRobot::SetMotionCommand(double linear_vel, double angular_vel) {
  auto scout = dynamic_cast<ScoutInterface*>(robot_);
  scout->SetMotionCommand(linear_vel, angular_vel);
}

void ScoutRobot::SetLightCommand(AgxLightMode f_mode, uint8_t f_value,
                                 AgxLightMode r_mode, uint8_t r_value) {
  auto scout = dynamic_cast<ScoutInterface*>(robot_);
  scout->SetLightCommand(f_mode, f_value, r_mode, r_value);
}

void ScoutRobot::DisableLightControl() {
  auto scout = dynamic_cast<ScoutInterface*>(robot_);
  scout->DisableLightControl();
}

ScoutCoreState ScoutRobot::GetRobotState() {
  auto scout = dynamic_cast<ScoutInterface*>(robot_);
  return scout->GetRobotState();
}

ScoutActuatorState ScoutRobot::GetActuatorState() {
  auto scout = dynamic_cast<ScoutInterface*>(robot_);
  return scout->GetActuatorState();
}
ScoutCommonSensorState ScoutRobot::GetCommonSensorState() {
  auto scout = dynamic_cast<ScoutInterface*>(robot_);
  return scout->GetCommonSensorState();
}

///////////////////////////////////////////////////////////////////////////

ScoutMiniOmniRobot::ScoutMiniOmniRobot(ProtocolVersion protocol)
    : ScoutRobot(ProtocolVersion::UNKONWN) {
  if (protocol == ProtocolVersion::AGX_V1) {
    robot_ = new ScoutMiniOmniBaseV1();
  } else if (protocol == ProtocolVersion::AGX_V2) {
    robot_ = new ScoutMiniOmniBaseV2();
  }
}

void ScoutMiniOmniRobot::SetMotionCommand(double linear_vel, double angular_vel,
                                          double lateral_velocity) {
  auto scout = dynamic_cast<ScoutOmniInterface*>(robot_);
  scout->SetMotionCommand(linear_vel, angular_vel, lateral_velocity);
}
}  // namespace westonrobot
