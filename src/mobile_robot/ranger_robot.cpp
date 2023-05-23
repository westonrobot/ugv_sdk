/*
 * ranger_robot.cpp
 *
 * Created on: Feb 23, 2023 17:10
 * Description:
 *
 * Copyright (c) 2023 Zps Pte. Ltd.
 */
#include "ugv_sdk/mobile_robot/ranger_robot.hpp"
#include "ugv_sdk/details/robot_base/ranger_base.hpp"

namespace westonrobot {
// robot control
void RangerMiniV1Robot::SetMotionCommand(double linear_vel, double steer_angle,
                                         double angular_vel) {
  AgilexBase<ProtocolV2Parser>::SendMotionCommand(
      linear_vel, 0.0, angular_vel, -steer_angle / 10.0 / 3.14 * 180);
}

RangerCoreState RangerMiniV1Robot::GetRobotState() {
  auto state = AgilexBase<ProtocolV2Parser>::GetRobotCoreStateMsgGroup();

  RangerCoreState ranger_state;
  ranger_state.time_stamp = state.time_stamp;
  ranger_state.system_state = state.system_state;

  ranger_state.motion_state.linear_velocity =
      state.motion_state.linear_velocity;
  ranger_state.motion_state.angular_velocity =
      -state.motion_state.angular_velocity;
  ranger_state.motion_state.lateral_velocity =
      state.motion_state.lateral_velocity;
  ranger_state.motion_state.steering_angle =
      -state.motion_state.steering_angle * 10 / 180.0 * 3.14;

  ranger_state.light_state = state.light_state;
  ranger_state.rc_state = state.rc_state;
  ranger_state.current_motion_mode = state.motion_mode_state;
  return ranger_state;
}

RangerActuatorState RangerMiniV1Robot::GetActuatorState() {
  auto actuator = AgilexBase<ProtocolV2Parser>::GetActuatorStateMsgGroup();

  RangerActuatorState ranger_actuator;
  ranger_actuator.time_stamp = actuator.time_stamp;

  ranger_actuator.motor_speeds.speed_1 = actuator.motor_speeds.speed_1;
  ranger_actuator.motor_speeds.speed_2 = actuator.motor_speeds.speed_2;
  ranger_actuator.motor_speeds.speed_3 = actuator.motor_speeds.speed_3;
  ranger_actuator.motor_speeds.speed_4 = actuator.motor_speeds.speed_4;
  ranger_actuator.motor_angles.angle_5 =
      -actuator.motor_angles.angle_5 / 18.0 * M_PI;
  ranger_actuator.motor_angles.angle_6 =
      -actuator.motor_angles.angle_6 / 18.0 * M_PI;
  ranger_actuator.motor_angles.angle_7 =
      -actuator.motor_angles.angle_7 / 18.0 * M_PI;
  ranger_actuator.motor_angles.angle_8 =
      -actuator.motor_angles.angle_8 / 18.0 * M_PI;

  for (int i = 0; i < 8; ++i) {
    ranger_actuator.actuator_hs_state[i] = actuator.actuator_hs_state[i];
    ranger_actuator.actuator_ls_state[i] = actuator.actuator_ls_state[i];
  }
  return ranger_actuator;
}
}  // namespace westonrobot