/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-04-19  19:36:24
 * @FileName  : ranger_base.cpp
 * @Mail      : wangzheqie@qq.com
 * Copyright  : AgileX Robotics
 **/

#include "ugv_sdk/ranger_base.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <ratio>
#include <string>
#include <thread>

#include "stopwatch.hpp"

#include "ugv_sdk/details/agilex_msg_parser.h"

namespace westonrobot {
void RangerBase::Connect(std::string dev_name) {
  AgilexBase::Connect(dev_name, std::bind(&RangerBase::ParseCANFrame, this,
                                          std::placeholders::_1));
}

void RangerBase::SetMotionCommand(double linear_vel, double steer_angle,
                                  double lateral_vel, double angular_vel) {
  AgilexBase::SetMotionCommand(linear_vel, angular_vel, lateral_vel,
                               steer_angle / 10.0);
}

void RangerBase::SetLightCommand(const RangerLightCmd &cmd) {
  if (cmd.cmd_ctrl_allowed) {
    AgilexBase::SendLightCommand(cmd.front_mode, cmd.front_custom_value,
                                 LightMode::CONST_OFF, 0);
  }
}

void RangerBase::SetMotionMode(uint8_t mode) {
  AgilexBase::SetMotionMode(mode);
}

RangerState RangerBase::GetRangerState() {
  std::lock_guard<std::mutex> guard(state_mutex_);
  return ranger_state_;
}

void RangerBase::ParseCANFrame(can_frame *rx_frame) {
  AgxMessage status_msg;
  DecodeCanFrame(rx_frame, &status_msg);
  std::lock_guard<std::mutex> guard(state_mutex_);
  UpdateRangerState(status_msg, ranger_state_);
}

void RangerBase::UpdateRangerState(const AgxMessage &status_msg,
                                   RangerState &state) {
  switch (status_msg.type) {
    case AgxMsgSystemState: {
      //   std::cout << "system status feedback received" << std::endl;
      state.system_state = status_msg.body.system_state_msg;
      break;
    }
    case AgxMsgMotionState: {
      //  std::cout << "motion control feedback received" << std::endl;
      state.motion_state = status_msg.body.motion_state_msg;
      state.motion_state.steering_angle *= 10;
      // std::cout << "steering angle: " << state.motion_state.steering_angle <<
      // std::endl;
      break;
    }
    case AgxMsgLightState: {
      // std::cout << "light control feedback received" << std::endl;
      state.light_state = status_msg.body.light_state_msg;
      break;
    }
    case AgxMsgRcState: {
      state.rc_state = status_msg.body.rc_state_msg;
      break;
    }
    case AgxMsgActuatorHSState: {
      // std::cout << "actuator hs feedback received" << std::endl;
      state.actuator_hs_state[status_msg.body.actuator_hs_state_msg.motor_id] =
          status_msg.body.actuator_hs_state_msg;
      break;
    }
    case AgxMsgActuatorLSState: {
      // std::cout << "actuator ls feedback received" << std::endl;
      state.actuator_ls_state[status_msg.body.actuator_ls_state_msg.motor_id] =
          status_msg.body.actuator_ls_state_msg;
      break;
    }
    case AgxMsgMotionModeState: {
      state.current_motion_mode = status_msg.body.motion_mode_feedback_msg;
      break;
    }
    /* sensor feedback */
    case AgxMsgOdometry: {
      // std::cout << "Odometer msg feedback received" << std::endl;
      state.odometry = status_msg.body.odometry_msg;
    }
    default:
      break;
  }
}
}  // namespace westonrobot
