/*
 * scout_base.cpp
 *
 * Created on: Jul 08, 2021 12:07
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "ugv_sdk/protocol_v2/scout_base_v2.hpp"

#include <string>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <ratio>
#include <thread>

#include "ugv_sdk/protocol_v2/agilex_msg_parser.h"

namespace westonrobot {
void ScoutBaseV2::Connect(std::string dev_name) {
  AgilexBase::ConnectPort(dev_name, std::bind(&ScoutBaseV2::ParseCANFrame, this,
                                              std::placeholders::_1));
}

void ScoutBaseV2::Connect(std::string uart_name, uint32_t baudrate) {
  // TODO
}

void ScoutBaseV2::SetMotionCommand(double linear_vel, double angular_vel) {
  AgilexBase::SendMotionCommand(linear_vel, angular_vel, 0.0, 0.0);
}

void ScoutBaseV2::SetLightCommand(LightMode f_mode, uint8_t f_value,
                                LightMode r_mode, uint8_t r_value) {
  AgilexBase::SendLightCommand(f_mode, f_value, r_mode, r_value);
}

ScoutState ScoutBaseV2::GetRobotState() {
  std::lock_guard<std::mutex> guard(state_mutex_);
  return scout_state_;
}

void ScoutBaseV2::ParseCANFrame(can_frame *rx_frame) {
  AgxMessage status_msg;
  DecodeCanFrame(rx_frame, &status_msg);
  std::lock_guard<std::mutex> guard(state_mutex_);
  UpdateScoutState(status_msg, scout_state_);
}

void ScoutBaseV2::UpdateScoutState(const AgxMessage &status_msg,
                                 ScoutState &state) {
  switch (status_msg.type) {
    case AgxMsgSystemState: {
      //   std::cout << "system status feedback received" << std::endl;
      state.system_state = status_msg.body.system_state_msg;
      break;
    }
    case AgxMsgMotionState: {
      // std::cout << "motion control feedback received" << std::endl;
      state.motion_state = status_msg.body.motion_state_msg;
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
