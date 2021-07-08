#include "ugv_sdk/protocol_v2/tracer_base.hpp"

#include <string>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <ratio>
#include <thread>

#include "stopwatch.hpp"

#include "ugv_sdk/protocol_v2/agilex_msg_parser.h"

namespace westonrobot {
void TracerBaseV2::Connect(std::string dev_name) {
  AgilexBase::ConnectPort(dev_name, std::bind(&TracerBaseV2::ParseCANFrame, this,
                                              std::placeholders::_1));
}

void TracerBaseV2::Connect(std::string uart_name, uint32_t baudrate) {
  // TODO
}

void TracerBaseV2::SetMotionCommand(double linear_vel, double angular_vel) {
  AgilexBase::SetMotionCommand(linear_vel, angular_vel, 0.0, 0.0);
}

void TracerBaseV2::SetLightCommand(const TracerLightCmd &cmd) {
  if (cmd.cmd_ctrl_allowed) {
    AgilexBase::SendLightCommand(cmd.front_mode, cmd.front_custom_value,
                                 LightMode::CONST_OFF, 0);
  }
}

TracerState TracerBaseV2::GetTracerState() {
  std::lock_guard<std::mutex> guard(state_mutex_);
  return tracer_state_;
}

void TracerBaseV2::ParseCANFrame(can_frame *rx_frame) {
  AgxMessage status_msg;
  DecodeCanFrame(rx_frame, &status_msg);
  std::lock_guard<std::mutex> guard(state_mutex_);
  UpdateTracerState(status_msg, tracer_state_);
}

void TracerBaseV2::UpdateTracerState(const AgxMessage &status_msg,
                                   TracerState &state) {
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
  }
}
}  // namespace westonrobot
