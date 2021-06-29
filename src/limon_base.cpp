#include "ugv_sdk/limon_base.h"

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

using namespace std::placeholders;

namespace westonrobot {
void LimonBase::Connect(std::string dev_name) {
//   AgilexBaseSerialPort::Connect(
//       dev_name,
//       std::bind(&LimonBase::ParseSerialFrame, this, ::_1, ::_2, ::_3));
}

void LimonBase::SetMotionCommand(double linear_vel, double steer_angle,
                                 double lateral_vel, double angular_vel) {
  LimonBase::SetMotionCommand(linear_vel, angular_vel, lateral_vel,
                              steer_angle / 10.0);
}

void LimonBase::SetLightCommand(const LimonLightCmd &cmd) {
  if (cmd.cmd_ctrl_allowed) {
    LimonBase::SendLightCommand(cmd.front_mode, cmd.front_custom_value,
                                LightMode::CONST_OFF, 0);
  }
}

void LimonBase::SetMotionMode(uint8_t mode) { LimonBase::SetMotionMode(mode); }

LimonState LimonBase::GetLimonState() {
  std::lock_guard<std::mutex> guard(state_mutex_);
  return ranger_state_;
}

void LimonBase::ParseCANFrame(can_frame *rx_frame) {
  AgxMessage status_msg;
  DecodeCanFrame(rx_frame, &status_msg);
  std::lock_guard<std::mutex> guard(state_mutex_);
  UpdateLimonState(status_msg, ranger_state_);
}

void LimonBase::UpdateLimonState(const AgxMessage &status_msg,
                                 LimonState &state) {
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
void LimonBase::ParseSerialFrame(uint8_t *data, const size_t bufsize, size_t len)
{

}
void LimonBase::ParseSerialFrame(can_frame *rx_frame)
{
    
}
}  // namespace westonrobot
