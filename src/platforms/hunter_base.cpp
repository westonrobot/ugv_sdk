#include "wrp_sdk/platforms/hunter/hunter_base.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <ratio>
#include <string>
#include <thread>

#include "stopwatch.h"

namespace westonrobot {

void HunterBase::SendRobotCmd() {
  static uint8_t cmd_count = 0;
  SendMotionCmd(cmd_count++);
}

void HunterBase::SendMotionCmd(uint8_t count) {
  // motion control message
  HunterMessage m_msg;
  m_msg.type = HunterMotionCmdMsg;

  if (can_connected_)
    m_msg.body.motion_cmd_msg.data.cmd.control_mode = CTRL_MODE_CMD_CAN;
  else if (serial_connected_)
    m_msg.body.motion_cmd_msg.data.cmd.control_mode = CTRL_MODE_CMD_UART;

  motion_cmd_mutex_.lock();
  m_msg.body.motion_cmd_msg.data.cmd.fault_clear_flag =
      static_cast<uint8_t>(current_motion_cmd_.fault_clear_flag);
  m_msg.body.motion_cmd_msg.data.cmd.linear_velocity_cmd =
      current_motion_cmd_.linear_velocity;
  m_msg.body.motion_cmd_msg.data.cmd.angular_velocity_cmd =
      current_motion_cmd_.angular_velocity;
  motion_cmd_mutex_.unlock();

  m_msg.body.motion_cmd_msg.data.cmd.reserved0 = 0;
  m_msg.body.motion_cmd_msg.data.cmd.reserved1 = 0;
  m_msg.body.motion_cmd_msg.data.cmd.count = count;

  if (can_connected_)
    m_msg.body.motion_cmd_msg.data.cmd.checksum = CalcHunterCANChecksum(
        CAN_MSG_MOTION_CMD_ID, m_msg.body.motion_cmd_msg.data.raw, 8);

  // send to can bus
  if (can_connected_) {
    can_frame m_frame;
    EncodeHunterMsgToCAN(&m_msg, &m_frame);
    can_if_->send_frame(m_frame);
  }
}

HunterState HunterBase::GetHunterState() {
  std::lock_guard<std::mutex> guard(hunter_state_mutex_);
  return hunter_state_;
}

void HunterBase::SetMotionCommand(
    double linear_vel, double steering_angle,
    HunterMotionCmd::FaultClearFlag fault_clr_flag) {
  // make sure cmd thread is started before attempting to send commands
  if (!cmd_thread_started_) StartCmdThread();

  if (linear_vel < HunterMotionCmd::min_linear_velocity)
    linear_vel = HunterMotionCmd::min_linear_velocity;
  if (linear_vel > HunterMotionCmd::max_linear_velocity)
    linear_vel = HunterMotionCmd::max_linear_velocity;
  if (steering_angle < HunterMotionCmd::min_steering_angle)
    steering_angle = HunterMotionCmd::min_steering_angle;
  if (steering_angle > HunterMotionCmd::max_steering_angle)
    steering_angle = HunterMotionCmd::max_steering_angle;

  std::lock_guard<std::mutex> guard(motion_cmd_mutex_);
  current_motion_cmd_.linear_velocity = static_cast<int8_t>(
      linear_vel / HunterMotionCmd::max_linear_velocity * 100.0);
  current_motion_cmd_.angular_velocity = static_cast<int8_t>(
      steering_angle / HunterMotionCmd::max_steering_angle * 100.0);
  current_motion_cmd_.fault_clear_flag = fault_clr_flag;
}

void HunterBase::ParseCANFrame(can_frame *rx_frame) {
  // validate checksum, discard frame if fails
  if (!rx_frame->data[7] == CalcHunterCANChecksum(rx_frame->can_id,
                                                  rx_frame->data,
                                                  rx_frame->can_dlc)) {
    std::cerr << "ERROR: checksum mismatch, discard frame with id "
              << rx_frame->can_id << std::endl;
    return;
  }

  // otherwise, update robot state with new frame
  HunterMessage status_msg;
  DecodeHunterMsgFromCAN(rx_frame, &status_msg);
  NewStatusMsgReceivedCallback(status_msg);
}

void HunterBase::NewStatusMsgReceivedCallback(const HunterMessage &msg) {
  // std::cout << "new status msg received" << std::endl;
  std::lock_guard<std::mutex> guard(hunter_state_mutex_);
  UpdateHunterState(msg, hunter_state_);
}

void HunterBase::UpdateHunterState(const HunterMessage &status_msg,
                                   HunterState &state) {
  switch (status_msg.type) {
    case HunterMotionStatusMsg: {
      // std::cout << "motion control feedback received" << std::endl;
      const MotionStatusMessage &msg = status_msg.body.motion_status_msg;
      state.linear_velocity =
          static_cast<int16_t>(
              static_cast<uint16_t>(msg.data.status.linear_velocity.low_byte) |
              static_cast<uint16_t>(msg.data.status.linear_velocity.high_byte)
                  << 8) /
          1000.0;
      state.steering_angle =
          static_cast<int16_t>(
              static_cast<uint16_t>(msg.data.status.angular_velocity.low_byte) |
              static_cast<uint16_t>(msg.data.status.angular_velocity.high_byte)
                  << 8) /
          1000.0;
      break;
    }
    case HunterSystemStatusMsg: {
      // std::cout << "system status feedback received" << std::endl;
      const SystemStatusMessage &msg = status_msg.body.system_status_msg;
      state.control_mode = msg.data.status.control_mode;
      state.base_state = msg.data.status.base_state;
      state.battery_voltage =
          (static_cast<uint16_t>(msg.data.status.battery_voltage.low_byte) |
           static_cast<uint16_t>(msg.data.status.battery_voltage.high_byte)
               << 8) /
          10.0;
      state.fault_code =
          (static_cast<uint16_t>(msg.data.status.fault_code.low_byte) |
           static_cast<uint16_t>(msg.data.status.fault_code.high_byte) << 8);
      break;
    }
    case HunterMotorDriverStatusMsg: {
      // std::cout << "motor 1 driver feedback received" << std::endl;
      const MotorDriverStatusMessage &msg =
          status_msg.body.motor_driver_status_msg;
      for (int i = 0; i < HunterState::motor_num; ++i) {
        state.motor_states[status_msg.body.motor_driver_status_msg.motor_id]
            .current =
            (static_cast<uint16_t>(msg.data.status.current.low_byte) |
             static_cast<uint16_t>(msg.data.status.current.high_byte) << 8) /
            10.0;
        state.motor_states[status_msg.body.motor_driver_status_msg.motor_id]
            .rpm = static_cast<int16_t>(
            static_cast<uint16_t>(msg.data.status.rpm.low_byte) |
            static_cast<uint16_t>(msg.data.status.rpm.high_byte) << 8);
        state.motor_states[status_msg.body.motor_driver_status_msg.motor_id]
            .temperature = msg.data.status.temperature;
      }
      break;
    }
    case HunterConfigStatusMsg: {
      const ConfigStatusMessage &msg = status_msg.body.config_status_msg;
      state.set_zero_steering = msg.data.status.set_zero_steering;
      break;
    }
    default:
      break;
  }
}
}  // namespace westonrobot
