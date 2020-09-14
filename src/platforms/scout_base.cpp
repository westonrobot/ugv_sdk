#include "wrp_sdk/platforms/scout/scout_base.hpp"

#include <string>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <ratio>
#include <thread>

#include "stopwatch.h"

namespace westonrobot {
void ScoutBase::SendRobotCmd() {
  static uint8_t cmd_count = 0;
  static uint8_t light_cmd_count = 0;
  SendMotionCmd(cmd_count++);
  if (light_ctrl_requested_) SendLightCmd(light_cmd_count++);
}

void ScoutBase::SendMotionCmd(uint8_t count) {
  // motion control message
  ScoutMessage m_msg;
  m_msg.type = ScoutMotionControlMsg;

  if (can_connected_)
    m_msg.body.motion_control_msg.data.cmd.control_mode = CTRL_MODE_CMD_CAN;
  else if (serial_connected_)
    m_msg.body.motion_control_msg.data.cmd.control_mode = CTRL_MODE_CMD_UART;

  motion_cmd_mutex_.lock();
  m_msg.body.motion_control_msg.data.cmd.fault_clear_flag =
      static_cast<uint8_t>(current_motion_cmd_.fault_clear_flag);
  m_msg.body.motion_control_msg.data.cmd.linear_velocity_cmd =
      current_motion_cmd_.linear_velocity;
  m_msg.body.motion_control_msg.data.cmd.angular_velocity_cmd =
      current_motion_cmd_.angular_velocity;
  motion_cmd_mutex_.unlock();

  m_msg.body.motion_control_msg.data.cmd.reserved0 = 0;
  m_msg.body.motion_control_msg.data.cmd.reserved1 = 0;
  m_msg.body.motion_control_msg.data.cmd.count = count;

  if (can_connected_)
    m_msg.body.motion_control_msg.data.cmd.checksum =
        CalcScoutCANChecksum(CAN_MSG_MOTION_CONTROL_CMD_ID,
                             m_msg.body.motion_control_msg.data.raw, 8);
  // serial_connected_: checksum will be calculated later when packed into a
  // complete serial frame

  if (can_connected_) {
    // send to can bus
    can_frame m_frame;
    EncodeScoutMsgToCAN(&m_msg, &m_frame);
    can_if_->SendFrame(m_frame);
  } else {
    // send to serial port
    EncodeScoutMsgToUART(&m_msg, tx_buffer_, &tx_cmd_len_);
    serial_if_->SendBytes(tx_buffer_, tx_cmd_len_);
  }
}

void ScoutBase::SendLightCmd(uint8_t count) {
  ScoutMessage l_msg;
  l_msg.type = ScoutLightControlMsg;

  light_cmd_mutex_.lock();
  if (light_ctrl_enabled_) {
    l_msg.body.light_control_msg.data.cmd.light_ctrl_enable = LIGHT_ENABLE_CTRL;

    l_msg.body.light_control_msg.data.cmd.front_light_mode =
        static_cast<uint8_t>(current_light_cmd_.front_mode);
    l_msg.body.light_control_msg.data.cmd.front_light_custom =
        current_light_cmd_.front_custom_value;
    l_msg.body.light_control_msg.data.cmd.rear_light_mode =
        static_cast<uint8_t>(current_light_cmd_.rear_mode);
    l_msg.body.light_control_msg.data.cmd.rear_light_custom =
        current_light_cmd_.rear_custom_value;

    // std::cout << "cmd: " << l_msg.data.cmd.front_light_mode << " , " <<
    // l_msg.data.cmd.front_light_custom << " , "
    //           << l_msg.data.cmd.rear_light_mode << " , " <<
    //           l_msg.data.cmd.rear_light_custom << std::endl;
    // std::cout << "light cmd generated" << std::endl;
  } else {
    l_msg.body.light_control_msg.data.cmd.light_ctrl_enable =
        LIGHT_DISABLE_CTRL;

    l_msg.body.light_control_msg.data.cmd.front_light_mode =
        LIGHT_MODE_CONST_OFF;
    l_msg.body.light_control_msg.data.cmd.front_light_custom = 0;
    l_msg.body.light_control_msg.data.cmd.rear_light_mode =
        LIGHT_MODE_CONST_OFF;
    l_msg.body.light_control_msg.data.cmd.rear_light_custom = 0;
  }
  light_ctrl_requested_ = false;
  light_cmd_mutex_.unlock();

  l_msg.body.light_control_msg.data.cmd.reserved0 = 0;
  l_msg.body.light_control_msg.data.cmd.count = count;

  if (can_connected_)
    l_msg.body.light_control_msg.data.cmd.checksum = CalcScoutCANChecksum(
        CAN_MSG_LIGHT_CONTROL_CMD_ID, l_msg.body.light_control_msg.data.raw, 8);
  // serial_connected_: checksum will be calculated later when packed into a
  // complete serial frame

  if (can_connected_) {
    // send to can bus
    can_frame l_frame;
    EncodeScoutMsgToCAN(&l_msg, &l_frame);

    can_if_->SendFrame(l_frame);
  } else {
    // send to serial port
    EncodeScoutMsgToUART(&l_msg, tx_buffer_, &tx_cmd_len_);
    serial_if_->SendBytes(tx_buffer_, tx_cmd_len_);
  }

  // std::cout << "cmd: " << static_cast<int>(l_msg.data.cmd.front_light_mode)
  // << " , " << static_cast<int>(l_msg.data.cmd.front_light_custom) << " , "
  //           << static_cast<int>(l_msg.data.cmd.rear_light_mode) << " , " <<
  //           static_cast<int>(l_msg.data.cmd.rear_light_custom) << std::endl;
  // std::cout << "can: ";
  // for (int i = 0; i < 8; ++i)
  //     std::cout << static_cast<int>(l_frame.data[i]) << " ";
  // std::cout << "uart: ";
  // for (int i = 0; i < tx_cmd_len_; ++i)
  //     std::cout << static_cast<int>(tx_buffer_[i]) << " ";
  // std::cout << std::endl;
}

ScoutState ScoutBase::GetScoutState() {
  std::lock_guard<std::mutex> guard(scout_state_mutex_);
  return scout_state_;
}

void ScoutBase::SetMotionCommand(
    double linear_vel, double angular_vel,
    ScoutMotionCmd::FaultClearFlag fault_clr_flag) {
  // make sure cmd thread is started before attempting to send commands
  if (!cmd_thread_started_) StartCmdThread();

  if (linear_vel < ScoutMotionCmd::min_linear_velocity)
    linear_vel = ScoutMotionCmd::min_linear_velocity;
  if (linear_vel > ScoutMotionCmd::max_linear_velocity)
    linear_vel = ScoutMotionCmd::max_linear_velocity;
  if (angular_vel < ScoutMotionCmd::min_angular_velocity)
    angular_vel = ScoutMotionCmd::min_angular_velocity;
  if (angular_vel > ScoutMotionCmd::max_angular_velocity)
    angular_vel = ScoutMotionCmd::max_angular_velocity;

  std::lock_guard<std::mutex> guard(motion_cmd_mutex_);
  current_motion_cmd_.linear_velocity = static_cast<int8_t>(
      linear_vel / ScoutMotionCmd::max_linear_velocity * 100.0);
  current_motion_cmd_.angular_velocity = static_cast<int8_t>(
      angular_vel / ScoutMotionCmd::max_angular_velocity * 100.0);
  current_motion_cmd_.fault_clear_flag = fault_clr_flag;
}

void ScoutBase::SetLightCommand(ScoutLightCmd cmd) {
  if (!cmd_thread_started_) StartCmdThread();

  std::lock_guard<std::mutex> guard(light_cmd_mutex_);
  current_light_cmd_ = cmd;
  light_ctrl_enabled_ = true;
  light_ctrl_requested_ = true;
}

void ScoutBase::DisableLightCmdControl() {
  std::lock_guard<std::mutex> guard(light_cmd_mutex_);
  light_ctrl_enabled_ = false;
  light_ctrl_requested_ = true;
}

void ScoutBase::ParseCANFrame(can_frame *rx_frame) {
  // validate checksum, discard frame if fails
  if (!rx_frame->data[7] == CalcScoutCANChecksum(rx_frame->can_id,
                                                 rx_frame->data,
                                                 rx_frame->can_dlc)) {
    std::cerr << "ERROR: checksum mismatch, discard frame with id "
              << rx_frame->can_id << std::endl;
    return;
  }

  // otherwise, update robot state with new frame
  ScoutMessage status_msg;
  DecodeScoutMsgFromCAN(rx_frame, &status_msg);
  NewStatusMsgReceivedCallback(status_msg);
}

void ScoutBase::ParseUARTBuffer(uint8_t *buf, const size_t bufsize,
                                size_t bytes_received) {
  // std::cout << "bytes received from serial: " << bytes_received << std::endl;
  // serial_parser_.PrintStatistics();
  // serial_parser_.ParseBuffer(buf, bytes_received);
  ScoutMessage status_msg;
  for (int i = 0; i < bytes_received; ++i) {
    if (DecodeScoutMsgFromUART(buf[i], &status_msg))
      NewStatusMsgReceivedCallback(status_msg);
  }
}

void ScoutBase::NewStatusMsgReceivedCallback(const ScoutMessage &msg) {
  // std::cout << "new status msg received" << std::endl;
  std::lock_guard<std::mutex> guard(scout_state_mutex_);
  UpdateScoutState(msg, scout_state_);
}

void ScoutBase::UpdateScoutState(const ScoutMessage &status_msg,
                                 ScoutState &state) {
  switch (status_msg.type) {
    case ScoutMotionStatusMsg: {
      // std::cout << "motion control feedback received" << std::endl;
      const MotionStatusMessage &msg = status_msg.body.motion_status_msg;
      state.linear_velocity =
          static_cast<int16_t>(
              static_cast<uint16_t>(msg.data.status.linear_velocity.low_byte) |
              static_cast<uint16_t>(msg.data.status.linear_velocity.high_byte)
                  << 8) /
          1000.0;
      state.angular_velocity =
          static_cast<int16_t>(
              static_cast<uint16_t>(msg.data.status.angular_velocity.low_byte) |
              static_cast<uint16_t>(msg.data.status.angular_velocity.high_byte)
                  << 8) /
          1000.0;
      break;
    }
    case ScoutLightStatusMsg: {
      // std::cout << "light control feedback received" << std::endl;
      const LightStatusMessage &msg = status_msg.body.light_status_msg;
      if (msg.data.status.light_ctrl_enable == LIGHT_DISABLE_CTRL)
        state.light_control_enabled = false;
      else
        state.light_control_enabled = true;
      state.front_light_state.mode = msg.data.status.front_light_mode;
      state.front_light_state.custom_value = msg.data.status.front_light_custom;
      state.rear_light_state.mode = msg.data.status.rear_light_mode;
      state.rear_light_state.custom_value = msg.data.status.rear_light_custom;
      break;
    }
    case ScoutSystemStatusMsg: {
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
    case ScoutMotorDriverStatusMsg: {
      // std::cout << "motor 1 driver feedback received" << std::endl;
      const MotorDriverStatusMessage &msg =
          status_msg.body.motor_driver_status_msg;
      for (int i = 0; i < ScoutState::motor_num; ++i) {
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
  }
}
}  // namespace westonrobot
