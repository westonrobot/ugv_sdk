#include "ugv_sdk/bunker/bunker_base.hpp"

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

namespace westonrobot {
void BunkerBase::SendRobotCmd() {
  static uint8_t cmd_count = 0;
  static uint8_t light_cmd_count = 0;
  SendMotionCmd(cmd_count++);
}

void BunkerBase::SendMotionCmd(uint8_t count) {
  // motion control message
  BunkerMessage m_msg;
  m_msg.type = BunkerMotionControlMsg;

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
        CalcBunkerCANChecksum(CAN_MSG_MOTION_CONTROL_CMD_ID,
                             m_msg.body.motion_control_msg.data.raw, 8);
  // serial_connected_: checksum will be calculated later when packed into a
  // complete serial frame

  if (can_connected_) {
    // send to can bus
    can_frame m_frame;
    EncodeBunkerMsgToCAN(&m_msg, &m_frame);
    can_if_->SendFrame(m_frame);
  } else {
    // send to serial port
//    EncodeBunkerMsgToUART(&m_msg, tx_buffer_, &tx_cmd_len_);
//    serial_if_->SendBytes(tx_buffer_, tx_cmd_len_);
  }
}



BunkerState BunkerBase::GetBunkerState() {
  std::lock_guard<std::mutex> guard(bunker_state_mutex_);
  return bunker_state_;
}

void BunkerBase::SetMotionCommand(
    double linear_vel, double angular_vel,
    BunkerMotionCmd::FaultClearFlag fault_clr_flag) {
  // make sure cmd thread is started before attempting to send commands
  if (!cmd_thread_started_) StartCmdThread();

  if (linear_vel < BunkerMotionCmd::min_linear_velocity)
    linear_vel = BunkerMotionCmd::min_linear_velocity;
  if (linear_vel > BunkerMotionCmd::max_linear_velocity)
    linear_vel = BunkerMotionCmd::max_linear_velocity;
  if (angular_vel < BunkerMotionCmd::min_angular_velocity)
    angular_vel = BunkerMotionCmd::min_angular_velocity;
  if (angular_vel > BunkerMotionCmd::max_angular_velocity)
    angular_vel = BunkerMotionCmd::max_angular_velocity;
  //std::cout<<angular_vel<<std::endl;
  std::lock_guard<std::mutex> guard(motion_cmd_mutex_);
  current_motion_cmd_.linear_velocity = static_cast<int8_t>(
      linear_vel / BunkerMotionCmd::max_linear_velocity * 100.0);
  current_motion_cmd_.angular_velocity = static_cast<int8_t>(
      angular_vel / BunkerMotionCmd::max_angular_velocity * 100.0);
  current_motion_cmd_.fault_clear_flag = fault_clr_flag;
  FeedCmdTimeoutWatchdog();
}



void BunkerBase::ParseCANFrame(can_frame *rx_frame) {
  // validate checksum, discard frame if fails
  if (!rx_frame->data[7] == CalcBunkerCANChecksum(rx_frame->can_id,
                                                 rx_frame->data,
                                                 rx_frame->can_dlc)) {
    std::cerr << "ERROR: checksum mismatch, discard frame with id "
              << rx_frame->can_id << std::endl;
    return;
  }

  // otherwise, update robot state with new frame
  BunkerMessage status_msg;
  DecodeBunkerMsgFromCAN(rx_frame, &status_msg);
  NewStatusMsgReceivedCallback(status_msg);
}

//void BunkerBase::ParseUARTBuffer(uint8_t *buf, const size_t bufsize,
//                                size_t bytes_received) {
//  // std::cout << "bytes received from serial: " << bytes_received << std::endl;
//  // serial_parser_.PrintStatistics();
//  // serial_parser_.ParseBuffer(buf, bytes_received);
//  BunkerMessage status_msg;
//  for (int i = 0; i < bytes_received; ++i) {
//    if (DecodeBunkerMsgFromUART(buf[i], &status_msg))
//      NewStatusMsgReceivedCallback(status_msg);
//  }
//}

void BunkerBase::NewStatusMsgReceivedCallback(const BunkerMessage &msg) {
  // std::cout << "new status msg received" << std::endl;
  std::lock_guard<std::mutex> guard(bunker_state_mutex_);
  UpdateBunkerState(msg, bunker_state_);
}

void BunkerBase::UpdateBunkerState(const BunkerMessage &status_msg,
                                 BunkerState &state) {
  switch (status_msg.type) {
    case BunkerMotionStatusMsg: {
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
    case BunkerLightStatusMsg: {
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
    case BunkerSystemStatusMsg: {
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
    case BunkerMotorDriverStatusMsg: {
      // std::cout << "motor 1 driver feedback received" << std::endl;
      const MotorDriverStatusMessage &msg =
          status_msg.body.motor_driver_status_msg;
      for (int i = 0; i < BunkerState::motor_num; ++i) {
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
