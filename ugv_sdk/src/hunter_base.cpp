#include "ugv_sdk/hunter/hunter_base.hpp"

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

namespace westonrobot {

void HunterBase::SendRobotCmd() {
  static uint8_t cmd_count = 0;
  SendModeCtl();
  SetParkMode();
  SendMotionCmd(cmd_count++);
}

void HunterBase::SendMotionCmd(uint8_t count) {
  // motion control message
  HunterMessage m_msg;
  m_msg.type = HunterMotionControlMsg;
  /*if (can_connected_)
    m_msg.body.motion_control_msg.data.cmd.control_mode = CTRL_MODE_CMD_CAN;
  else if (serial_connected_)
    m_msg.body.motion_cmd_msg.data.cmd.control_mode = CTRL_MODE_CMD_UART*/
  ;
  motion_cmd_mutex_.lock();
  m_msg.body.motion_control_msg.data.cmd.linear_velocity_cmd.high_byte =
      current_motion_cmd_.linear_velocity_height_byte;
  m_msg.body.motion_control_msg.data.cmd.linear_velocity_cmd.low_byte =
      current_motion_cmd_.linear_velocity_low_byte;
  m_msg.body.motion_control_msg.data.cmd.angular_velocity_cmd.high_byte =
      current_motion_cmd_.angular_velocity_height_byte;
  m_msg.body.motion_control_msg.data.cmd.angular_velocity_cmd.low_byte =
      current_motion_cmd_.angular_velocity_low_byte;
  motion_cmd_mutex_.unlock();

  m_msg.body.motion_control_msg.data.cmd.reserved0 = 0;
  m_msg.body.motion_control_msg.data.cmd.reserved1 = 0;
  m_msg.body.motion_control_msg.data.cmd.reserved2 = 0;
  m_msg.body.motion_control_msg.data.cmd.reserved3 = 0;

  // send to can bus
  if (can_connected_) {
    can_frame m_frame;
    EncodeHunterMsgToCAN(&m_msg, &m_frame);
    can_if_->SendFrame(m_frame);
  }
}

HunterState HunterBase::GetHunterState() {
  std::lock_guard<std::mutex> guard(hunter_state_mutex_);
  return hunter_state_;
}

void HunterBase::SetMotionCommand(
    double linear_vel, double angular_vel, double steering_angle,
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
  current_motion_cmd_.linear_velocity_height_byte =
      static_cast<int16_t>(linear_vel * 1000) >> 8;
  current_motion_cmd_.linear_velocity_low_byte =
      static_cast<int16_t>(linear_vel * 1000) & 0xff;
  current_motion_cmd_.angular_velocity_height_byte =
      static_cast<int16_t>(angular_vel * 1000) >> 8;
  current_motion_cmd_.angular_velocity_low_byte =
      static_cast<int16_t>(angular_vel * 1000) & 0xff;
  current_motion_cmd_.fault_clear_flag = fault_clr_flag;

  FeedCmdTimeoutWatchdog();
}

void HunterBase::SendModeCtl() {
  HunterMessage m_msg;
  m_msg.type = HunterControlModeMsg;
  mode_cmd_mutex_.lock();
  m_msg.body.mode_cmd_msg.data.cmd.mode_control = 0x01;
  mode_cmd_mutex_.unlock();
  m_msg.body.mode_cmd_msg.data.cmd.reserved0 = 0;
  m_msg.body.mode_cmd_msg.data.cmd.reserved1 = 0;
  m_msg.body.mode_cmd_msg.data.cmd.reserved2 = 0;
  m_msg.body.mode_cmd_msg.data.cmd.reserved3 = 0;
  m_msg.body.mode_cmd_msg.data.cmd.reserved4 = 0;
  m_msg.body.mode_cmd_msg.data.cmd.reserved5 = 0;
  m_msg.body.mode_cmd_msg.data.cmd.reserved6 = 0;
  if (can_connected_) {
    // send to can bus
    can_frame m_frame;
    EncodeHunterMsgToCAN(&m_msg, &m_frame);
    can_if_->SendFrame(m_frame);
  } else {
  }
}

void HunterBase::SetParkMode() {
  HunterMessage m_msg;
  m_msg.type = HunterParkControlMsg;
  bool flag = current_motion_cmd_.linear_velocity_height_byte ||
              current_motion_cmd_.linear_velocity_low_byte;
  if (flag) {
    pack_mode_cmd_mutex_.lock();
    m_msg.body.park_control_msg.data.cmd.packing_mode = 0x00;
    pack_mode_cmd_mutex_.unlock();
  } else {
    pack_mode_cmd_mutex_.lock();
    m_msg.body.park_control_msg.data.cmd.packing_mode = 0x01;
    pack_mode_cmd_mutex_.unlock();
  }
  m_msg.body.park_control_msg.data.cmd.reserved0 = 0;
  m_msg.body.park_control_msg.data.cmd.reserved1 = 0;
  m_msg.body.park_control_msg.data.cmd.reserved2 = 0;
  m_msg.body.park_control_msg.data.cmd.reserved3 = 0;
  m_msg.body.park_control_msg.data.cmd.reserved4 = 0;
  m_msg.body.park_control_msg.data.cmd.reserved5 = 0;
  m_msg.body.park_control_msg.data.cmd.reserved6 = 0;
  if (can_connected_) {
    // send to can bus
    can_frame m_frame;
    EncodeHunterMsgToCAN(&m_msg, &m_frame);
    can_if_->SendFrame(m_frame);
  } else {
  }
}

void HunterBase::ParseCANFrame(can_frame *rx_frame) {
  // update robot state with new frame
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
      state.park_mode = msg.data.status.park_mode;
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
    case HunterMotorDriverHeightSpeedStatusMsg: {
      // std::cout << "motor driver height speed feedback received" <<
      // std::endl;
      const MotorDriverHeightSpeedStatusMessage &msg =
          status_msg.body.motor_driver_height_speed_status_msg;
      for (int i = 0; i < HunterState::motor_num; ++i) {
        state.motor_hs_state[msg.motor_id].current =
            (static_cast<uint16_t>(msg.data.status.current.low_byte) |
             static_cast<uint16_t>(msg.data.status.current.high_byte) << 8) /
            10.0;
        state.motor_hs_state[msg.motor_id].rpm = static_cast<int16_t>(
            static_cast<uint16_t>(msg.data.status.rpm.low_byte) |
            static_cast<uint16_t>(msg.data.status.rpm.high_byte) << 8);
        state.motor_hs_state[msg.motor_id].motor_pose = static_cast<int32_t>(
            static_cast<uint32_t>(msg.data.status.moter_pose.lowest) |
            static_cast<uint32_t>(msg.data.status.moter_pose.sec_lowest) << 8 |
            static_cast<uint32_t>(msg.data.status.moter_pose.sec_heighest)
                << 16 |
            static_cast<uint32_t>(msg.data.status.moter_pose.heighest) << 24);
      }
      break;
    }
    case HunterMotorDriverLowSpeedStatusMsg: {
      // std::cout << "motor driver low speed feedback received" << std::endl;
      const MotorDriverLowSpeedStatusMessage &msg =
          status_msg.body.motor_driver_low_speed_status_msg;
      for (int i = 0; i < HunterState::motor_num; ++i) {
        state.motor_ls_state[msg.motor_id].driver_voltage =
            (static_cast<uint16_t>(msg.data.status.driver_voltage.low_byte) |
             static_cast<uint16_t>(msg.data.status.driver_voltage.high_byte)
                 << 8) /
            10.0;
        state.motor_ls_state[msg.motor_id]
            .driver_temperature = static_cast<int16_t>(
            static_cast<uint16_t>(msg.data.status.driver_temperature.low_byte) |
            static_cast<uint16_t>(msg.data.status.driver_temperature.high_byte)
                << 8);
        state.motor_ls_state[msg.motor_id].motor_temperature =
            msg.data.status.motor_temperature;
        state.motor_ls_state[msg.motor_id].driver_state =
            msg.data.status.driver_status;
      }
    }
    default:
      break;
  }
}
}  // namespace westonrobot
