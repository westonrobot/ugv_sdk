#include "ugv_sdk/tracer/tracer_base.hpp"

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
void TracerBase::SendRobotCmd() {
  static uint8_t cmd_count = 0;
  static uint8_t light_cmd_count = 0;
  if (can_connected_) {
    SendMotionCmd(cmd_count++);
    SendLightCmd(light_cmd_count++);
    // if (light_ctrl_requested_)
    //   SendLightCmd(light_cmd_count++);
    // else
    //   std::cout << "not updating light cmd" << std::endl;
  }
}

void TracerBase::EnableCommandedMode() {
  AgxMessage c_msg;
  c_msg.type = AgxMsgCtrlModeSelect;
  memset(c_msg.body.ctrl_mode_select_msg.raw, 0, 8);
  c_msg.body.ctrl_mode_select_msg.cmd.control_mode = CTRL_MODE_CMD_CAN;

  // send to can bus
  can_frame c_frame;
  EncodeCanFrame(&c_msg, &c_frame);
  can_if_->SendFrame(c_frame);
}

void TracerBase::SendMotionCmd(uint8_t count) {
  // motion control message
  AgxMessage m_msg;
  m_msg.type = AgxMsgMotionCommand;
  memset(m_msg.body.motion_command_msg.raw, 0, 8);

  motion_cmd_mutex_.lock();
  int16_t linear_cmd =
      static_cast<int16_t>(current_motion_cmd_.linear_velocity) * 1000;
  int16_t angular_cmd =
      static_cast<int16_t>(current_motion_cmd_.angular_velocity) * 1000;
  motion_cmd_mutex_.unlock();

  // SendControlCmd();
  m_msg.body.motion_command_msg.cmd.linear_velocity.high_byte =
      (static_cast<uint16_t>(linear_cmd) >> 8) & 0x00ff;
  m_msg.body.motion_command_msg.cmd.linear_velocity.low_byte =
      (static_cast<uint16_t>(linear_cmd) >> 0) & 0x00ff;
  m_msg.body.motion_command_msg.cmd.angular_velocity.high_byte =
      (static_cast<uint16_t>(angular_cmd) >> 8) & 0x00ff;
  m_msg.body.motion_command_msg.cmd.angular_velocity.low_byte =
      (static_cast<uint16_t>(angular_cmd) >> 0) & 0x00ff;

  // send to can bus
  can_frame m_frame;
  EncodeCanFrame(&m_msg, &m_frame);
  can_if_->SendFrame(m_frame);
}

void TracerBase::SendLightCmd(uint8_t count) {
  AgxMessage l_msg;
  l_msg.type = AgxMsgLightCommand;
  memset(l_msg.body.light_command_msg.raw, 0, 8);

  light_cmd_mutex_.lock();
  if (light_ctrl_enabled_) {
    l_msg.body.light_command_msg.cmd.light_ctrl_enabled = LIGHT_CTRL_ENABLE;

    l_msg.body.light_command_msg.cmd.front_light_mode =
        static_cast<uint8_t>(current_light_cmd_.front_mode);
    l_msg.body.light_command_msg.cmd.front_light_custom =
        current_light_cmd_.front_custom_value;
    l_msg.body.light_command_msg.cmd.rear_light_mode =
        static_cast<uint8_t>(current_light_cmd_.rear_mode);
    l_msg.body.light_command_msg.cmd.rear_light_custom =
        current_light_cmd_.rear_custom_value;
  } else {
    l_msg.body.light_command_msg.cmd.light_ctrl_enabled = LIGHT_CTRL_DISABLE;
  }
  light_ctrl_requested_ = false;
  light_cmd_mutex_.unlock();

  l_msg.body.light_command_msg.cmd.count = count;

  // send to can bus
  can_frame l_frame;
  EncodeCanFrame(&l_msg, &l_frame);
  can_if_->SendFrame(l_frame);
}

TracerState TracerBase::GetTracerState() {
  std::lock_guard<std::mutex> guard(tracer_state_mutex_);
  return tracer_state_;
}

void TracerBase::SetMotionCommand(double linear_vel, double angular_vel) {
  // make sure cmd thread is started before attempting to send commands
  if (!cmd_thread_started_) StartCmdThread();

  if (linear_vel < TracerMotionCmd::min_linear_velocity)
    linear_vel = TracerMotionCmd::min_linear_velocity;
  if (linear_vel > TracerMotionCmd::max_linear_velocity)
    linear_vel = TracerMotionCmd::max_linear_velocity;
  if (angular_vel < TracerMotionCmd::min_angular_velocity)
    angular_vel = TracerMotionCmd::min_angular_velocity;
  if (angular_vel > TracerMotionCmd::max_angular_velocity)
    angular_vel = TracerMotionCmd::max_angular_velocity;

  std::lock_guard<std::mutex> guard(motion_cmd_mutex_);
  current_motion_cmd_.linear_velocity = linear_vel;
  current_motion_cmd_.angular_velocity = angular_vel;

  FeedCmdTimeoutWatchdog();
}

void TracerBase::SetLightCommand(TracerLightCmd cmd) {
  if (!cmd_thread_started_) StartCmdThread();

  std::lock_guard<std::mutex> guard(light_cmd_mutex_);
  current_light_cmd_ = cmd;
  light_ctrl_enabled_ = true;
  light_ctrl_requested_ = true;
}

void TracerBase::DisableLightCmdControl() {
  std::lock_guard<std::mutex> guard(light_cmd_mutex_);
  light_ctrl_enabled_ = false;
  light_ctrl_requested_ = true;
}

void TracerBase::ParseCANFrame(can_frame *rx_frame) {
  AgxMessage status_msg;
  DecodeCanFrame(rx_frame, &status_msg);
  NewStatusMsgReceivedCallback(status_msg);
}

void TracerBase::NewStatusMsgReceivedCallback(const AgxMessage &msg) {
  // std::cout << "new status msg received" << std::endl;
  std::lock_guard<std::mutex> guard(tracer_state_mutex_);
  UpdateTracerState(msg, tracer_state_);
}

void TracerBase::UpdateTracerState(const AgxMessage &status_msg,
                                   TracerState &state) {
  switch (status_msg.type) {
    case AgxMsgSystemState: {
      // std::cout << "system status feedback received" << std::endl;
      const SystemStateMessage &msg = status_msg.body.system_state_msg;
      state.control_mode = msg.state.control_mode;
      state.base_state = msg.state.vehicle_state;
      state.battery_voltage =
          (static_cast<uint16_t>(msg.state.battery_voltage.low_byte) |
           static_cast<uint16_t>(msg.state.battery_voltage.high_byte) << 8) /
          10.0;
      state.fault_code = msg.state.fault_code;
      break;
    }
    case AgxMsgMotionState: {
      // std::cout << "motion control feedback received" << std::endl;
      const MotionStateMessage &msg = status_msg.body.motion_state_msg;
      state.linear_velocity =
          static_cast<int16_t>(
              static_cast<uint16_t>(msg.state.linear_velocity.low_byte) |
              static_cast<uint16_t>(msg.state.linear_velocity.high_byte) << 8) /
          1000.0;
      state.angular_velocity =
          static_cast<int16_t>(
              static_cast<uint16_t>(msg.state.angular_velocity.low_byte) |
              static_cast<uint16_t>(msg.state.angular_velocity.high_byte)
                  << 8) /
          1000.0;
      break;
    }
    case AgxMsgLightState: {
      // std::cout << "light control feedback received" << std::endl;
      const LightStateMessage &msg = status_msg.body.light_state_msg;
      if (msg.state.light_ctrl_enabled == LIGHT_CTRL_DISABLE)
        state.light_control_enabled = false;
      else
        state.light_control_enabled = true;
      state.front_light_state.mode = msg.state.front_light_mode;
      state.front_light_state.custom_value = msg.state.front_light_custom;
      break;
    }
    case AgxMsgActuatorHSState: {
      // std::cout << "actuator hs feedback received" << std::endl;
      const ActuatorHSStateMessage &msg = status_msg.body.actuator_hs_state_msg;
      state.actuator_states[msg.motor_id].motor_current =
          (static_cast<uint16_t>(msg.data.state.current.low_byte) |
           static_cast<uint16_t>(msg.data.state.current.high_byte) << 8) /
          10.0;
      state.actuator_states[msg.motor_id].motor_rpm = static_cast<int16_t>(
          static_cast<uint16_t>(msg.data.state.rpm.low_byte) |
          static_cast<uint16_t>(msg.data.state.rpm.high_byte) << 8);
      state.actuator_states[msg.motor_id].motor_pulses = static_cast<int16_t>(
          static_cast<uint16_t>(msg.data.state.pulse_count.low_byte) |
          static_cast<uint16_t>(msg.data.state.pulse_count.high_byte) << 8);
      break;
    }
    case AgxMsgActuatorLSState: {
      // std::cout << "actuator ls feedback received" << std::endl;
      const ActuatorLSStateMessage &msg = status_msg.body.actuator_ls_state_msg;
      for (int i = 0; i < 2; ++i) {
        state.actuator_states[msg.motor_id].driver_voltage =
            (static_cast<uint16_t>(msg.data.state.driver_voltage.low_byte) |
             static_cast<uint16_t>(msg.data.state.driver_voltage.high_byte)
                 << 8) /
            10.0;
        state.actuator_states[msg.motor_id]
            .driver_temperature = static_cast<int16_t>(
            static_cast<uint16_t>(msg.data.state.driver_temperature.low_byte) |
            static_cast<uint16_t>(msg.data.state.driver_temperature.high_byte)
                << 8);
        state.actuator_states[msg.motor_id].motor_temperature =
            msg.data.state.motor_temperature;
        state.actuator_states[msg.motor_id].driver_state =
            msg.data.state.driver_state;
      }
      break;
    }
    case AgxMsgOdometry: {
      // std::cout << "Odometer msg feedback received" << std::endl;
      const OdometryMessage &msg = status_msg.body.odometry_msg;
      state.right_odometry = static_cast<int32_t>(
          (static_cast<uint32_t>(msg.state.right_wheel.lsb)) |
          (static_cast<uint32_t>(msg.state.right_wheel.low_byte) << 8) |
          (static_cast<uint32_t>(msg.state.right_wheel.high_byte) << 16) |
          (static_cast<uint32_t>(msg.state.right_wheel.msb) << 24));
      state.left_odometry = static_cast<int32_t>(
          (static_cast<uint32_t>(msg.state.left_wheel.lsb)) |
          (static_cast<uint32_t>(msg.state.left_wheel.low_byte) << 8) |
          (static_cast<uint32_t>(msg.state.left_wheel.high_byte) << 16) |
          (static_cast<uint32_t>(msg.state.left_wheel.msb) << 24));
    }
  }
}
}  // namespace westonrobot
