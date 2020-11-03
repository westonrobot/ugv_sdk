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

namespace westonrobot
{
void TracerBase::SendRobotCmd() {
  static uint8_t cmd_count = 0;
  static uint8_t light_cmd_count = 0;
  if(can_connected_)  SendControlCmd();
  SendMotionCmd(cmd_count++);
  if (light_ctrl_requested_) SendLightCmd(light_cmd_count++);
}


void TracerBase::SendMotionCmd(uint8_t count)
{
    if(can_connected_)
    {
        // motion control message
        TracerMessage m_msg;
        m_msg.type = TracerMotionCmdMsg;

        //SendControlCmd();
        motion_cmd_mutex_.lock();
        m_msg.body.motion_cmd_msg.data.cmd.linear_velocity.H_byte = current_motion_cmd_.linear_velocity_H;
        m_msg.body.motion_cmd_msg.data.cmd.linear_velocity.L_byte = current_motion_cmd_.linear_velocity_L;
        m_msg.body.motion_cmd_msg.data.cmd.angular_velocity.H_byte = current_motion_cmd_.angular_velocity_H;
        m_msg.body.motion_cmd_msg.data.cmd.angular_velocity.L_byte = current_motion_cmd_.angular_velocity_L;
        motion_cmd_mutex_.unlock();
        m_msg.body.motion_cmd_msg.data.cmd.reserved0 = 0;
        m_msg.body.motion_cmd_msg.data.cmd.reserved1 = 0;
        m_msg.body.motion_cmd_msg.data.cmd.reserved2 = 0;
        m_msg.body.motion_cmd_msg.data.cmd.reserved3 = 0;

        if (can_connected_)
        {
            // send to can bus
            can_frame m_frame;
            EncodeTracerMsgToCAN(&m_msg, &m_frame);
            can_if_->SendFrame(m_frame);
        }
//        else
//        {
//            // TODO
//            // send to serial port
//            // EncodeTracerMsgToUART(&m_msg, tx_buffer_, &tx_cmd_len_);
//            // serial_if_->send_bytes(tx_buffer_, tx_cmd_len_);
//        }
//        if (can_connected_)
//            m_msg.body.motion_cmd_msg.data.cmd.checksum = CalcTracerCANChecksum(CAN_MSG_MOTION_CMD_ID, m_msg.body.motion_cmd_msg.data.raw, 8);
//         serial_connected_: checksum will be calculated later when packed into a complete serial frame

    }
    else if (serial_connected_){
        UartTracerMessage m_msg;
        m_msg.type = UartTracerMotionControlMsg;
        m_msg.body.motion_control_msg.data.cmd.control_mode = CTRL_MODE_CMD_UART;

        //SendControlCmd();
        motion_cmd_mutex_.lock();
        m_msg.body.motion_control_msg.data.cmd.fault_clear_flag =
            static_cast<uint8_t>(uart_current_motion_cmd_.fault_clear_flag);
        m_msg.body.motion_control_msg.data.cmd.linear_velocity_cmd =
            uart_current_motion_cmd_.linear_velocity;
        m_msg.body.motion_control_msg.data.cmd.angular_velocity_cmd =
            uart_current_motion_cmd_.angular_velocity;
        motion_cmd_mutex_.unlock();

        m_msg.body.motion_control_msg.data.cmd.reserved0 = 0;
        m_msg.body.motion_control_msg.data.cmd.reserved1 = 0;
        m_msg.body.motion_control_msg.data.cmd.count = count;
//        serial_connected_: checksum will be calculated later when packed into a
//        complete serial frame
        EncodeTracerMsgToUART(&m_msg, tx_buffer_, &tx_cmd_len_);
        serial_if_->SendBytes(tx_buffer_, tx_cmd_len_);
    }
}

void TracerBase::SendLightCmd(uint8_t count)
{
    UartTracerMessage l_msg;
    l_msg.type = UartTracerLightControlMsg;

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

    // serial_connected_: checksum will be calculated later when packed into a
    // complete serial frame
    // send to serial port
    EncodeTracerMsgToUART(&l_msg, tx_buffer_, &tx_cmd_len_);
    serial_if_->SendBytes(tx_buffer_, tx_cmd_len_);

    /*TracerMessage l_msg;
    l_msg.type = TracerLightControlMsg;

    light_cmd_mutex_.lock();
    if (light_ctrl_enabled_)
    {
        l_msg.body.light_control_msg.data.cmd.light_ctrl_enable = LIGHT_ENABLE_CTRL;

        l_msg.body.light_control_msg.data.cmd.front_light_mode = static_cast<uint8_t>(current_light_cmd_.front_mode);
        l_msg.body.light_control_msg.data.cmd.front_light_custom = current_light_cmd_.front_custom_value;
        //l_msg.body.light_control_msg.data.cmd.rear_light_mode = static_cast<uint8_t>(current_light_cmd_.rear_mode);
        //l_msg.body.light_control_msg.data.cmd.rear_light_custom = current_light_cmd_.rear_custom_value;

        // std::cout << "cmd: " << l_msg.data.cmd.front_light_mode << " , " << l_msg.data.cmd.front_light_custom << " , "
        //           << l_msg.data.cmd.rear_light_mode << " , " << l_msg.data.cmd.rear_light_custom << std::endl;
        // std::cout << "light cmd generated" << std::endl;
    }
    else
    {
        l_msg.body.light_control_msg.data.cmd.light_ctrl_enable = LIGHT_DISABLE_CTRL;

        l_msg.body.light_control_msg.data.cmd.front_light_mode = LIGHT_MODE_CONST_OFF;
        l_msg.body.light_control_msg.data.cmd.front_light_custom = 0;
        //l_msg.body.light_control_msg.data.cmd.rear_light_mode = LIGHT_MODE_CONST_OFF;
        //l_msg.body.light_control_msg.data.cmd.rear_light_custom = 0;
    }
    light_ctrl_requested_ = false;
    light_cmd_mutex_.unlock();

    l_msg.body.light_control_msg.data.cmd.reserved0 = 0;
    l_msg.body.light_control_msg.data.cmd.count = count;

    if (can_connected_)
       // l_msg.body.light_control_msg.data.cmd.checksum = CalcTracerCANChecksum(CAN_MSG_LIGHT_CONTROL_CMD_ID, l_msg.body.light_control_msg.data.raw, 8);
    // serial_connected_: checksum will be calculated later when packed into a complete serial frame

    if (can_connected_)
    {
        // send to can bus
        can_frame l_frame;
        EncodeTracerMsgToCAN(&l_msg, &l_frame);

        can_if_->SendFrame(l_frame);
    }
    // else
    // {
    //     // send to serial port
    //     EncodeTracerMsgToUART(&l_msg, tx_buffer_, &tx_cmd_len_);
    //     serial_if_->send_bytes(tx_buffer_, tx_cmd_len_);
    // }

    // std::cout << "cmd: " << static_cast<int>(l_msg.data.cmd.front_light_mode) << " , " << static_cast<int>(l_msg.data.cmd.front_light_custom) << " , "
    //           << static_cast<int>(l_msg.data.cmd.rear_light_mode) << " , " << static_cast<int>(l_msg.data.cmd.rear_light_custom) << std::endl;
    // std::cout << "can: ";
    // for (int i = 0; i < 8; ++i)
    //     std::cout << static_cast<int>(l_frame.data[i]) << " ";
    // std::cout << "uart: ";
    // for (int i = 0; i < tx_cmd_len_; ++i)
    //     std::cout << static_cast<int>(tx_buffer_[i]) << " ";
    // std::cout << std::endl;*/
}

void TracerBase::SendControlCmd()
{
      TracerMessage c_msg;
      c_msg.type = TracerModeControlMsg;

      mode_cmd_mutex_.lock();
      c_msg.body.mode_cmd_msg.data.cmd.control_mode=0x01;
      mode_cmd_mutex_.unlock();
      c_msg.body.mode_cmd_msg.data.cmd.reserved0=0;
      c_msg.body.mode_cmd_msg.data.cmd.reserved1=0;
      c_msg.body.mode_cmd_msg.data.cmd.reserved2=0;
      c_msg.body.mode_cmd_msg.data.cmd.reserved3=0;
      c_msg.body.mode_cmd_msg.data.cmd.reserved4=0;
      c_msg.body.mode_cmd_msg.data.cmd.reserved5=0;
      c_msg.body.mode_cmd_msg.data.cmd.reserved6=0;
      if (can_connected_)
      {
          // send to can bus
          can_frame c_frame;
          EncodeTracerMsgToCAN(&c_msg, &c_frame);
          can_if_->SendFrame(c_frame);
      }
      else
      {
          // TODO
          // send to serial port
          // EncodeTracerMsgToUART(&m_msg, tx_buffer_, &tx_cmd_len_);
          // serial_if_->send_bytes(tx_buffer_, tx_cmd_len_);
      }

}

TracerState TracerBase::GetTracerState()
{
    std::lock_guard<std::mutex> guard(tracer_state_mutex_);
    return tracer_state_;
}

UartTracerState TracerBase::GetUartTracerState()
{
    std::lock_guard<std::mutex> guard(uart_tracer_state_mutex_);
    return uart_tracer_state_;
}

void TracerBase::SetMotionCommand(double linear_vel, double angular_vel, TracerMotionCmd::FaultClearFlag fault_clr_flag)
{
    // make sure cmd thread is started before attempting to send commands
    if (!cmd_thread_started_) StartCmdThread();
    if(can_connected_)
    {
        if (linear_vel < TracerMotionCmd::min_linear_velocity)
            linear_vel = TracerMotionCmd::min_linear_velocity;
        if (linear_vel > TracerMotionCmd::max_linear_velocity)
            linear_vel = TracerMotionCmd::max_linear_velocity;
        if (angular_vel < TracerMotionCmd::min_angular_velocity)
            angular_vel = TracerMotionCmd::min_angular_velocity;
        if (angular_vel > TracerMotionCmd::max_angular_velocity)
            angular_vel = TracerMotionCmd::max_angular_velocity;

        std::lock_guard<std::mutex> guard(motion_cmd_mutex_);
        current_motion_cmd_.linear_velocity_H = static_cast<int16_t>(linear_vel*1000)>>8;
        current_motion_cmd_.linear_velocity_L = static_cast<int16_t>(linear_vel*1000)&0xff;
        current_motion_cmd_.angular_velocity_H = static_cast<int16_t>(angular_vel*1000)>>8;
        current_motion_cmd_.angular_velocity_L = static_cast<int16_t>(angular_vel*1000)&0xff;
        current_motion_cmd_.fault_clear_flag = fault_clr_flag;
    }
    else
    {
        if (linear_vel < UartTracerMotionCmd::min_linear_velocity)
            linear_vel = UartTracerMotionCmd::min_linear_velocity;
        if (linear_vel > UartTracerMotionCmd::max_linear_velocity)
            linear_vel = UartTracerMotionCmd::max_linear_velocity;
        if (angular_vel < UartTracerMotionCmd::min_angular_velocity)
            angular_vel = UartTracerMotionCmd::min_angular_velocity;
        if (angular_vel > UartTracerMotionCmd::max_angular_velocity)
            angular_vel = UartTracerMotionCmd::max_angular_velocity;

        std::lock_guard<std::mutex> guard(motion_cmd_mutex_);
        uart_current_motion_cmd_.linear_velocity = static_cast<int8_t>(
            linear_vel / UartTracerMotionCmd::max_linear_velocity * 100.0);
        uart_current_motion_cmd_.angular_velocity = static_cast<int8_t>(
            angular_vel / UartTracerMotionCmd::max_angular_velocity * 100.0);
        //uart_current_motion_cmd_.fault_clear_flag = FaultClearFlag::NO_FAULT;
    //std::cout<<"linear_vel:"<<linear_vel<<std::endl;
    }
    FeedCmdTimeoutWatchdog();
}

void TracerBase::SetLightCommand(TracerLightCmd cmd)
{
    if (!cmd_thread_started_)
        StartCmdThread();

    std::lock_guard<std::mutex> guard(light_cmd_mutex_);
    current_light_cmd_ = cmd;
    light_ctrl_enabled_ = true;
    light_ctrl_requested_ = true;
    FeedCmdTimeoutWatchdog();
}

void TracerBase::DisableLightCmdControl()
{
    std::lock_guard<std::mutex> guard(light_cmd_mutex_);
    light_ctrl_enabled_ = false;
    light_ctrl_requested_ = true;
}

void TracerBase::ParseCANFrame(can_frame *rx_frame)
{
    // otherwise, update robot state with new frame
    TracerMessage status_msg;
    DecodeTracerMsgFromCAN(rx_frame, &status_msg);//assigned a value to status_msg from can include status_msg->type
    NewStatusMsgReceivedCallback(status_msg);
}

void TracerBase::ParseUARTBuffer(uint8_t *buf, const size_t bufsize, size_t bytes_received)
{
    // std::cout << "bytes received from serial: " << bytes_received << std::endl;
    // serial_parser_.PrintStatistics();
    // serial_parser_.ParseBuffer(buf, bytes_received);

     UartTracerMessage status_msg;
     for (int i = 0; i < bytes_received; ++i)
     {
         if (DecodeTracerMsgFromUART(buf[i], &status_msg))
             UartNewStatusMsgReceivedCallback(status_msg);
     }
}

void TracerBase::NewStatusMsgReceivedCallback(const TracerMessage &msg)
{
    // std::cout << "new status msg received" << std::endl;
    std::lock_guard<std::mutex> guard(tracer_state_mutex_);
    UpdateTracerState(msg, tracer_state_);
}

void TracerBase::UartNewStatusMsgReceivedCallback(const UartTracerMessage &msg)
{
    // std::cout << "new status msg received" << std::endl;
    std::lock_guard<std::mutex> guard(tracer_state_mutex_);
    UartUpdateTracerState(msg, uart_tracer_state_);
}

void TracerBase::UpdateTracerState(const TracerMessage &status_msg, TracerState &state)
{
    switch (status_msg.type)
    {
    case TracerMotionStatusMsg:
    {
        // std::cout << "motion control feedback received" << std::endl;
        const MotionStatusMessage &msg = status_msg.body.motion_status_msg;
        state.linear_velocity = static_cast<int16_t>(static_cast<uint16_t>(msg.data.status.linear_velocity.low_byte) | static_cast<uint16_t>(msg.data.status.linear_velocity.high_byte) << 8) / 1000.0;
        state.angular_velocity = static_cast<int16_t>(static_cast<uint16_t>(msg.data.status.angular_velocity.low_byte) | static_cast<uint16_t>(msg.data.status.angular_velocity.high_byte) << 8) / 1000.0;
        break;
    }
    case TracerLightStatusMsg:
    {
        // std::cout << "light control feedback received" << std::endl;
        const LightStatusMessage &msg = status_msg.body.light_status_msg;
        if (msg.data.status.light_ctrl_enable == LIGHT_DISABLE_CTRL)
            state.light_control_enabled = false;
        else
            state.light_control_enabled = true;
        state.front_light_state.mode = msg.data.status.front_light_mode;
        state.front_light_state.custom_value = msg.data.status.front_light_custom;
        //state.rear_light_state.mode = msg.data.status.rear_light_mode;
        //state.rear_light_state.custom_value = msg.data.status.rear_light_custom;
        break;
    }
    case TracerSystemStatusMsg:
    {
        // std::cout << "system status feedback received" << std::endl;
        const SystemStatusMessage &msg = status_msg.body.system_status_msg;
        state.control_mode = msg.data.status.control_mode;
        state.base_state = msg.data.status.base_state;
        state.battery_voltage = (static_cast<uint16_t>(msg.data.status.battery_voltage.low_byte) | static_cast<uint16_t>(msg.data.status.battery_voltage.high_byte) << 8) / 10.0;
        state.fault_code = msg.data.status.fault_code;
        break;
    }
    case TracerMotorDriverStatusMsg:
    {
        // std::cout << "motor 1 driver feedback received" << std::endl;
        //const MotorDriverStatusMessage &msg = status_msg.body.motor_driver_status_msg;
        const MotorHeightSpeedStatusMessage &msg = status_msg.body.motor_heigh_speed_msg;
        for (int i = 0; i < 2; ++i)
        {
            //state.motor_states[status_msg.body.motor_driver_status_msg.motor_id].current = (static_cast<uint16_t>(msg.data.status.current.low_byte) | static_cast<uint16_t>(msg.data.status.current.high_byte) << 8) / 10.0;
            state.motor_states[status_msg.body.motor_heigh_speed_msg.motor_id].rpm = static_cast<int16_t>(static_cast<uint16_t>(msg.data.status.rpm.low_byte) | static_cast<uint16_t>(msg.data.status.rpm.high_byte) << 8);
            //state.motor_states[status_msg.body.motor_driver_status_msg.motor_id].temperature = msg.data.status.temperature;
        }
        break;
    }
    case TracerOdometerMsg:
    {
        // std::cout << "Odometer msg feedback received" << std::endl;
        const OdometerMessage &msg = status_msg.body.odom_msg;
        state.right_odomter=static_cast<int32_t>((static_cast<uint32_t>(msg.data.status.rightodometer.lowest))|(static_cast<uint32_t>(msg.data.status.rightodometer.sec_lowest)<<8)|(static_cast<uint32_t>(msg.data.status.rightodometer.sec_highest)<<16)|(static_cast<uint32_t>(msg.data.status.rightodometer.highest)<<24));
        state.left_odomter=static_cast<int32_t>((static_cast<uint32_t>(msg.data.status.leftodometer.lowest))|(static_cast<uint32_t>(msg.data.status.leftodometer.sec_lowest)<<8)|(static_cast<uint32_t>(msg.data.status.leftodometer.sec_highest)<<16)|(static_cast<uint32_t>(msg.data.status.leftodometer.highest)<<24));

    }
    }
}

void TracerBase::UartUpdateTracerState(const UartTracerMessage &status_msg, UartTracerState &state)
{
  switch (status_msg.type) {
    case UartTracerMotionStatusMsg: {
      // std::cout << "motion control feedback received" << std::endl;
      const UartMotionStatusMessage &msg = status_msg.body.motion_status_msg;
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
    case UartTracerLightStatusMsg: {
      // std::cout << "light control feedback received" << std::endl;
      const UartLightStatusMessage &msg = status_msg.body.light_status_msg;
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
    case UartTracerSystemStatusMsg: {
      // std::cout << "system status feedback received" << std::endl;
      const UartSystemStatusMessage &msg = status_msg.body.system_status_msg;
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
    case UartTracerMotorDriverStatusMsg: {
      // std::cout << "motor 1 driver feedback received" << std::endl;
      const UartMotorDriverStatusMessage &msg =
          status_msg.body.motor_driver_status_msg;
      for (int i = 0; i < UartTracerState::motor_num; ++i) {
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
  default:break;
  }
}
} // namespace westonrobot
