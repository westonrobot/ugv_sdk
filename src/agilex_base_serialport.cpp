#include "ugv_sdk/agilex_base_serialport.h"

#include "stopwatch.hpp"
#include "ugv_sdk/details/agilex_msg_parser.h"

namespace westonrobot {
AgilexBaseSerialPort::~AgilexBaseSerialPort() {
  // release resource if occupied
  Disconnect();

  // joint cmd thread
  if (cmd_thread_.joinable()) cmd_thread_.join();
}

void AgilexBaseSerialPort::Connect(std::string dev_name,
                                   SerialFrameRxCallback cb,
                                   uint32_t bouadrate) {
  serial_ = std::make_shared<AsyncSerial>(dev_name, bouadrate);
  serial_->SetReceiveCallback(cb);
  serial_->StartListening();
  serial_->SetBaudRate(bouadrate);
  serial_connected_ = true;
}

void AgilexBaseSerialPort::Disconnect() {
  if (serial_connected_) serial_->StopService();
}

void AgilexBaseSerialPort::Terminate() {
  keep_running_ = false;
  std::terminate();
}

void AgilexBaseSerialPort::EnableCmdTimeout(uint32_t timeout_ms) {
  enable_timeout_ = true;
  timeout_ms_ = timeout_ms;
}

void AgilexBaseSerialPort::StartCmdThread() {
  keep_running_ = true;
  cmd_thread_ = std::thread(std::bind(&AgilexBaseSerialPort::ControlLoop, this,
                                      cmd_thread_period_ms_));
  cmd_thread_started_ = true;
}

void AgilexBaseSerialPort::ControlLoop(int32_t period_ms) {
  uint32_t timeout_iter_num;

  if (enable_timeout_) {
    if (timeout_ms_ < period_ms) timeout_ms_ = period_ms;
    timeout_iter_num = static_cast<uint32_t>(timeout_ms_ / period_ms);
    // std::cout << "Timeout iteration number: " << timeout_iter_num <<
    // std::endl;
  }

  Timer tm;
  while (keep_running_) {
    tm.reset();
    if (enable_timeout_) {
      if (watchdog_counter_ < timeout_iter_num) {
        SendRobotCmd();
        ++watchdog_counter_;
      }
      //   else {
      //     std::cout << "Warning: cmd timeout, no cmd sent to robot" <<
      //     std::endl;
      //   }
    } else {
      SendRobotCmd();
    }
    tm.sleep_until_ms(period_ms);
  }
}

void AgilexBaseSerialPort::EnableCommandedMode() {
  // construct message
  AgxMessage msg;
  msg.type = AgxMsgControlModeConfig;

  // mode = 0x00 is the default control mode
  // mode = 0x01 for soft control mode
  msg.body.control_mode_config_msg.mode = CONTROL_MODE_CAN;

  // encode msg to can frame and send to bus
  can_frame frame;
  EncodeCanFrame(&msg, &frame);
  SendCanFrame(frame);
}

void AgilexBaseSerialPort::SetMotionCommand(double linear_vel,
                                            double angular_vel,
                                            double lateral_velocity,
                                            double steering_angle) {
  // make sure cmd thread is started before attempting to send commands
  if (!cmd_thread_started_) StartCmdThread();

  std::lock_guard<std::mutex> guard(motion_cmd_mutex_);
  current_motion_cmd_.linear_velocity = linear_vel;
  current_motion_cmd_.angular_velocity = angular_vel;
  current_motion_cmd_.lateral_velocity = lateral_velocity;
  current_motion_cmd_.steering_angle = steering_angle;

  FeedCmdTimeoutWatchdog();
}

void AgilexBaseSerialPort::SendRobotCmd() {
  if (serial_connected_) {
    // motion control message
    AgxMessage msg;
    msg.type = AgxMsgMotionCommand;

    motion_cmd_mutex_.lock();
    msg.body.motion_command_msg = current_motion_cmd_;
    motion_cmd_mutex_.unlock();

    // send to can bus
    can_frame frame;
    EncodeCanFrame(&msg, &frame);
    SendCanFrame(frame);
  }
}

void AgilexBaseSerialPort::SendLightCommand(LightMode front_mode,
                                            uint8_t front_custom_value,
                                            LightMode rear_mode,
                                            uint8_t rear_custom_value) {
  //   AgxMessage msg;
  //   msg.type = AgxMsgLightCommand;

  //   msg.body.light_command_msg.cmd_ctrl_allowed = true;
  //   msg.body.light_command_msg.front_light.mode = front_mode;
  //   msg.body.light_command_msg.front_light.custom_value = front_custom_value;
  //   msg.body.light_command_msg.rear_light.mode = rear_mode;
  //   msg.body.light_command_msg.rear_light.custom_value = rear_custom_value;

  //   // send to can bus
  //   can_frame frame;
  //   EncodeCanFrame(&msg, &frame);
  //   serial_->SendFrame(frame);
}

void AgilexBaseSerialPort::DisableLightControl() {
  //   AgxMessage msg;
  //   msg.type = AgxMsgLightCommand;

  //   msg.body.light_command_msg.cmd_ctrl_allowed = false;

  //   // send to can bus
  //   can_frame frame;
  //   EncodeCanFrame(&msg, &frame);
  //   serial_->SendFrame(frame);
}

void AgilexBaseSerialPort::SetMotionMode(uint8_t mode) {
  AgxMessage msg;
  msg.type = AgxMsgSetMotionMode;
  msg.body.motion_mode_msg.motion_mode = mode;

  // send to can bus
  can_frame frame;
  EncodeCanFrame(&msg, &frame);
  SendCanFrame(frame);
}

void AgilexBaseSerialPort::SendCanFrame(const can_frame &frame) {
  size_t len = frame.can_dlc;
  if (len > 8) {  // 0~8 length
    return;
  }

  uint32_t checksum = 0;
  uint8_t frame_len = 0x0e;
  uint8_t data[frame_len] = {0x55, frame_len};
  data[2] = (uint8_t)(frame.can_id >> 8);
  data[3] = (uint8_t)(frame.can_id & 0xff);
  for (size_t i = 0; i < len; i++) {
    data[i + 4] = frame.data[i];
    checksum += frame.data[i];
  }
  data[frame_len - 1] = (uint8_t)(checksum & 0xff);
  serial_->SendBytes(data, frame_len);
}

}  // namespace westonrobot