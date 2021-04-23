/*
 * agilex_base.cpp
 *
 * Created on: Dec 22, 2020 17:20
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "ugv_sdk/agilex_base.hpp"

#include "ugv_sdk/details/agilex_msg_parser.h"
#include "stopwatch.hpp"

namespace westonrobot {
AgilexBase::~AgilexBase() {
  // release resource if occupied
  Disconnect();

  // joint cmd thread
  if (cmd_thread_.joinable()) cmd_thread_.join();
}

void AgilexBase::Connect(std::string dev_name, CANFrameRxCallback cb) {
  can_ = std::make_shared<AsyncCAN>(dev_name);
  can_->SetReceiveCallback(cb);
  can_->StartListening();
  can_connected_ = true;
}

void AgilexBase::Disconnect() {
  if (can_connected_) can_->StopService();
}

void AgilexBase::Terminate() {
  keep_running_ = false;
  std::terminate();
}

void AgilexBase::EnableCmdTimeout(uint32_t timeout_ms) {
  enable_timeout_ = true;
  timeout_ms_ = timeout_ms;
}

void AgilexBase::StartCmdThread() {
  keep_running_ = true;
  cmd_thread_ = std::thread(
      std::bind(&AgilexBase::ControlLoop, this, cmd_thread_period_ms_));
  cmd_thread_started_ = true;
}

void AgilexBase::ControlLoop(int32_t period_ms) {
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

void AgilexBase::EnableCommandedMode() {
  // construct message
  AgxMessage msg;
  msg.type = AgxMsgControlModeConfig;
  msg.body.control_mode_config_msg.mode = CONTROL_MODE_CAN;

  // encode msg to can frame and send to bus
  can_frame frame;
  EncodeCanFrame(&msg, &frame);
  can_->SendFrame(frame);
}

void AgilexBase::SetMotionCommand(double linear_vel, double angular_vel,
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

void AgilexBase::SendRobotCmd() {
  if (can_connected_) {
    // motion control message
    AgxMessage msg;
    msg.type = AgxMsgMotionCommand;

    motion_cmd_mutex_.lock();
    msg.body.motion_command_msg = current_motion_cmd_;
    motion_cmd_mutex_.unlock();

    // send to can bus
    can_frame frame;
    EncodeCanFrame(&msg, &frame);
    can_->SendFrame(frame);
  }
}

void AgilexBase::SendLightCommand(LightMode front_mode,
                                  uint8_t front_custom_value,
                                  LightMode rear_mode,
                                  uint8_t rear_custom_value) {
  AgxMessage msg;
  msg.type = AgxMsgLightCommand;

  msg.body.light_command_msg.cmd_ctrl_allowed = true;
  msg.body.light_command_msg.front_light.mode = front_mode;
  msg.body.light_command_msg.front_light.custom_value = front_custom_value;
  msg.body.light_command_msg.rear_light.mode = rear_mode;
  msg.body.light_command_msg.rear_light.custom_value = rear_custom_value;

  // send to can bus
  can_frame frame;
  EncodeCanFrame(&msg, &frame);
  can_->SendFrame(frame);
}

void AgilexBase::DisableLightControl() {
  AgxMessage msg;
  msg.type = AgxMsgLightCommand;

  msg.body.light_command_msg.cmd_ctrl_allowed = false;

  // send to can bus
  can_frame frame;
  EncodeCanFrame(&msg, &frame);
  can_->SendFrame(frame);
}

void AgilexBase::SetMotionMode(uint8_t mode)
{
   AgxMessage msg;
   msg.type = AgxMsgSetMotionMode;
   msg.body.motion_mode_msg.motion_mode = mode;

   // send to can bus
   can_frame frame;
   EncodeCanFrame(&msg, &frame);
   can_->SendFrame(frame);
}
}  // namespace westonrobot
