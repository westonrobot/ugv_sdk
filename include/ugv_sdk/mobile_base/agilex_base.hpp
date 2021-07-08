/*
 * agilex_base.hpp
 *
 * Created on: Dec 22, 2020 17:14
 * Description:
 *
 * Each robot class derived from this base class should provide implementation
 * for the following two functions:
 *
 * - virtual void Connect(std::string dev_name) = 0;
 * - virtual void ParseCANFrame(can_frame *rx_frame) = 0;
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef AGILEX_BASE_HPP
#define AGILEX_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <atomic>

#include "ugv_sdk/details/stopwatch.hpp"
#include "ugv_sdk/details/async_port/async_can.hpp"

#include "ugv_sdk/interface/robot_interface.hpp"
#include "ugv_sdk/protocol_v2/agilex_msg_parser.h"

namespace westonrobot {
enum class AgilexProtocol { V1, V2 };

template <typename ParserType>
class AgilexBase : public RobotInterface {
 public:
  AgilexBase() = default;
  virtual ~AgilexBase() { DisconnectPort(); }

  // do not allow copy or assignment
  AgilexBase(const AgilexBase &hunter) = delete;
  AgilexBase &operator=(const AgilexBase &hunter) = delete;

  // switch to commanded mode
  void EnableCommandedMode() {
    // construct message
    AgxMessage msg;
    msg.type = AgxMsgControlModeConfig;
    msg.body.control_mode_config_msg.mode = CONTROL_MODE_CAN;

    // encode msg to can frame and send to bus
    can_frame frame;
    parser_.EncodeMessage(&msg, &frame);
    can_->SendFrame(frame);
  }

  // must be called at a frequency >= 50Hz
  void SendMotionCommand(double linear_vel, double angular_vel,
                         double lateral_velocity, double steering_angle) {
    if (can_connected_) {
      // motion control message
      AgxMessage msg;
      msg.type = AgxMsgMotionCommand;
      msg.body.motion_command_msg.linear_velocity = linear_vel;
      msg.body.motion_command_msg.angular_velocity = angular_vel;
      msg.body.motion_command_msg.lateral_velocity = lateral_velocity;
      msg.body.motion_command_msg.steering_angle = steering_angle;

      // send to can bus
      can_frame frame;
      parser_.EncodeMessage(&msg, &frame);
      can_->SendFrame(frame);
    }
  }

  // one-shot light command
  void SendLightCommand(LightMode front_mode, uint8_t front_custom_value,
                        LightMode rear_mode, uint8_t rear_custom_value) {
    if (can_connected_) {
      AgxMessage msg;
      msg.type = AgxMsgLightCommand;
      msg.body.light_command_msg.cmd_ctrl_allowed = true;
      msg.body.light_command_msg.front_light.mode = front_mode;
      msg.body.light_command_msg.front_light.custom_value = front_custom_value;
      msg.body.light_command_msg.rear_light.mode = rear_mode;
      msg.body.light_command_msg.rear_light.custom_value = rear_custom_value;

      // send to can bus
      can_frame frame;
      parser_.EncodeMessage(&msg, &frame);
      can_->SendFrame(frame);
    }
  }

  void DisableLightControl() {
    if (can_connected_) {
      AgxMessage msg;
      msg.type = AgxMsgLightCommand;

      msg.body.light_command_msg.cmd_ctrl_allowed = false;

      // send to can bus
      can_frame frame;
      parser_.EncodeMessage(&msg, &frame);
      can_->SendFrame(frame);
    }
  }

  // motion mode change
  void SetMotionMode(uint8_t mode) {
    if (can_connected_) {
      AgxMessage msg;
      msg.type = AgxMsgSetMotionMode;
      msg.body.motion_mode_msg.motion_mode = mode;

      // send to can bus
      can_frame frame;
      parser_.EncodeMessage(&msg, &frame);
      can_->SendFrame(frame);
    }
  }

 protected:
  std::mutex state_mutex_;
  //   std::mutex motion_cmd_mutex_;
  MotionCommandMessage current_motion_cmd_;

  ParserType parser_;

  // communication interface
  bool can_connected_ = false;
  std::shared_ptr<AsyncCAN> can_;

  // connect to roboot from CAN or serial
  using CANFrameRxCallback = AsyncCAN::ReceiveCallback;
  void ConnectPort(std::string dev_name, CANFrameRxCallback cb) {
    can_ = std::make_shared<AsyncCAN>(dev_name);
    can_->SetReceiveCallback(cb);
    can_->StartListening();
    can_connected_ = true;
  }

  void DisconnectPort() {
    if (can_connected_) can_->StopService();
  }

  virtual void ParseCANFrame(can_frame *rx_frame) = 0;
};
}  // namespace westonrobot

#endif /* AGILEX_BASE_HPP */
