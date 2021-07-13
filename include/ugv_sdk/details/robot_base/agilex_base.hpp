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
#include "ugv_sdk/details/interface/robot_interface.hpp"

namespace westonrobot {
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
    if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
  }

  // must be called at a frequency >= 50Hz
  void SendMotionCommand(double linear_vel, double angular_vel,
                         double lateral_velocity, double steering_angle) {
    if (can_connected_) {
      // motion control message
      AgxMessage msg;
      if (parser_.GetProtocolVersion() == ProtocolVersion::AGX_V1) {
        msg.type = AgxMsgMotionCommandV1;
        msg.body.v1_motion_command_msg.control_mode = CONTROL_MODE_CAN;
        msg.body.v1_motion_command_msg.clear_all_error = false;
        msg.body.v1_motion_command_msg.linear = linear_vel;
        msg.body.v1_motion_command_msg.angular = angular_vel;
        msg.body.v1_motion_command_msg.lateral = lateral_velocity;
      } else if (parser_.GetProtocolVersion() == ProtocolVersion::AGX_V2) {
        msg.type = AgxMsgMotionCommand;
        msg.body.motion_command_msg.linear_velocity = linear_vel;
        msg.body.motion_command_msg.angular_velocity = angular_vel;
        msg.body.motion_command_msg.lateral_velocity = lateral_velocity;
        msg.body.motion_command_msg.steering_angle = steering_angle;
      }

      std::cout << "sending motion cmd: " << linear_vel << "," << angular_vel
                << std::endl;
                
      // send to can bus
      can_frame frame;
      if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
    }
  }

  // one-shot light command
  void SendLightCommand(LightMode front_mode, uint8_t front_custom_value,
                        LightMode rear_mode, uint8_t rear_custom_value) {
    if (can_connected_) {
      AgxMessage msg;
      msg.type = AgxMsgLightCommand;
      msg.body.light_command_msg.enable_cmd_ctrl = true;
      msg.body.light_command_msg.front_light.mode = front_mode;
      msg.body.light_command_msg.front_light.custom_value = front_custom_value;
      msg.body.light_command_msg.rear_light.mode = rear_mode;
      msg.body.light_command_msg.rear_light.custom_value = rear_custom_value;

      // send to can bus
      can_frame frame;
      if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
    }
  }

  void DisableLightControl() {
    if (can_connected_) {
      AgxMessage msg;
      msg.type = AgxMsgLightCommand;

      msg.body.light_command_msg.enable_cmd_ctrl = false;

      // send to can bus
      can_frame frame;
      if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
    }
  }

  // motion mode change
  void SetMotionMode(uint8_t mode) {
    if (can_connected_) {
      AgxMessage msg;
      msg.type = AgxMsgSetMotionModeCommand;
      msg.body.motion_mode_msg.motion_mode = mode;

      // send to can bus
      can_frame frame;
      if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
    }
  }

  ProtocolVersion GetProtocolVersion() override {
    return parser_.GetProtocolVersion();
  }

 protected:
  ParserType parser_;
  std::mutex state_mutex_;

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
