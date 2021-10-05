/*
 * agilex_base.hpp
 *
 * Created on: Dec 22, 2020 17:14
 * Description:
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

#include "ugv_sdk/details/async_port/async_can.hpp"
#include "ugv_sdk/details/interface/robot_common_interface.hpp"
#include "ugv_sdk/details/interface/parser_interface.hpp"

namespace westonrobot {
template <typename ParserType>
class AgilexBase : public RobotCommonInterface {
 public:
  AgilexBase() {
    static_assert(
        std::is_base_of<ParserInterface<ProtocolVersion::AGX_V1>,
                        ParserType>::value ||
            std::is_base_of<ParserInterface<ProtocolVersion::AGX_V2>,
                            ParserType>::value,
        "Incompatible parser for the AgilexBase class, expecting one derived "
        "from ParserInterface!");
  };
  virtual ~AgilexBase() { DisconnectPort(); }

  // do not allow copy or assignment
  AgilexBase(const AgilexBase &hunter) = delete;
  AgilexBase &operator=(const AgilexBase &hunter) = delete;

  bool Connect(std::string can_name) override {
    return ConnectPort(can_name,
                       std::bind(&AgilexBase<ParserType>::ParseCANFrame, this,
                                 std::placeholders::_1));
  }

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
                         double lateral_vel, double steering_angle) {
    if (can_connected_) {
      // motion control message
      AgxMessage msg;
      if (parser_.GetParserProtocolVersion() == ProtocolVersion::AGX_V1) {
        msg.type = AgxMsgMotionCommandV1;
        msg.body.v1_motion_command_msg.control_mode = CONTROL_MODE_CAN;
        msg.body.v1_motion_command_msg.clear_all_error = false;
        msg.body.v1_motion_command_msg.linear = linear_vel;
        // normally only one of angular_vel and steering_angle can be non-zero
        msg.body.v1_motion_command_msg.angular =
            std::abs(angular_vel) > std::abs(steering_angle) ? angular_vel
                                                             : steering_angle;
        msg.body.v1_motion_command_msg.lateral = lateral_vel;
      } else if (parser_.GetParserProtocolVersion() ==
                 ProtocolVersion::AGX_V2) {
        msg.type = AgxMsgMotionCommand;
        msg.body.motion_command_msg.linear_velocity = linear_vel;
        msg.body.motion_command_msg.angular_velocity = angular_vel;
        msg.body.motion_command_msg.lateral_velocity = lateral_vel;
        msg.body.motion_command_msg.steering_angle = steering_angle;
      }

      std::cout << "Sending motion cmd: " << linear_vel << ", " << angular_vel
                << ", " << lateral_vel << std::endl;

      // send to can bus
      can_frame frame;
      if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
    }
  }

  // one-shot light command
  void SendLightCommand(AgxLightMode front_mode, uint8_t front_custom_value,
                        AgxLightMode rear_mode, uint8_t rear_custom_value) {
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

  void ResetRobotState() override {}

  ProtocolVersion GetParserProtocolVersion() override {
    return parser_.GetParserProtocolVersion();
  }

  CoreStateMsgGroup GetRobotCoreStateMsgGroup() override {
    std::lock_guard<std::mutex> guard(core_state_mtx_);
    return core_state_msgs_;
  }

  ActuatorStateMsgGroup GetActuatorStateMsgGroup() override {
    std::lock_guard<std::mutex> guard(actuator_state_mtx_);
    return actuator_state_msgs_;
  }

 protected:
  ParserType parser_;

  // divide feedback messages into smaller groups to avoid the
  // state mutex being locked for too often such that accessing
  // the data become difficult

  /* feedback group 1: core state */
  std::mutex core_state_mtx_;
  CoreStateMsgGroup core_state_msgs_;

  /* feedback group 2: actuator state */
  std::mutex actuator_state_mtx_;
  ActuatorStateMsgGroup actuator_state_msgs_;

  /* feedback group 3: common sensor */

  // communication interface
  bool can_connected_ = false;
  std::shared_ptr<AsyncCAN> can_;

  // connect to roboot from CAN or serial
  using CANFrameRxCallback = AsyncCAN::ReceiveCallback;
  bool ConnectPort(std::string dev_name, CANFrameRxCallback cb) {
    can_ = std::make_shared<AsyncCAN>(dev_name);
    can_->SetReceiveCallback(cb);
    can_connected_ = can_->StartListening();
    return can_connected_;
  }

  void DisconnectPort() {
    if (can_connected_) can_->StopService();
  }

  void SetBrakeMode(AgxBrakeMode mode) {
    // construct message
    AgxMessage msg;
    msg.type = AgxMsgBrakeModeConfig;
    msg.body.brake_mode_config_msg.mode = mode;

    // encode msg to can frame and send to bus
    can_frame frame;
    if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
  }

  void ParseCANFrame(can_frame *rx_frame) {
    AgxMessage status_msg;
    if (parser_.DecodeMessage(rx_frame, &status_msg)) {
      UpdateRobotCoreState(status_msg);
      UpdateActuatorState(status_msg);
    }
  }

  void UpdateRobotCoreState(const AgxMessage &status_msg) {
    std::lock_guard<std::mutex> guard(core_state_mtx_);
    switch (status_msg.type) {
      case AgxMsgSystemState: {
        //   std::cout << "system status feedback received" << std::endl;
        core_state_msgs_.time_stamp = AgxMsgRefClock::now();
        core_state_msgs_.system_state = status_msg.body.system_state_msg;
        break;
      }
      case AgxMsgMotionState: {
        // std::cout << "motion control feedback received" << std::endl;
        core_state_msgs_.time_stamp = AgxMsgRefClock::now();
        core_state_msgs_.motion_state = status_msg.body.motion_state_msg;
        break;
      }
      case AgxMsgLightState: {
        // std::cout << "light control feedback received" << std::endl;
        core_state_msgs_.time_stamp = AgxMsgRefClock::now();
        core_state_msgs_.light_state = status_msg.body.light_state_msg;
        break;
      }
      case AgxMsgMotionModeState: {
        // std::cout << "motion mode feedback received" << std::endl;
        core_state_msgs_.time_stamp = AgxMsgRefClock::now();
        core_state_msgs_.motion_mode_state =
            status_msg.body.motion_mode_state_msg;
        break;
      }
      case AgxMsgRcState: {
        // std::cout << "rc feedback received" << std::endl;
        core_state_msgs_.time_stamp = AgxMsgRefClock::now();
        core_state_msgs_.rc_state = status_msg.body.rc_state_msg;
        break;
      }
      default:
        break;
    }
  }

  void UpdateActuatorState(const AgxMessage &status_msg) {
    std::lock_guard<std::mutex> guard(actuator_state_mtx_);
    switch (status_msg.type) {
      case AgxMsgActuatorHSState: {
        // std::cout << "actuator hs feedback received" << std::endl;
        actuator_state_msgs_.time_stamp = AgxMsgRefClock::now();
        actuator_state_msgs_
            .actuator_hs_state[status_msg.body.actuator_hs_state_msg.motor_id] =
            status_msg.body.actuator_hs_state_msg;
        break;
      }
      case AgxMsgActuatorLSState: {
        // std::cout << "actuator ls feedback received" << std::endl;
        actuator_state_msgs_.time_stamp = AgxMsgRefClock::now();
        actuator_state_msgs_
            .actuator_ls_state[status_msg.body.actuator_ls_state_msg.motor_id] =
            status_msg.body.actuator_ls_state_msg;
        break;
      }
      case AgxMsgActuatorStateV1: {
        // std::cout << "actuator v1 feedback received" << std::endl;
        actuator_state_msgs_.time_stamp = AgxMsgRefClock::now();
        actuator_state_msgs_
            .actuator_state[status_msg.body.v1_actuator_state_msg.motor_id] =
            status_msg.body.v1_actuator_state_msg;
        break;
      }
      default:
        break;
    }
  }
};
}  // namespace westonrobot

#endif /* AGILEX_BASE_HPP */
