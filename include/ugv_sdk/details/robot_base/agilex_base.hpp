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
#include <iostream>
#include <chrono>

#include "ugv_sdk/details/async_port/async_can.hpp"
#include "ugv_sdk/details/interface/robot_common_interface.hpp"
#include "ugv_sdk/details/parser_base.hpp"

namespace westonrobot {
struct CoreStateMsgGroup {
  SdkTimePoint time_stamp;

  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  LightStateMessage light_state;
  MotionModeStateMessage motion_mode_state;
  RcStateMessage rc_state;
};

struct ActuatorStateMsgGroup {
  SdkTimePoint time_stamp;

  ActuatorHSStateMessage actuator_hs_state[AGX_MAX_ACTUATOR_NUM];  // v2 only
  ActuatorLSStateMessage actuator_ls_state[AGX_MAX_ACTUATOR_NUM];  // v2 only
  ActuatorStateMessageV1 actuator_state[AGX_MAX_ACTUATOR_NUM];     // v1 only

  MotorAngleMessage motor_angles;  // ranger only
  MotorSpeedMessage motor_speeds;  // ranger only
};

struct CommonSensorStateMsgGroup {
  SdkTimePoint time_stamp;
  BmsBasicMessage bms_basic_state;
  BmsExtendedMessage bms_extend_state;
};

template <typename ParserType>
class AgilexBase : public RobotCommonInterface {
 public:
  AgilexBase() {
    static_assert(
        std::is_base_of<ParserBase<ProtocolVersion::AGX_V1>,
                        ParserType>::value ||
            std::is_base_of<ParserBase<ProtocolVersion::AGX_V2>,
                            ParserType>::value,
        "Incompatible parser for the AgilexBase class, expecting one derived "
        "from ParserBase!");
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
    if (can_ != nullptr && can_->IsOpened()) {
      can_frame frame;
      if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
    }
  }

  // must be called at a frequency >= 50Hz
  void SendMotionCommand(double linear_vel, double angular_vel,
                         double lateral_vel, double steering_angle) {
    if (can_ != nullptr && can_->IsOpened()) {
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

      //   std::cout << "Sending motion cmd: " << linear_vel << ", " <<
      //   angular_vel
      //             << ", " << lateral_vel << std::endl;

      // send to can bus
      can_frame frame;
      if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
    }
  }

  // one-shot light command
  void SendLightCommand(AgxLightMode front_mode, uint8_t front_custom_value,
                        AgxLightMode rear_mode, uint8_t rear_custom_value) {
    if (can_ != nullptr && can_->IsOpened()) {
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
    if (can_ != nullptr && can_->IsOpened()) {
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
    if (can_ != nullptr && can_->IsOpened()) {
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

  CoreStateMsgGroup GetRobotCoreStateMsgGroup() {
    std::lock_guard<std::mutex> guard(core_state_mtx_);
    return core_state_msgs_;
  }

  ActuatorStateMsgGroup GetActuatorStateMsgGroup() {
    std::lock_guard<std::mutex> guard(actuator_state_mtx_);
    return actuator_state_msgs_;
  }

  CommonSensorStateMsgGroup GetCommonSensorStateMsgGroup() {
    std::lock_guard<std::mutex> guard(common_sensor_state_mtx_);
    return common_sensor_state_msgs_;
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
  std::mutex common_sensor_state_mtx_;
  CommonSensorStateMsgGroup common_sensor_state_msgs_;

  std::mutex version_str_buf_mtx_;
  std::string version_string_buffer_;

  // communication interface
  std::shared_ptr<AsyncCAN> can_;

  // connect to roboot from CAN or serial
  using CANFrameRxCallback = AsyncCAN::ReceiveCallback;
  bool ConnectPort(std::string dev_name, CANFrameRxCallback cb) {
    can_ = std::make_shared<AsyncCAN>(dev_name);
    can_->SetReceiveCallback(cb);
    return can_->Open();
  }

  void DisconnectPort() {
    if (can_ != nullptr && can_->IsOpened()) can_->Close();
  }

  void SetBrakeMode(AgxBrakeMode mode) {
    // construct message
    AgxMessage msg;
    msg.type = AgxMsgBrakeModeConfig;
    msg.body.brake_mode_config_msg.mode = mode;

    // encode msg to can frame and send to bus
    if (can_ != nullptr && can_->IsOpened()) {
      can_frame frame;
      if (parser_.EncodeMessage(&msg, &frame)) can_->SendFrame(frame);
    }
  }

  std::string RequestVersion(int timeout_sec) {
    // clear buffer first
    version_string_buffer_.clear();

    // send request msg
    can_frame frame;
    frame.can_id = ((uint32_t)0x4a1);
    frame.can_dlc = 1;
    frame.data[0] = 0x01;
    can_->SendFrame(frame);

    // wait for response msgs
    using Clock = std::chrono::steady_clock;
    using Timepoint = Clock::time_point;

    std::string version_string;
    auto start_time = Clock::now();
    while (std::chrono::duration_cast<std::chrono::seconds>(Clock::now() -
                                                            start_time)
               .count() < timeout_sec) {
      // copy version string received from CAN
      {
        std::lock_guard<std::mutex> lock(version_str_buf_mtx_);
        version_string = version_string_buffer_;
      }
      if (version_string.size() >= 80) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // set version string to empty if not enough bytes received
    if (version_string.size() < 80) version_string = "";

    return version_string;
  }

  void ParseCANFrame(can_frame *rx_frame) {
    AgxMessage status_msg;

    if (parser_.DecodeMessage(rx_frame, &status_msg)) {
      UpdateRobotCoreState(status_msg);
      UpdateActuatorState(status_msg);
      UpdateCommonSensorState(status_msg);
      UpdateResponseVersion(status_msg);
    }
  }

  void UpdateRobotCoreState(const AgxMessage &status_msg) {
    std::lock_guard<std::mutex> guard(core_state_mtx_);
    switch (status_msg.type) {
      case AgxMsgSystemState: {
        //   std::cout << "system status feedback received" << std::endl;
        core_state_msgs_.time_stamp = SdkClock::now();
        core_state_msgs_.system_state = status_msg.body.system_state_msg;
        break;
      }
      case AgxMsgMotionState: {
        // std::cout << "motion control feedback received" << std::endl;
        core_state_msgs_.time_stamp = SdkClock::now();
        core_state_msgs_.motion_state = status_msg.body.motion_state_msg;
        break;
      }
      case AgxMsgLightState: {
        // std::cout << "light control feedback received" << std::endl;
        core_state_msgs_.time_stamp = SdkClock::now();
        core_state_msgs_.light_state = status_msg.body.light_state_msg;
        break;
      }
      case AgxMsgMotionModeState: {
        // std::cout << "motion mode feedback received" << std::endl;
        core_state_msgs_.time_stamp = SdkClock::now();
        core_state_msgs_.motion_mode_state =
            status_msg.body.motion_mode_state_msg;
        break;
      }
      case AgxMsgRcState: {
        // std::cout << "rc feedback received" << std::endl;
        core_state_msgs_.time_stamp = SdkClock::now();
        core_state_msgs_.rc_state = status_msg.body.rc_state_msg;
        break;
      }
      default:
        break;
    }
  }

  void UpdateActuatorState(const AgxMessage &status_msg) {
    std::lock_guard<std::mutex> guard(actuator_state_mtx_);
    actuator_state_msgs_.time_stamp = SdkClock::now();
    switch (status_msg.type) {
      case AgxMsgMotorAngle: {
        actuator_state_msgs_.motor_angles.angle_5 =
            status_msg.body.motor_angle_msg.angle_5;
        actuator_state_msgs_.motor_angles.angle_6 =
            status_msg.body.motor_angle_msg.angle_6;
        actuator_state_msgs_.motor_angles.angle_7 =
            status_msg.body.motor_angle_msg.angle_7;
        actuator_state_msgs_.motor_angles.angle_8 =
            status_msg.body.motor_angle_msg.angle_8;
        break;
      }
      case AgxMsgMotorSpeed: {
        actuator_state_msgs_.motor_speeds.speed_1 =
            status_msg.body.motor_speed_msg.speed_1;
        actuator_state_msgs_.motor_speeds.speed_2 =
            status_msg.body.motor_speed_msg.speed_2;
        actuator_state_msgs_.motor_speeds.speed_3 =
            status_msg.body.motor_speed_msg.speed_3;
        actuator_state_msgs_.motor_speeds.speed_4 =
            status_msg.body.motor_speed_msg.speed_4;
        break;
      }
      case AgxMsgActuatorHSState: {
        // std::cout << "actuator hs feedback received" << std::endl;
        actuator_state_msgs_
            .actuator_hs_state[status_msg.body.actuator_hs_state_msg.motor_id] =
            status_msg.body.actuator_hs_state_msg;
        break;
      }
      case AgxMsgActuatorLSState: {
        // std::cout << "actuator ls feedback received" << std::endl;
        actuator_state_msgs_
            .actuator_ls_state[status_msg.body.actuator_ls_state_msg.motor_id] =
            status_msg.body.actuator_ls_state_msg;
        break;
      }
      case AgxMsgActuatorStateV1: {
        // std::cout << "actuator v1 feedback received" << std::endl;
        actuator_state_msgs_
            .actuator_state[status_msg.body.v1_actuator_state_msg.motor_id] =
            status_msg.body.v1_actuator_state_msg;
        break;
      }
      default:
        break;
    }
  }

  void UpdateCommonSensorState(const AgxMessage &status_msg) {
    std::lock_guard<std::mutex> guard(common_sensor_state_mtx_);
    //    std::cout << common_sensor_state_msgs_.bms_basic_state.battery_soc<<
    //    std::endl;
    switch (status_msg.type) {
      case AgxMsgBmsBasic: {
        //      std::cout << "system status feedback received" << std::endl;
        common_sensor_state_msgs_.time_stamp = SdkClock::now();
        common_sensor_state_msgs_.bms_basic_state =
            status_msg.body.bms_basic_msg;
        break;
      }
      case AgxMsgBmsExtended: {
        common_sensor_state_msgs_.bms_extend_state = 
          status_msg.body.bms_extended_msg;
      }
      default:
        break;
    }
  }

  void UpdateResponseVersion(const AgxMessage &status_msg) {
    switch (status_msg.type) {
      case AgxMsgVersionResponse: {
        std::lock_guard<std::mutex> lock(version_str_buf_mtx_);
        for (int i = 0; i < 8; i++) {
          uint8_t data = status_msg.body.version_response_msg.bytes[i];
          if (data < 32 || data > 126) data = 32;
          version_string_buffer_ += data;
        }
        break;
      }
      default:
        break;
    }
  }
};
}  // namespace westonrobot

#endif /* AGILEX_BASE_HPP */
