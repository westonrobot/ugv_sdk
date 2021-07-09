/*
 * scout_protocol_v1_parser.cpp
 *
 * Created on: Jul 08, 2021 22:43
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "ugv_sdk/details/protocol_v1/scout_protocol_v1_parser.hpp"

namespace westonrobot {
// CAN support
bool ScoutProtocolV1Parser::DecodeMessage(const struct can_frame *rx_frame,
                                          AgxMessage *msg) {
  //   ScoutMessage scout_msg;
  // if ScoutMessage found, convert to AgxMessage
  //   if (DecodeScoutMsgFromCAN(rx_frame, &scout_msg)) {
  //     switch (scout_msg.type) {
  //       case ScoutMotionStatusMsg: {
  //         break;
  //       }
  //       case ScoutLightStatusMsg: {
  //         break;
  //       }
  //       case ScoutSystemStatusMsg: {
  //         break;
  //       }
  //       case ScoutMotorDriverStatusMsg: {
  //         break;
  //       }
  //     }
  //   }
  return false;
}

void ScoutProtocolV1Parser::EncodeMessage(const AgxMessage *msg,
                                          struct can_frame *tx_frame) {
  //   ScoutMessage scout_msg;
  // convert to ScoutMessage, then encode to can frame
  //   switch (msg->type) {
  //     case AgxMsgMotionCommand: {
  //     //   scout_msg.type = ScoutMotionControlMsg;
  //     //   scout_msg.body.motion_control_msg.data.cmd.control_mode =
  //     //       CTRL_MODE_COMMANDED;
  //     //   scout_msg.body.motion_control_msg.data.cmd.control_mode =
  //     //       CTRL_MODE_CMD_CAN;
  //     //   scout_msg.body.motion_control_msg.data.cmd.fault_clear_flag =
  //     0x00;

  //       /*
  //       std::lock_guard<std::mutex> guard(motion_cmd_mutex_);
  //     current_motion_cmd_.linear_velocity = static_cast<int8_t>(
  //         linear_vel / ScoutCmdLimits::max_linear_velocity * 100.0);
  //     current_motion_cmd_.angular_velocity = static_cast<int8_t>(
  //         angular_vel / ScoutCmdLimits::max_angular_velocity * 100.0);
  //       */
  //     //   scout_msg.body.motion_control_msg.data.cmd.linear_velocity_cmd =
  //     //       current_motion_cmd_.linear_velocity;
  //     //   scout_msg.body.motion_control_msg.data.cmd.angular_velocity_cmd =
  //     //       current_motion_cmd_.angular_velocity;
  //     //   scout_msg.body.motion_control_msg.data.cmd.reserved0 = 0;
  //     //   scout_msg.body.motion_control_msg.data.cmd.reserved1 = 0;
  //     //   scout_msg.body.motion_control_msg.data.cmd.count = count;
  //     //   scout_msg.body.motion_control_msg.data.cmd.checksum =
  //     //       CalculateChecksum(CAN_MSG_MOTION_CONTROL_CMD_ID,
  //     //                         scout_msg.body.motion_control_msg.data.raw,
  //     8);
  //       break;
  //     }
  //     case AgxMsgLightCommand: {
  //       scout_msg.body.light_control_msg.data.cmd.light_ctrl_enable =
  //           LIGHT_ENABLE_CTRL;

  //     //   scout_msg.body.light_control_msg.data.cmd.front_mode =
  //     //       static_cast<uint8_t>(current_light_cmd_.front_mode);
  //     //   scout_msg.body.light_control_msg.data.cmd.front_custom =
  //     //       current_light_cmd_.front_custom_value;
  //     //   scout_msg.body.light_control_msg.data.cmd.rear_mode =
  //     //       static_cast<uint8_t>(current_light_cmd_.rear_mode);
  //     //   scout_msg.body.light_control_msg.data.cmd.rear_custom =
  //     //       current_light_cmd_.rear_custom_value;
  //     //   scout_msg.body.light_control_msg.data.cmd.reserved0 = 0;
  //     //   scout_msg.body.light_control_msg.data.cmd.count = count;

  //       scout_msg.body.light_control_msg.data.cmd.checksum =
  //           CalculateChecksum(CAN_MSG_LIGHT_CONTROL_CMD_ID,
  //                             scout_msg.body.light_control_msg.data.raw, 8);

  //       break;
  //     }
  //   }
}

uint8_t ScoutProtocolV1Parser::CalculateChecksum(uint16_t id, uint8_t *data,
                                                 uint8_t dlc) {
  uint8_t checksum = 0x00;
  checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
  for (int i = 0; i < (dlc - 1); ++i) checksum += data[i];
  return checksum;
}

// UART support
bool ScoutProtocolV1Parser::DecodeMessage(uint8_t *data, uint8_t dlc,
                                          AgxMessage *msg) {}

void ScoutProtocolV1Parser::EncodeMessage(const AgxMessage *msg, uint8_t *buf,
                                          uint8_t *len) {}

uint8_t ScoutProtocolV1Parser::CalculateChecksum(uint8_t *buf, uint8_t len) {}
}  // namespace westonrobot
