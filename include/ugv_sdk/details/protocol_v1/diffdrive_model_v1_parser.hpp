/*
 * scout_protocol_v1_parser.hpp
 *
 * Created on: Jul 08, 2021 22:30
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef SCOUT_PROTOCOL_V1_PARSER_HPP
#define SCOUT_PROTOCOL_V1_PARSER_HPP

#include "ugv_sdk/details/interface/parser_interface.hpp"

#include "ugv_sdk/details/protocol_v1/agilex_msg_parser_v1.h"
#include "ugv_sdk/details/protocol_v1/robot_limits.hpp"

namespace westonrobot {
template <typename RobotLimitsType>
class DiffDriveModelV1Parser : public ParserInterface {
 public:
  bool DecodeMessage(const struct can_frame *rx_frame,
                     AgxMessage *msg) override {
    return DecodeCanFrameV1(rx_frame, msg);
  }

  void EncodeMessage(const AgxMessage *msg,
                     struct can_frame *tx_frame) override {
    AgxMessage msg_v1 = *msg;
    if (msg->type == AgxMsgMotionCommand) {
      float linear = msg->body.motion_command_msg.linear_velocity;
      float angular = msg->body.motion_command_msg.angular_velocity;
      if (linear > ScoutV2Limits::max_linear_velocity)
        linear = ScoutV2Limits::max_linear_velocity;
      else if (linear < ScoutV2Limits::min_linear_velocity)
        linear = ScoutV2Limits::min_linear_velocity;
      if (angular > ScoutV2Limits::max_angular_velocity)
        angular = ScoutV2Limits::max_angular_velocity;
      else if (angular < ScoutV2Limits::min_angular_velocity)
        angular = ScoutV2Limits::min_angular_velocity;
      msg_v1.body.motion_command_msg.linear_velocity =
          linear / ScoutV2Limits::max_linear_velocity * 100.0;
      msg_v1.body.motion_command_msg.angular_velocity = static_cast<int8_t>(
          angular / ScoutV2Limits::max_angular_velocity * 100.0);
    }
    EncodeCanFrameV1(&msg_v1, tx_frame);
  }

  uint8_t CalculateChecksum(uint16_t id, uint8_t *data, uint8_t dlc) override {
    return CalcCanFrameChecksumV1(id, data, dlc);
  }

  // UART support
  bool DecodeMessage(uint8_t *data, uint8_t dlc, AgxMessage *msg) override {}

  void EncodeMessage(const AgxMessage *msg, uint8_t *buf,
                     uint8_t *len) override {}

  uint8_t CalculateChecksum(uint8_t *buf, uint8_t len) override {}
};

using ScoutProtocolV1Parser = DiffDriveModelV1Parser<ScoutV2Limits>;
using ScoutMiniProtocolV1Parser = DiffDriveModelV1Parser<ScoutV2Limits>;
}  // namespace westonrobot

#endif /* SCOUT_PROTOCOL_V1_PARSER_HPP */
