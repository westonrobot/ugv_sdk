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

#include "ugv_sdk/details/parser_base.hpp"

#include "ugv_sdk/details/protocol_v1/agilex_msg_parser_v1.h"
#include "ugv_sdk/details/protocol_v1/robot_limits.hpp"

namespace westonrobot {
template <typename RobotLimitsType>
class ProtocolV1Parser : public ParserBase<ProtocolVersion::AGX_V1> {
 public:
  bool DecodeMessage(const struct can_frame *rx_frame,
                     AgxMessage *msg) override {
    return DecodeCanFrameV1(rx_frame, msg);
  }

  bool EncodeMessage(const AgxMessage *msg,
                     struct can_frame *tx_frame) override {
    AgxMessage msg_v1 = *msg;
    if (msg->type == AgxMsgMotionCommandV1) {
      float linear = msg->body.v1_motion_command_msg.linear;
      float angular = msg->body.v1_motion_command_msg.angular;
      float lateral = msg->body.v1_motion_command_msg.lateral;

      if (linear > RobotLimitsType::max_linear)
        linear = RobotLimitsType::max_linear;
      else if (linear < RobotLimitsType::min_linear)
        linear = RobotLimitsType::min_linear;
      if (angular > RobotLimitsType::max_angular)
        angular = RobotLimitsType::max_angular;
      else if (angular < RobotLimitsType::min_angular)
        angular = RobotLimitsType::min_angular;
      if (lateral > RobotLimitsType::max_lateral)
        lateral = RobotLimitsType::max_lateral;
      else if (lateral < RobotLimitsType::min_lateral)
        lateral = RobotLimitsType::min_lateral;

      msg_v1.body.v1_motion_command_msg.linear =
          linear / RobotLimitsType::max_linear * 100.0;
      msg_v1.body.v1_motion_command_msg.angular =
          angular / RobotLimitsType::max_angular * 100.0;
      msg_v1.body.v1_motion_command_msg.lateral =
          lateral / RobotLimitsType::max_lateral * 100.0;
    }
    return EncodeCanFrameV1(&msg_v1, tx_frame);
  }

  uint8_t CalculateChecksum(uint16_t id, uint8_t *data, uint8_t dlc) override {
    return CalcCanFrameChecksumV1(id, data, dlc);
  }

  // UART support
  bool DecodeMessage(uint8_t *data, uint8_t dlc, AgxMessage *msg) override {
    return false;
  }

  void EncodeMessage(const AgxMessage *msg, uint8_t *buf,
                     uint8_t *len) override {}

  uint8_t CalculateChecksum(uint8_t *buf, uint8_t len) override { return 0; }
};

using ScoutProtocolV1Parser = ProtocolV1Parser<ScoutV2Limits>;
using ScoutMiniProtocolV1Parser = ProtocolV1Parser<ScoutMiniLimits>;

using BunkerProtocolV1Parser = ProtocolV1Parser<BunkerLimits>;

using HunterProtocolV1Parser = ProtocolV1Parser<HunterV1Limits>;

}  // namespace westonrobot

#endif /* SCOUT_PROTOCOL_V1_PARSER_HPP */
