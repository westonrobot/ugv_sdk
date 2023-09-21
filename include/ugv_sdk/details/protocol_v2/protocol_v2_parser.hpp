/*
 * protocol_v2_parser.hpp
 *
 * Created on: Jul 08, 2021 14:51
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef PROTOCOL_V2_PARSER_HPP
#define PROTOCOL_V2_PARSER_HPP

#include "ugv_sdk/details/parser_base.hpp"

namespace westonrobot {
class ProtocolV2Parser : public ParserBase<ProtocolVersion::AGX_V2> {
 public:
  bool DecodeMessage(const struct can_frame *rx_frame,
                     AgxMessage *msg) override;
  bool EncodeMessage(const AgxMessage *msg,
                     struct can_frame *tx_frame) override;
  uint8_t CalculateChecksum(uint16_t id, uint8_t *data, uint8_t dlc) override;
};
}  // namespace westonrobot

#endif /* PROTOCOL_V2_PARSER_HPP */
