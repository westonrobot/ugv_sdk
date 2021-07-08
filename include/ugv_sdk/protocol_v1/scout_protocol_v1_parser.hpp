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

#include "ugv_sdk/interface/parser_interface.hpp"

namespace westonrobot {
class ScoutProtocolV1Parser : public ParserInterface {
 public:
  bool DecodeMessage(const struct can_frame *rx_frame,
                     AgxMessage *msg) override;
  void EncodeMessage(const AgxMessage *msg,
                     struct can_frame *tx_frame) override;
  uint8_t CalculateChecksum(uint16_t id, uint8_t *data, uint8_t dlc) override;

  // UART support
  bool DecodeMessage(uint8_t *data, uint8_t dlc, AgxMessage *msg) override;
  void EncodeMessage(const AgxMessage *msg, uint8_t *buf,
                     uint8_t *len) override;
  uint8_t CalculateChecksum(uint8_t *buf, uint8_t len) override;
};
}  // namespace westonrobot

#endif /* SCOUT_PROTOCOL_V1_PARSER_HPP */
