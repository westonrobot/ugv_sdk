/*
 * hunter_protocol_v1_parser.hpp
 *
 * Created on: Jul 09, 2021 22:20
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef HUNTER_PROTOCOL_V1_PARSER_HPP
#define HUNTER_PROTOCOL_V1_PARSER_HPP

#include "ugv_sdk/details/interface/parser_interface.hpp"

namespace westonrobot {
class HunterProtocolV1Parser : public ParserInterface {
 public:
  bool DecodeMessage(const struct can_frame *rx_frame,
                     AgxMessage *msg) override;
  void EncodeMessage(const AgxMessage *msg,
                     struct can_frame *tx_frame) override;
  uint8_t CalculateChecksum(uint16_t id, uint8_t *data, uint8_t dlc) override;
};
}  // namespace westonrobot

#endif /* HUNTER_PROTOCOL_V1_PARSER_HPP */
