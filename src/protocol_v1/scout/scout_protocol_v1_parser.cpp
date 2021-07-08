/*
 * scout_protocol_v1_parser.cpp
 *
 * Created on: Jul 08, 2021 22:43
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "ugv_sdk/details/protocol_v1/scout_protocol_v1_parser.hpp"

#include "protocol_v1/scout/scout_can_parser.h"

namespace westonrobot {
bool ScoutProtocolV1Parser::DecodeMessage(const struct can_frame *rx_frame,
                                          AgxMessage *msg) {}

void ScoutProtocolV1Parser::EncodeMessage(const AgxMessage *msg,
                                          struct can_frame *tx_frame) {}

uint8_t ScoutProtocolV1Parser::CalculateChecksum(uint16_t id, uint8_t *data,
                                                 uint8_t dlc) {}

// UART support
bool ScoutProtocolV1Parser::DecodeMessage(uint8_t *data, uint8_t dlc,
                                          AgxMessage *msg) {}

void ScoutProtocolV1Parser::EncodeMessage(const AgxMessage *msg, uint8_t *buf,
                                          uint8_t *len) {}

uint8_t ScoutProtocolV1Parser::CalculateChecksum(uint8_t *buf, uint8_t len) {}
}  // namespace westonrobot
