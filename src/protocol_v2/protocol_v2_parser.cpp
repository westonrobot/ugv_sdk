/*
 * protocol_v2_parser.cpp
 *
 * Created on: Jul 08, 2021 14:53
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "ugv_sdk/details/protocol_v2/protocol_v2_parser.hpp"
#include "protocol_v2/agilex_msg_parser.h"

namespace westonrobot {
bool ProtocolV2Parser::DecodeMessage(const struct can_frame *rx_frame,
                                     AgxMessage *msg) {
  return DecodeCanFrame(rx_frame, msg);
}

void ProtocolV2Parser::EncodeMessage(const AgxMessage *msg,
                                     struct can_frame *tx_frame) {
  EncodeCanFrame(msg, tx_frame);
}

uint8_t ProtocolV2Parser::CalculateChecksum(uint16_t id, uint8_t *data,
                                            uint8_t dlc) {
  return CalcCanFrameChecksum(id, data, dlc);
}
}  // namespace westonrobot