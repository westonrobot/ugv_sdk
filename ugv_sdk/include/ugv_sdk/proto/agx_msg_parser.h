/*
 * agx_msg_parser.h
 *
 * Created on: Nov 04, 2020 13:40
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef AGX_MSG_PARSER_H
#define AGX_MSG_PARSER_H

// #ifdef __cplusplus
// extern "C" {
// #endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ugv_sdk/proto/agx_protocol_v2.h"

#ifdef __linux__
#include <linux/can.h>
#else
struct can_frame {
  uint32_t can_id;
  uint8_t can_dlc;
  uint8_t data[8] __attribute__((aligned(8)));
};
#endif

bool DecodeScoutMsgFromCAN(const struct can_frame *rx_frame, AgxMessage *msg);
void EncodeScoutMsgToCAN(const AgxMessage *msg, struct can_frame *tx_frame);
uint8_t CalcScoutCANChecksum(uint16_t id, uint8_t *data, uint8_t dlc);

bool DecodeScoutMsgFromUART(uint8_t c, AgxMessage *msg);
void EncodeScoutMsgToUART(const AgxMessage *msg, uint8_t *buf, uint8_t *len);
uint8_t CalcScoutUARTChecksum(uint8_t *buf, uint8_t len);

// #ifdef __cplusplus
// }
// #endif

#endif /* AGX_PARSER_H */
