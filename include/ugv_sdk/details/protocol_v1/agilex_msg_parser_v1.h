/*
 * agilex_msg_parser.h
 *
 * Created on: Jul 09, 2021 22:04
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef AGILEX_MSG_PARSER_H
#define AGILEX_MSG_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __linux__
#include <linux/can.h>
#else
struct can_frame {
  uint32_t can_id;
  uint8_t can_dlc;
  uint8_t data[8] __attribute__((aligned(8)));
};
#endif

#include "ugv_sdk/details/interface/agilex_message.h"

bool DecodeCanFrameV1(const struct can_frame *rx_frame, AgxMessage *msg);
bool EncodeCanFrameV1(const AgxMessage *msg, struct can_frame *tx_frame);
uint8_t CalcCanFrameChecksumV1(uint16_t id, uint8_t *data, uint8_t dlc);

#ifdef __cplusplus
}
#endif

#endif /* AGILEX_MSG_PARSER_H */
