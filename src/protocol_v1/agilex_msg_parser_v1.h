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

#include "protocol_v1/agilex_message_v1.h"

bool DecodeCanFrame(const struct can_frame *rx_frame, AgxMessageV1 *msg);
void EncodeCanFrame(const AgxMessageV1 *msg, struct can_frame *tx_frame);
uint8_t CalcCanFrameChecksum(uint16_t id, uint8_t *data, uint8_t dlc);

#ifdef __cplusplus
}
#endif

#endif /* AGILEX_MSG_PARSER_H */
