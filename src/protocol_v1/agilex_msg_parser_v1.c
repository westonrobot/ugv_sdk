/*
 * agilex_msg_parser.c
 *
 * Created on: Jul 09, 2021 22:04
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "agilex_protocol_v1.h"
#include "protocol_v1/agilex_msg_parser_v1.h"

#include "stdio.h"
#include "string.h"

bool DecodeCanFrame(const struct can_frame *rx_frame, AgxMessageV1 *msg) {}
void EncodeCanFrame(const AgxMessageV1 *msg, struct can_frame *tx_frame) {}
uint8_t CalcCanFrameChecksum(uint16_t id, uint8_t *data, uint8_t dlc) {}
