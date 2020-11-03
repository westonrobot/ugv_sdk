/* 
 * bunker_can_parser.h
 * 
 * Created on: Aug 31, 2019 04:23
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef BUNKER_CAN_PARSER_H
#define BUNKER_CAN_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "ugv_sdk/bunker/bunker_protocol.h"

#ifdef __linux__
#include <linux/can.h>
#else
struct can_frame
{
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t data[8]__attribute__((aligned(8)));
};
#endif

bool DecodeBunkerMsgFromCAN(const struct can_frame *rx_frame, BunkerMessage *msg);
void EncodeBunkerMsgToCAN(const BunkerMessage *msg, struct can_frame *tx_frame);

uint8_t CalcBunkerCANChecksum(uint16_t id, uint8_t *data, uint8_t dlc);

#ifdef __cplusplus
}
#endif

#endif /* BUNKER_CAN_PARSER_H */
