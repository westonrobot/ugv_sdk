/* 
 * hunter_can_parser.h
 * 
 * Created on: Jan 02, 2020 12:36
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef HUNTER_CAN_PARSER_H
#define HUNTER_CAN_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "wrp_sdk/platforms/hunter/hunter_protocol.h"

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

bool DecodeHunterMsgFromCAN(const struct can_frame *rx_frame, HunterMessage *msg);
void EncodeHunterMsgToCAN(const HunterMessage *msg, struct can_frame *tx_frame);

uint8_t CalcHunterCANChecksum(uint16_t id, uint8_t *data, uint8_t dlc);

#ifdef __cplusplus
}
#endif

#endif /* HUNTER_CAN_PARSER_H */
