/* 
 * tracer_can_parser.h
 * 
 * Created on: Apr 14, 2020 10:52
 * Description: 
 * 
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */ 

#ifndef TRACER_CAN_PARSER_H
#define TRACER_CAN_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "ugv_sdk/tracer/tracer_protocol.h"

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

bool DecodeTracerMsgFromCAN(const struct can_frame *rx_frame, TracerMessage *msg);
void EncodeTracerMsgToCAN(const TracerMessage *msg, struct can_frame *tx_frame);

uint8_t CalcTracerCANChecksum(uint16_t id, uint8_t *data, uint8_t dlc);

#ifdef __cplusplus
}
#endif

#endif /* TRACER_CAN_PARSER_H */
