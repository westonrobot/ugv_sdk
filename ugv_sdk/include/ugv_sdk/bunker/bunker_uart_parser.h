/* 
 * scout_uart_parser.h
 * 
 * Created on: Aug 14, 2019 12:01
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef BUNKER_UART_PARSER_H
#define BUNKER_UART_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "ugv_sdk/bunker/bunker_protocol.h"

bool DecodeBunkerMsgFromUART(uint8_t c, BunkerMessage *msg);
void EncodeBunkerMsgToUART(const BunkerMessage *msg, uint8_t *buf, uint8_t *len);

uint8_t CalcBunkerUARTChecksum(uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* BUNKER_UART_PARSER_H */
