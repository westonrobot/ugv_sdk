/* 
 * scout_uart_parser.h
 * 
 * Created on: Aug 14, 2019 12:01
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_UART_PARSER_H
#define SCOUT_UART_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "wrp_sdk/platforms/scout/scout_protocol.h"

bool DecodeScoutMsgFromUART(uint8_t c, ScoutMessage *msg);
void EncodeScoutMsgToUART(const ScoutMessage *msg, uint8_t *buf, uint8_t *len);

uint8_t CalcScoutUARTChecksum(uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* SCOUT_UART_PARSER_H */
