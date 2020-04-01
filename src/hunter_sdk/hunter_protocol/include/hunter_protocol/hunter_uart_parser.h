/* 
 * hunter_uart_parser.h
 * 
 * Created on: Apr 01, 2020 09:48
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef HUNTER_UART_PARSER_H
#define HUNTER_UART_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "hunter_protocol/hunter_protocol.h"

bool DecodeHunterMsgFromUART(uint8_t c, HunterMessage *msg);
void EncodeHunterMsgToUART(const HunterMessage *msg, uint8_t *buf, uint8_t *len);

uint8_t CalcHunterUARTChecksum(uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* HUNTER_UART_PARSER_H */
