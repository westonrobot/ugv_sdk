
#ifndef TRACE_UART_PARSER_H
#define TRACE_UART_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "ugv_sdk/tracer/tracer_protocol.h"

bool DecodeTracerMsgFromUART(uint8_t c, UartTracerMessage *msg);
void EncodeTracerMsgToUART(const UartTracerMessage *msg, uint8_t *buf, uint8_t *len);

uint8_t CalcTracerUARTChecksum(uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* TRACE_UART_PARSER_H */
