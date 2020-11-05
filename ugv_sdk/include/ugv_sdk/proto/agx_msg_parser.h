/*
 * agx_msg_parser.h
 *
 * Created on: Nov 04, 2020 13:40
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef AGX_MSG_PARSER_H
#define AGX_MSG_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ugv_sdk/proto/agx_protocol_v2.h"

#ifdef __linux__
#include <linux/can.h>
#else
struct can_frame {
  uint32_t can_id;
  uint8_t can_dlc;
  uint8_t data[8] __attribute__((aligned(8)));
};
#endif

#pragma pack(push, 1)
typedef enum {
  AgxMsgUnkonwn = 0x00,
  // command
  AgxMsgMotionCommand = 0x01,
  AgxMsgLightCommand = 0x02,
  AgxMsgCtrlModeSelect = 0x03,
  AgxMsgFaultByteReset = 0x04,
  // state feedback
  AgxMsgSystemState = 0x21,
  AgxMsgMotionState = 0x22,
  AgxMsgLightState = 0x23,
  AgxMsgRcState = 0x24,
  AgxMsgActuatorHSState = 0x25,
  AgxMsgActuatorLSState = 0x26,
  AgxMsgOdometry = 0x27
} MsgType;

typedef struct {
  MsgType type;
  union {
    // command
    MotionCommandMessage motion_command_msg;
    LightCommandMessage light_command_msg;
    CtrlModeSelectMessage ctrl_mode_select_msg;
    StateResetMessage state_reset_msg;
    // state feedback
    SystemStateMessage system_state_msg;
    MotionStateMessage motion_state_msg;
    LightStateMessage light_state_msg;
    RcStateMessage rc_state_msg;
    ActuatorHSStateMessage actuator_hs_state_msg;
    ActuatorLSStateMessage actuator_ls_state_msg;
    OdometryMessage odometry_msg;
  } body;
} AgxMessage;
#pragma pack(pop)

bool DecodeCanFrame(const struct can_frame *rx_frame, AgxMessage *msg);
void EncodeCanFrame(const AgxMessage *msg, struct can_frame *tx_frame);
uint8_t CalcCanFrameChecksum(uint16_t id, uint8_t *data, uint8_t dlc);

#ifdef __cplusplus
}
#endif

#endif /* AGX_PARSER_H */
