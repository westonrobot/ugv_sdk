/*
 * agx_msg_parser.c
 *
 * Created on: Aug 31, 2019 04:25
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "ugv_sdk/proto/agx_msg_parser.h"

#include "string.h"

bool DecodeCanFrame(const struct can_frame *rx_frame, AgxMessage *msg) {
  msg->type = AgxMsgUnkonwn;

  switch (rx_frame->can_id) {
    // command frame
    case CAN_MSG_MOTION_COMMAND_ID: {
      msg->type = AgxMsgMotionCommand;
      memcpy(msg->body.motion_command_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_LIGHT_COMMAND_ID: {
      msg->type = AgxMsgLightCommand;
      memcpy(msg->body.light_command_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_RC_STATE_ID: {
      msg->type = AgxMsgRcState;
      memcpy(msg->body.rc_state_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_CTRL_MODE_SELECT_ID: {
      msg->type = AgxMsgCtrlModeSelect;
      memcpy(msg->body.ctrl_mode_select_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_STATE_RESET_ID: {
      msg->type = AgxMsgFaultByteReset;
      memcpy(msg->body.state_reset_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    // state feedback frame
    case CAN_MSG_SYSTEM_STATE_ID: {
      msg->type = AgxMsgSystemState;
      memcpy(msg->body.system_state_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_MOTION_STATE_ID: {
      msg->type = AgxMsgMotionState;
      memcpy(msg->body.motion_state_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_LIGHT_STATE_ID: {
      msg->type = AgxMsgLightState;
      memcpy(msg->body.light_state_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_ODOMETRY_ID: {
      msg->type = AgxMsgOdometry;
      memcpy(msg->body.odometry_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_ACTUATOR1_HS_STATE_ID:
    case CAN_MSG_ACTUATOR2_HS_STATE_ID:
    case CAN_MSG_ACTUATOR3_HS_STATE_ID:
    case CAN_MSG_ACTUATOR4_HS_STATE_ID: {
      msg->type = AgxMsgActuatorLSState;
      msg->body.actuator_hs_state_msg.motor_id =
          (uint8_t)(rx_frame->can_id - CAN_MSG_ACTUATOR1_HS_STATE_ID);
      memcpy(msg->body.actuator_hs_state_msg.data.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_ACTUATOR1_LS_STATE_ID:
    case CAN_MSG_ACTUATOR2_LS_STATE_ID:
    case CAN_MSG_ACTUATOR3_LS_STATE_ID:
    case CAN_MSG_ACTUATOR4_LS_STATE_ID: {
      msg->type = AgxMsgActuatorLSState;
      msg->body.actuator_ls_state_msg.motor_id =
          (uint8_t)(rx_frame->can_id - CAN_MSG_ACTUATOR1_LS_STATE_ID);
      memcpy(msg->body.actuator_ls_state_msg.data.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    default:
      break;
  }

  return true;
}

void EncodeCanFrame(const AgxMessage *msg, struct can_frame *tx_frame) {
  switch (msg->type) {
    // command frame
    case AgxMsgMotionCommand: {
      tx_frame->can_id = CAN_MSG_MOTION_COMMAND_ID;
      tx_frame->can_dlc = 8;
      memcpy(tx_frame->data, msg->body.motion_command_msg.raw,
             tx_frame->can_dlc);
      break;
    }
    case AgxMsgLightCommand: {
      tx_frame->can_id = CAN_MSG_LIGHT_COMMAND_ID;
      tx_frame->can_dlc = 8;
      memcpy(tx_frame->data, msg->body.light_command_msg.raw,
             tx_frame->can_dlc);
      break;
    }
    case AgxMsgCtrlModeSelect: {
      tx_frame->can_id = CAN_MSG_CTRL_MODE_SELECT_ID;
      tx_frame->can_dlc = 8;
      memcpy(tx_frame->data, msg->body.ctrl_mode_select_msg.raw,
             tx_frame->can_dlc);
      break;
    }
    case AgxMsgFaultByteReset: {
      tx_frame->can_id = CAN_MSG_STATE_RESET_ID;
      tx_frame->can_dlc = 8;
      memcpy(tx_frame->data, msg->body.state_reset_msg.raw, tx_frame->can_dlc);
      break;
    }
    // state feedback frame
    case AgxMsgSystemState: {
      tx_frame->can_id = CAN_MSG_SYSTEM_STATE_ID;
      tx_frame->can_dlc = 8;
      memcpy(tx_frame->data, msg->body.system_state_msg.raw, tx_frame->can_dlc);
      break;
    }
    case AgxMsgMotionState: {
      tx_frame->can_id = CAN_MSG_MOTION_STATE_ID;
      tx_frame->can_dlc = 8;
      memcpy(tx_frame->data, msg->body.motion_state_msg.raw, tx_frame->can_dlc);
      break;
    }
    case AgxMsgLightState: {
      tx_frame->can_id = CAN_MSG_LIGHT_STATE_ID;
      tx_frame->can_dlc = 8;
      memcpy(tx_frame->data, msg->body.light_state_msg.raw, tx_frame->can_dlc);
      break;
    }
    case AgxMsgOdometry: {
      tx_frame->can_id = CAN_MSG_ODOMETRY_ID;
      tx_frame->can_dlc = 8;
      memcpy(tx_frame->data, msg->body.odometry_msg.raw, tx_frame->can_dlc);
      break;
    }
    case AgxMsgActuatorHSState: {
      switch (msg->body.actuator_hs_state_msg.motor_id) {
        case ACTUATOR1_ID: {
          tx_frame->can_id = CAN_MSG_ACTUATOR1_HS_STATE_ID;
          break;
        }
        case ACTUATOR2_ID: {
          tx_frame->can_id = CAN_MSG_ACTUATOR2_HS_STATE_ID;
          break;
        }
        case ACTUATOR3_ID: {
          tx_frame->can_id = CAN_MSG_ACTUATOR3_HS_STATE_ID;
          break;
        }
        case ACTUATOR4_ID: {
          tx_frame->can_id = CAN_MSG_ACTUATOR4_HS_STATE_ID;
          break;
        }
      }
      tx_frame->can_dlc = 8;
      memcpy(tx_frame->data, msg->body.actuator_hs_state_msg.data.raw,
             tx_frame->can_dlc);
      break;
    }
    case AgxMsgActuatorLSState: {
      switch (msg->body.actuator_ls_state_msg.motor_id) {
        case ACTUATOR1_ID: {
          tx_frame->can_id = CAN_MSG_ACTUATOR1_LS_STATE_ID;
          break;
        }
        case ACTUATOR2_ID: {
          tx_frame->can_id = CAN_MSG_ACTUATOR2_LS_STATE_ID;
          break;
        }
        case ACTUATOR3_ID: {
          tx_frame->can_id = CAN_MSG_ACTUATOR3_LS_STATE_ID;
          break;
        }
        case ACTUATOR4_ID: {
          tx_frame->can_id = CAN_MSG_ACTUATOR4_LS_STATE_ID;
          break;
        }
      }
      tx_frame->can_dlc = 8;
      memcpy(tx_frame->data, msg->body.actuator_ls_state_msg.data.raw,
             tx_frame->can_dlc);
      break;
    }
    default:
      break;
  }
  //   tx_frame->data[7] =
  //       CalcCanFrameChecksum(tx_frame->can_id, tx_frame->data,
  //       tx_frame->can_dlc);
}

uint8_t CalcCanFrameChecksum(uint16_t id, uint8_t *data, uint8_t dlc) {
  uint8_t checksum = 0x00;
  checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
  for (int i = 0; i < (dlc - 1); ++i) checksum += data[i];
  return checksum;
}
