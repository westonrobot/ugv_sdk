/*
 * agilex_msg_parser.c
 *
 * Created on: Jul 09, 2021 22:04
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "ugv_sdk/details/protocol_v1/agilex_protocol_v1.h"
#include "ugv_sdk/details/protocol_v1//agilex_msg_parser_v1.h"

#include "stdio.h"
#include "string.h"

bool DecodeCanFrameV1(const struct can_frame *rx_frame, AgxMessage *msg) {
  switch (rx_frame->can_id) {
    case CAN_MSG_SYSTEM_STATE_ID: {
      msg->type = AgxMsgSystemState;
      // msg->system_status_msg.id = CAN_MSG_SYSTEM_STATUS_STATUS_ID;
      //   memcpy(msg->body.system_state_msg.data.raw, rx_frame->data,
      //          rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_MOTION_STATE_ID: {
      msg->type = AgxMsgMotionState;
      // msg->motion_status_msg.id = CAN_MSG_MOTION_CONTROL_STATUS_ID;
      //   memcpy(msg->body.motion_state_msg.data.raw, rx_frame->data,
      //          rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_LIGHT_STATE_ID: {
      msg->type = AgxMsgLightState;
      // msg->light_status_msg.id = CAN_MSG_LIGHT_CONTROL_STATUS_ID;
      //   memcpy(msg->body.light_state_msg.data.raw, rx_frame->data,
      //          rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_ACTUATOR1_STATE_ID: {
      msg->type = AgxMsgActuatorStateV1;
      // msg->motor_driver_status_msg.id = CAN_MSG_MOTOR1_DRIVER_STATUS_ID;
      //   msg->body.v1_actuator_state_msg.motor_id = SCOUT_MOTOR1_ID;
      //   memcpy(msg->body.motor_driver_status_msg.data.raw, rx_frame->data,
      //          rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_ACTUATOR2_STATE_ID: {
      msg->type = AgxMsgActuatorStateV1;
      // msg->motor_driver_status_msg.id = CAN_MSG_MOTOR2_DRIVER_STATUS_ID;
      //   msg->body.v1_actuator_state_msg.motor_id = SCOUT_MOTOR2_ID;
      //   memcpy(msg->body.motor_driver_status_msg.data.raw, rx_frame->data,
      //          rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_ACTUATOR3_STATE_ID: {
      msg->type = AgxMsgActuatorStateV1;
      // msg->motor_driver_status_msg.id = CAN_MSG_MOTOR3_DRIVER_STATUS_ID;
      //   msg->body.v1_actuator_state_msg.motor_id = SCOUT_MOTOR3_ID;
      //   memcpy(msg->body.motor_driver_status_msg.data.raw, rx_frame->data,
      //          rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_ACTUATOR4_STATE_ID: {
      msg->type = AgxMsgActuatorStateV1;
      // msg->motor_driver_status_msg.id = CAN_MSG_MOTOR4_DRIVER_STATUS_ID;
      //   msg->body.v1_actuator_state_msg.motor_id = SCOUT_MOTOR4_ID;
      //   memcpy(msg->body.motor_driver_status_msg.data.raw, rx_frame->data,
      //          rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    default:
      break;
  }

  return true;
}

void EncodeCanFrameV1(const AgxMessage *msg, struct can_frame *tx_frame) {
  switch (msg->type) {
    case AgxMsgMotionCommandV1: {
      static uint8_t count = 0;
      tx_frame->can_id = CAN_MSG_MOTION_COMMAND_ID;
      tx_frame->can_dlc = 8;
      tx_frame->data[0] = CTRL_MODE_CMD_CAN;
      tx_frame->data[1] = ERROR_CLR_NONE;
      tx_frame->data[2] =
          (int8_t)(msg->body.motion_command_msg.linear_velocity * 100);
      tx_frame->data[3] =
          (int8_t)(msg->body.motion_command_msg.angular_velocity * 100);
      tx_frame->data[4] = 0;
      tx_frame->data[5] = 0;
      tx_frame->data[6] = count++;
      tx_frame->data[7] = CalcCanFrameChecksumV1(
          tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
      break;
    }
    case AgxMsgValueSetCommandV1: {
      static uint8_t count = 0;
      tx_frame->can_id = CAN_MSG_LIGHT_COMMAND_ID;
      tx_frame->can_dlc = 8;
      LightCommandFrame frame;
      if (msg->body.light_command_msg.enable_cmd_ctrl) {
        frame.enable_cmd_ctrl = LIGHT_ENABLE_CMD_CTRL;
        frame.front_mode = msg->body.light_command_msg.front_light.mode;
        frame.front_custom =
            msg->body.light_command_msg.front_light.custom_value;
        frame.rear_mode = msg->body.light_command_msg.rear_light.mode;
        frame.rear_custom = msg->body.light_command_msg.rear_light.custom_value;
      } else {
        frame.enable_cmd_ctrl = LIGHT_DISABLE_CMD_CTRL;
        frame.front_mode = 0;
        frame.front_custom = 0;
        frame.rear_mode = 0;
        frame.rear_custom = 0;
      }
      frame.reserved0 = 0;
      frame.count = count++;
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      tx_frame->data[7] = CalcCanFrameChecksumV1(
          tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
      break;
    }
    case AgxMsgLightCommand: {
      break;
    }
    default:
      break;
  }
}

uint8_t CalcCanFrameChecksumV1(uint16_t id, uint8_t *data, uint8_t dlc) {
  uint8_t checksum = 0x00;
  checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
  for (int i = 0; i < (dlc - 1); ++i) checksum += data[i];
  return checksum;
}
