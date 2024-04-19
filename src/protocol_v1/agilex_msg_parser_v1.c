/*
 * agilex_msg_parser.c
 *
 * Created on: Jul 09, 2021 22:04
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "ugv_sdk/details/protocol_v1//agilex_msg_parser_v1.h"

#include "protocol_v1/agilex_protocol_v1.h"

#include "stdio.h"
#include "string.h"

bool DecodeCanFrameV1(const struct can_frame *rx_frame, AgxMessage *msg) {
  bool ret = true;
  // if checksum not correct
  if (!CalcCanFrameChecksumV1(rx_frame->can_id, (uint8_t *)rx_frame->data,
                              rx_frame->can_dlc)) {
    // printf("wrong checksum for id: %x-------------->\n", rx_frame->can_id);
    return false;
  }

  switch (rx_frame->can_id) {
    case CAN_MSG_SYSTEM_STATE_ID: {
      msg->type = AgxMsgSystemState;
      msg->body.system_state_msg.vehicle_state = rx_frame->data[0];
      msg->body.system_state_msg.control_mode = rx_frame->data[1];
      msg->body.system_state_msg.battery_voltage =
          ((((uint16_t)rx_frame->data[2]) << 8) | (uint16_t)rx_frame->data[3]) /
          10.0f;
      msg->body.system_state_msg.error_code =
          ((((uint16_t)rx_frame->data[4]) << 8) | (uint16_t)rx_frame->data[5]);
      break;
    }
    case CAN_MSG_MOTION_STATE_ID: {
      msg->type = AgxMsgMotionState;
      msg->body.motion_state_msg.linear_velocity =
          ((int16_t)((((uint16_t)rx_frame->data[0]) << 8) |
                     (uint16_t)rx_frame->data[1])) /
          1000.0f;
      msg->body.motion_state_msg.angular_velocity =
          ((int16_t)((((uint16_t)rx_frame->data[2]) << 8) |
                     (uint16_t)rx_frame->data[3])) /
          1000.0f;
      msg->body.motion_state_msg.lateral_velocity =
          ((int16_t)((((uint16_t)rx_frame->data[4]) << 8) |
                     (uint16_t)rx_frame->data[5])) /
          1000.0f;
      break;
    }
    case CAN_MSG_LIGHT_STATE_ID: {
      msg->type = AgxMsgLightState;
      LightStateFrame *frame = (LightStateFrame *)(rx_frame->data);
      msg->body.light_command_msg.enable_cmd_ctrl =
          (frame->enable_cmd_ctrl != 0) ? true : false;
      msg->body.light_command_msg.front_light.mode = frame->front_mode;
      msg->body.light_command_msg.front_light.custom_value =
          frame->front_custom;
      msg->body.light_command_msg.rear_light.mode = frame->rear_mode;
      msg->body.light_command_msg.rear_light.custom_value = frame->rear_custom;
      break;
    }
    case CAN_MSG_VALUE_SET_STATE_ID: {
      msg->type = AgxMsgValueSetStateV1;
      if (rx_frame->data[0] == 0xaa)
        msg->body.v1_value_set_state_msg.set_neutral = true;
      else
        msg->body.v1_value_set_state_msg.set_neutral = false;
      break;
    }
    case CAN_MSG_ACTUATOR1_STATE_ID:
    case CAN_MSG_ACTUATOR2_STATE_ID:
    case CAN_MSG_ACTUATOR3_STATE_ID:
    case CAN_MSG_ACTUATOR4_STATE_ID: {
      msg->type = AgxMsgActuatorStateV1;
      msg->body.v1_actuator_state_msg.motor_id =
          rx_frame->can_id - CAN_MSG_ACTUATOR1_STATE_ID;
      msg->body.v1_actuator_state_msg.current = (int16_t)((
          ((uint16_t)rx_frame->data[0]) << 8) | (uint16_t)rx_frame->data[1]) * 0.1;
      msg->body.v1_actuator_state_msg.rpm = (int16_t)((
          (((uint16_t)rx_frame->data[2]) << 8) | (uint16_t)rx_frame->data[3]));
      msg->body.v1_actuator_state_msg.driver_temp = rx_frame->data[4];
      msg->body.v1_actuator_state_msg.motor_temp = rx_frame->data[5];
      break;
    }
    default:
      ret = false;
      break;
  }

  return ret;
}

bool EncodeCanFrameV1(const AgxMessage *msg, struct can_frame *tx_frame) {
  bool ret = true;
  switch (msg->type) {
    case AgxMsgMotionCommandV1: {
      static uint8_t count = 0;
      tx_frame->can_id = CAN_MSG_MOTION_COMMAND_ID;
      tx_frame->can_dlc = 8;
      MotionCommandFrame frame;
      frame.control_mode = CTRL_MODE_CMD_CAN;
      frame.error_clear_byte = ERROR_CLR_NONE;
      frame.linear_percentage =
          (int8_t)(msg->body.v1_motion_command_msg.linear);
      frame.angular_percentage =
          (int8_t)(msg->body.v1_motion_command_msg.angular);
      frame.lateral_percentage =
          (int8_t)(msg->body.v1_motion_command_msg.lateral);
      frame.reserved0 = 0;
      frame.count = count++;
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      tx_frame->data[7] = CalcCanFrameChecksumV1(
          tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
      break;
    }
    case AgxMsgValueSetCommandV1: {
      static uint8_t count = 0;
      tx_frame->can_id = CAN_MSG_VALUE_SET_COMMAND_ID;
      tx_frame->can_dlc = 8;
      ValueSetCommandFrame frame;
      frame.set_neutral =
          msg->body.v1_value_set_command_msg.set_neutral ? 0xaa : 0x00;
      frame.reserved0 = 0x00;
      frame.reserved1 = 0x00;
      frame.reserved2 = 0x00;
      frame.reserved3 = 0x00;
      frame.reserved4 = 0x00;
      frame.count = count++;
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      tx_frame->data[7] = CalcCanFrameChecksumV1(
          tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
      break;
    }
    case AgxMsgLightCommand: {
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
    default: {
      ret = false;
      break;
    }
  }
  return ret;
}

uint8_t CalcCanFrameChecksumV1(uint16_t id, uint8_t *data, uint8_t dlc) {
  uint8_t checksum = 0x00;
  checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
  for (int i = 0; i < (dlc - 1); ++i) checksum += data[i];
  return checksum;
}
