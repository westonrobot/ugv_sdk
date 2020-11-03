/* 
 * hunter_can_parser.c
 * 
 * Created on: Jan 02, 2020 12:40
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#include "ugv_sdk/hunter_v2/hunter_can_parser.h"
#include "string.h"

static void EncodeHunterMotionControlMsgToCAN(const MotionControlMessage *msg, struct can_frame *tx_frame);
static void EncodeHunterControlModeMsgToCAN(const ModSelectMessage *msg,struct can_frame *tx_frame);
static void EncodeHunterControlParkMsgToCAN(const ParkControlMessage *msg,struct can_frame *tx_frame);

bool DecodeHunterMsgFromCAN(const struct can_frame *rx_frame, HunterMessage *msg)
{
    msg->type = HunterMsgNone;

    switch (rx_frame->can_id)
    {
    // in the current implementation, both MsgType and can_frame include 8 * uint8_t
    case CAN_MSG_MOTION_CONTROL_STATUS_ID:
    {
        msg->type = HunterMotionStatusMsg;
        memcpy(msg->body.motion_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_SYSTEM_STATUS_STATUS_ID:
    {
        msg->type = HunterSystemStatusMsg;
        memcpy(msg->body.system_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR1_HEIGHT_DRIVER_STATUS_ID:
    {
        msg->type = HunterMotorDriverHeightSpeedStatusMsg;
        msg->body.motor_driver_height_speed_status_msg.motor_id = HUNTER_MOTOR1_ID;
        memcpy(msg->body.motor_driver_height_speed_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR2_HEIGHT_DRIVER_STATUS_ID:
    {
        msg->type = HunterMotorDriverHeightSpeedStatusMsg;
        msg->body.motor_driver_height_speed_status_msg.motor_id = HUNTER_MOTOR2_ID;
        memcpy(msg->body.motor_driver_height_speed_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR3_HEIGHT_DRIVER_STATUS_ID:
    {
        msg->type = HunterMotorDriverHeightSpeedStatusMsg;
        msg->body.motor_driver_height_speed_status_msg.motor_id = HUNTER_MOTOR3_ID;
        memcpy(msg->body.motor_driver_height_speed_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR1_LOW_DRIVER_STATUS_ID:
    {
        msg->type = HunterMotorDriverLowSpeedStatusMsg;
        msg->body.motor_driver_low_speed_status_msg.motor_id = HUNTER_MOTOR1_ID;
        memcpy(msg->body.motor_driver_low_speed_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR2_LOW_DRIVER_STATUS_ID:
    {
        msg->type = HunterMotorDriverLowSpeedStatusMsg;
        msg->body.motor_driver_low_speed_status_msg.motor_id = HUNTER_MOTOR2_ID;
        memcpy(msg->body.motor_driver_low_speed_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR3_LOW_DRIVER_STATUS_ID:
    {
        msg->type = HunterMotorDriverLowSpeedStatusMsg;
        msg->body.motor_driver_low_speed_status_msg.motor_id = HUNTER_MOTOR3_ID;
        memcpy(msg->body.motor_driver_low_speed_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTION_CONTROL_CMD_ID:
    {
        msg->type = HunterMotionControlMsg;
        memcpy(msg->body.motion_control_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_PARK_CONTROL_ID:
    {
         msg->type = HunterParkControlMsg;
         memcpy(msg->body.park_control_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
         break;
    }
    case CAN_MSG_SELECT_CONTROL_MODE_ID:
    {
         msg->type = HunterControlModeMsg;
         memcpy(msg->body.mode_cmd_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
         break;
    }
    default:
        break;
    }

    return true;
}

void EncodeHunterMsgToCAN(const HunterMessage *msg, struct can_frame *tx_frame)
{
    switch (msg->type)
    {
    // in the current implementation, both MsgType and can_frame include 8 * uint8_t
    case HunterMotionStatusMsg:
    {
        tx_frame->can_id = CAN_MSG_MOTION_CONTROL_STATUS_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.motion_status_msg.data.raw, tx_frame->can_dlc);
        break;
    }
    case HunterSystemStatusMsg:
    {
        tx_frame->can_id = CAN_MSG_SYSTEM_STATUS_STATUS_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.system_status_msg.data.raw, tx_frame->can_dlc);
        break;
    }
    case HunterMotionControlMsg:
    {
        EncodeHunterMotionControlMsgToCAN(&(msg->body.motion_control_msg), tx_frame);
        break;
    }
    case HunterControlModeMsg:
    {
        EncodeHunterControlModeMsgToCAN(&(msg->body.mode_cmd_msg), tx_frame);
        break;
    }
    case HunterParkControlMsg:
    {
        EncodeHunterControlParkMsgToCAN(&(msg->body.park_control_msg), tx_frame);

        break;
    }
    default:
        break;
    }
}

void EncodeHunterMotionControlMsgToCAN(const MotionControlMessage *msg, struct can_frame *tx_frame)
{
    tx_frame->can_id = CAN_MSG_MOTION_CONTROL_CMD_ID;
    tx_frame->can_dlc = 8;
    memcpy(tx_frame->data, msg->data.raw, tx_frame->can_dlc);
}
void EncodeHunterControlModeMsgToCAN(const ModSelectMessage *msg,struct can_frame *tx_frame)
{
    tx_frame->can_id = CAN_MSG_SELECT_CONTROL_MODE_ID;
    tx_frame->can_dlc = 8;
    memcpy(tx_frame->data, msg->data.raw, tx_frame->can_dlc);
}
void EncodeHunterControlParkMsgToCAN(const ParkControlMessage *msg,struct can_frame *tx_frame)
{
    tx_frame->can_id = CAN_MSG_PARK_CONTROL_ID;
    tx_frame->can_dlc = 8;
    memcpy(tx_frame->data, msg->data.raw, tx_frame->can_dlc);
}

uint8_t CalcHunterCANChecksum(uint16_t id, uint8_t *data, uint8_t dlc)
{
    uint8_t checksum = 0x00;
    checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
    for (int i = 0; i < (dlc - 1); ++i)
        checksum += data[i];
    return checksum;
}
