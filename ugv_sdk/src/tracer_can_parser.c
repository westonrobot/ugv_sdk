/* 
 * tracer_can_parser.c
 * 
 * Created on: Apr 14, 2020 10:35
 * Description: 
 * 
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */ 

#include "tracer_protocol/tracer_can_parser.h"

#include "string.h"

static void EncodeTracerMotionControlMsgToCAN(const MotionCmdMessage *msg, struct can_frame *tx_frame);

bool DecodeTracerMsgFromCAN(const struct can_frame *rx_frame, TracerMessage *msg)
{
    msg->type = TracerMsgNone;

    switch (rx_frame->can_id)
    {
    // in the current implementation, both MsgType and can_frame include 8 * uint8_t
    case CAN_MSG_MOTION_STATUS_ID:
    {
        msg->type = TracerMotionStatusMsg;
        memcpy(msg->body.motion_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_SYSTEM_STATUS_STATUS_ID:
    {
        msg->type = TracerSystemStatusMsg;
        memcpy(msg->body.system_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR1_DRIVER_STATUS_ID:
    {
        msg->type = TracerMotorDriverStatusMsg;
        msg->body.motor_driver_status_msg.motor_id = TRACER_MOTOR1_ID;
        memcpy(msg->body.motor_driver_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTOR2_DRIVER_STATUS_ID:
    {
        msg->type = TracerMotorDriverStatusMsg;
        msg->body.motor_driver_status_msg.motor_id = TRACER_MOTOR2_ID;
        memcpy(msg->body.motor_driver_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    // in the current implementation, both MsgType and can_frame include 8 * uint8_t
    case CAN_MSG_MOTION_CMD_ID:
    {
        msg->type = TracerMotionCmdMsg;
        memcpy(msg->body.motion_cmd_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    default:
        break;
    }

    return true;
}

void EncodeTracerMsgToCAN(const TracerMessage *msg, struct can_frame *tx_frame)
{
    switch (msg->type)
    {
    // in the current implementation, both MsgType and can_frame include 8 * uint8_t
    case TracerMotionStatusMsg:
    {
        tx_frame->can_id = CAN_MSG_MOTION_STATUS_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.motion_status_msg.data.raw, tx_frame->can_dlc);
        break;
    }
    case TracerSystemStatusMsg:
    {
        tx_frame->can_id = CAN_MSG_SYSTEM_STATUS_STATUS_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.system_status_msg.data.raw, tx_frame->can_dlc);
        break;
    }
    case TracerMotorDriverStatusMsg:
    {
        if (msg->body.motor_driver_status_msg.motor_id == TRACER_MOTOR1_ID)
            tx_frame->can_id = CAN_MSG_MOTOR1_DRIVER_STATUS_ID;
        else if (msg->body.motor_driver_status_msg.motor_id == TRACER_MOTOR2_ID)
            tx_frame->can_id = CAN_MSG_MOTOR2_DRIVER_STATUS_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.motor_driver_status_msg.data.raw, tx_frame->can_dlc);
        break;
    }
    case TracerMotionCmdMsg:
    {
        EncodeTracerMotionControlMsgToCAN(&(msg->body.motion_cmd_msg), tx_frame);
        break;
    }
    default:
        break;
    }
    tx_frame->data[7] = CalcTracerCANChecksum(tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
}

void EncodeTracerMotionControlMsgToCAN(const MotionCmdMessage *msg, struct can_frame *tx_frame)
{
    tx_frame->can_id = CAN_MSG_MOTION_CMD_ID;
    tx_frame->can_dlc = 8;
    memcpy(tx_frame->data, msg->data.raw, tx_frame->can_dlc);
    tx_frame->data[7] = CalcTracerCANChecksum(tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
}

uint8_t CalcTracerCANChecksum(uint16_t id, uint8_t *data, uint8_t dlc)
{
    uint8_t checksum = 0x00;
    checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
    for (int i = 0; i < (dlc - 1); ++i)
        checksum += data[i];
    return checksum;
}