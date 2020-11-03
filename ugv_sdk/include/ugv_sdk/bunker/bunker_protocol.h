/* 
 * bunker_protocol.h
 * 
 * Created on: Aug 07, 2019 21:49
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 
#ifndef BUNKER_PROTOCOL_H
#define BUNKER_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define BUNKER_CMD_BUF_LEN               32
#define BUNKER_STATUS_BUF_LEN            32
#define BUNKER_FRAME_SIZE                13

#define BUNKER_MOTOR1_ID                 ((uint8_t)0x00)
#define BUNKER_MOTOR2_ID                 ((uint8_t)0x01)
//#define BUNKER_MOTOR3_ID                 ((uint8_t)0x02)
//#define BUNKER_MOTOR4_ID                 ((uint8_t)0x03)

// UART Definitions
#define UART_FRAME_SYSTEM_STATUS_ID         ((uint8_t)0x01)
#define UART_FRAME_MOTION_STATUS_ID         ((uint8_t)0x02)
#define UART_FRAME_MOTOR1_DRIVER_STATUS_ID  ((uint8_t)0x03)
#define UART_FRAME_MOTOR2_DRIVER_STATUS_ID  ((uint8_t)0x04)
#define UART_FRAME_MOTOR3_DRIVER_STATUS_ID  ((uint8_t)0x05)
#define UART_FRAME_MOTOR4_DRIVER_STATUS_ID  ((uint8_t)0x06)
#define UART_FRAME_LIGHT_STATUS_ID          ((uint8_t)0x07)

#define UART_FRAME_MOTION_CONTROL_ID        ((uint8_t)0x01)
#define UART_FRAME_LIGHT_CONTROL_ID         ((uint8_t)0x02)

// CAN Definitions
#define CAN_MSG_MOTION_CONTROL_CMD_ID       ((uint32_t)0x130)
#define CAN_MSG_MOTION_CONTROL_STATUS_ID    ((uint32_t)0x131)
#define CAN_MSG_LIGHT_CONTROL_CMD_ID        ((uint32_t)0x140)
#define CAN_MSG_LIGHT_CONTROL_STATUS_ID     ((uint32_t)0x141)
#define CAN_MSG_SYSTEM_STATUS_STATUS_ID     ((uint32_t)0x151)
#define CAN_MSG_MOTOR1_DRIVER_STATUS_ID     ((uint32_t)0x200)
#define CAN_MSG_MOTOR2_DRIVER_STATUS_ID     ((uint32_t)0x201)
//#define CAN_MSG_MOTOR3_DRIVER_STATUS_ID     ((uint32_t)0x202)
//#define CAN_MSG_MOTOR4_DRIVER_STATUS_ID     ((uint32_t)0x203)

/*--------------------- Control/State Constants ------------------------*/

// Motion Control
#define CTRL_MODE_REMOTE                ((uint8_t)0x00)
#define CTRL_MODE_CMD_CAN               ((uint8_t)0x01)
#define CTRL_MODE_CMD_UART              ((uint8_t)0x02)
#define CTRL_MODE_COMMANDED             ((uint8_t)0x03)

#define FAULT_CLR_NONE                  ((uint8_t)0x00)
#define FAULT_CLR_BAT_UNDER_VOL         ((uint8_t)0x01)
#define FAULT_CLR_BAT_OVER_VOL          ((uint8_t)0x02)
#define FAULT_CLR_MOTOR1_COMM           ((uint8_t)0x03)
#define FAULT_CLR_MOTOR2_COMM           ((uint8_t)0x04)
#define FAULT_CLR_MOTOR3_COMM           ((uint8_t)0x05)
#define FAULT_CLR_MOTOR4_COMM           ((uint8_t)0x06)
#define FAULT_CLR_MOTOR_DRV_OVERHEAT    ((uint8_t)0x07)
#define FAULT_CLR_MOTOR_OVERCURRENT     ((uint8_t)0x08)

// Light Control
#define LIGHT_DISABLE_CTRL              ((uint8_t)0x00)
#define LIGHT_ENABLE_CTRL               ((uint8_t)0x01)

#define LIGHT_MODE_CONST_OFF            ((uint8_t)0x00)
#define LIGHT_MODE_CONST_ON             ((uint8_t)0x01)
#define LIGHT_MODE_BREATH               ((uint8_t)0x02)
#define LIGHT_MODE_CUSTOM               ((uint8_t)0x03)

// System Status Feedback
#define BASE_STATE_NORMAL               ((uint8_t)0x00)
#define BASE_STATE_ESTOP                ((uint8_t)0x01)
#define BASE_STATE_EXCEPTION            ((uint8_t)0x02)

#define FAULT_CAN_CHECKSUM_ERROR        ((uint16_t)0x0100)
#define FAULT_MOTOR_DRV_OVERHEAT_W      ((uint16_t)0x0200)
#define FAULT_MOTOR_OVERCURRENT_W       ((uint16_t)0x0400)
#define FAULT_BAT_UNDER_VOL_W           ((uint16_t)0x0800)
#define FAULT_RC_SIGNAL_LOSS            ((uint16_t)0x1000)
#define FAULT_HIGH_BYTE_RESERVED2       ((uint16_t)0x2000)
#define FAULT_HIGH_BYTE_RESERVED3       ((uint16_t)0x4000)
#define FAULT_HIGH_BYTE_RESERVED4       ((uint16_t)0x8000)

#define FAULT_BAT_UNDER_VOL_F           ((uint16_t)0x0001)
#define FAULT_BAT_OVER_VOL_F            ((uint16_t)0x0002)
#define FAULT_MOTOR1_COMM_F             ((uint16_t)0x0004)
#define FAULT_MOTOR2_COMM_F             ((uint16_t)0x0008)
#define FAULT_MOTOR3_COMM_F             ((uint16_t)0x0010)
#define FAULT_MOTOR4_COMM_F             ((uint16_t)0x0020)
#define FAULT_MOTOR_DRV_OVERHEAT_F      ((uint16_t)0x0040)
#define FAULT_MOTOR_OVERCURRENT_F       ((uint16_t)0x0080)

/*-------------------- Control/Feedback Messages -----------------------*/

/* No padding in the struct */
// reference: https://stackoverflow.com/questions/3318410/pragma-pack-effect
#pragma pack(push, 1)

// Note: id could be different for UART and CAN protocol

// Motion Control
typedef struct {
    union
    {
        struct
        {
            uint8_t control_mode;
            uint8_t fault_clear_flag;
            int8_t linear_velocity_cmd;
            int8_t angular_velocity_cmd;
            uint8_t reserved0;
            uint8_t reserved1;
            uint8_t count;
            uint8_t checksum;
        } cmd;
        uint8_t raw[8];
    } data;
} MotionControlMessage;

typedef struct {
    union
    {
        struct
        {
            struct
            {
                uint8_t high_byte;
                uint8_t low_byte;
            } linear_velocity;
            struct
            {
                uint8_t high_byte;
                uint8_t low_byte;
            } angular_velocity;
            uint8_t reserved0;
            uint8_t reserved1;
            uint8_t count;
            uint8_t checksum;
        } status;
        uint8_t raw[8];
    } data;
} MotionStatusMessage;

// System Status Feedback
typedef struct {
    union
    {
        struct
        {
            uint8_t base_state;
            uint8_t control_mode;
            struct
            {
                uint8_t high_byte;
                uint8_t low_byte;
            } battery_voltage;
            struct
            {
                uint8_t high_byte;
                uint8_t low_byte;
            } fault_code;
            uint8_t count;
            uint8_t checksum;
        } status;
        uint8_t raw[8];
    } data;
} SystemStatusMessage;

// Light Control
typedef struct {
    union
    {
        struct
        {
            uint8_t light_ctrl_enable;
            uint8_t front_light_mode;
            uint8_t front_light_custom;
            uint8_t rear_light_mode;
            uint8_t rear_light_custom;
            uint8_t reserved0;
            uint8_t count;
            uint8_t checksum;
        } cmd;
        uint8_t raw[8];
    } data;
} LightControlMessage;

typedef struct {
    union
    {
        struct
        {
            uint8_t light_ctrl_enable;
            uint8_t front_light_mode;
            uint8_t front_light_custom;
            uint8_t rear_light_mode;
            uint8_t rear_light_custom;
            uint8_t reserved0;
            uint8_t count;
            uint8_t checksum;
        } status;
        uint8_t raw[8];
    } data;
} LightStatusMessage;

// Motor Driver Feedback
typedef struct
{
    uint8_t motor_id;
    union {
        struct
        {
            struct
            {
                uint8_t high_byte;
                uint8_t low_byte;
            } current;
            struct
            {
                uint8_t high_byte;
                uint8_t low_byte;
            } rpm;
            int8_t temperature;
            uint8_t reserved0;
            uint8_t count;
            uint8_t checksum;
        } status;
        uint8_t raw[8];
    } data;
} MotorDriverStatusMessage;

// For convenience to access status/control message
typedef enum
{
    BunkerMsgNone = 0x00,
    // status messages
    BunkerMotionStatusMsg = 0x01,
    BunkerLightStatusMsg = 0x02,
    BunkerSystemStatusMsg = 0x03,
    BunkerMotorDriverStatusMsg = 0x04,
    // control messages
    BunkerMotionControlMsg = 0x21,
    BunkerLightControlMsg = 0x22
} BunkerMsgType;

typedef struct 
{
    BunkerMsgType type;
    union {
        // status messages
        MotionStatusMessage motion_status_msg;
        LightStatusMessage light_status_msg;
        SystemStatusMessage system_status_msg;
        MotorDriverStatusMessage motor_driver_status_msg;
        // control messages
        MotionControlMessage motion_control_msg;
        LightControlMessage light_control_msg;
    } body;
} BunkerMessage;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* BUNKER_PROTOCOL_H */
