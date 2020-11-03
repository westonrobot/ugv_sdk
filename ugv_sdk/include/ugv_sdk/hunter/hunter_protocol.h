/* 
 * hunter_protocol.h
 * 
 * Created on: Jan 02, 2020 12:06
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef HUNTER_PROTOCOL_H
#define HUNTER_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define HUNTER_CMD_BUF_LEN               32
#define HUNTER_STATUS_BUF_LEN            32
#define HUNTER_FRAME_SIZE                13

#define HUNTER_MOTOR1_ID                 ((uint8_t)0x00)
#define HUNTER_MOTOR2_ID                 ((uint8_t)0x01)
#define HUNTER_MOTOR3_ID                 ((uint8_t)0x02)

// CAN Definitions
#define CAN_MSG_MOTION_CONTROL_CMD_ID           ((uint32_t)0x111)
#define CAN_MSG_MOTION_CONTROL_STATUS_ID        ((uint32_t)0x221)
#define CAN_MSG_SELECT_CONTROL_MODE_ID          ((uint32_t)0x421)
#define CAN_MSG_SYSTEM_STATUS_STATUS_ID         ((uint32_t)0x211)
#define CAN_MSG_MOTOR1_HEIGHT_DRIVER_STATUS_ID  ((uint32_t)0x251)
#define CAN_MSG_MOTOR2_HEIGHT_DRIVER_STATUS_ID  ((uint32_t)0x252)
#define CAN_MSG_MOTOR3_HEIGHT_DRIVER_STATUS_ID  ((uint32_t)0x253)
#define CAN_MSG_MOTOR1_LOW_DRIVER_STATUS_ID     ((uint32_t)0x261)
#define CAN_MSG_MOTOR2_LOW_DRIVER_STATUS_ID     ((uint32_t)0x262)
#define CAN_MSG_MOTOR3_LOW_DRIVER_STATUS_ID     ((uint32_t)0x263)
#define CAN_MSG_PARK_CONTROL_ID                 ((uint32_t)0x131)

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

// System Configuration
#define STEERING_ZERO_CONFIG_FAIL       ((uint8_t)0x00)
#define STEERING_ZERO_CONFIG_SUCCESS    ((uint8_t)0xaa)

// System Status Feedback
#define BASE_STATE_NORMAL               ((uint8_t)0x00)
#define BASE_STATE_ESTOP                ((uint8_t)0x01)
#define BASE_STATE_EXCEPTION            ((uint8_t)0x02)

#define FAULT_CAN_CHECKSUM_ERROR        ((uint16_t)0x0100)
#define FAULT_FRONT_STEER_ENCODER_F     ((uint16_t)0x0200)
#define FAULT_RC_SIGNAL_LOSS            ((uint16_t)0x0400)
#define FAULT_HIGH_BYTE_RESERVED1       ((uint16_t)0x0800)
#define FAULT_HIGH_BYTE_RESERVED2       ((uint16_t)0x1000)
#define FAULT_HIGH_BYTE_RESERVED3       ((uint16_t)0x2000)
#define FAULT_HIGH_BYTE_RESERVED4       ((uint16_t)0x4000)
#define FAULT_HIGH_BYTE_RESERVED5       ((uint16_t)0x8000)

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

// Motion Status Feedback
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
            uint8_t reserved0;
            uint8_t reserved1;
            uint8_t reserved2;
            uint8_t reserved3;
            struct
            {
                uint8_t high_byte;
                uint8_t low_byte;
            } angular_velocity;
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
            uint8_t park_mode;
            uint8_t count;
        } status;
        uint8_t raw[8];
    } data;
} SystemStatusMessage;

// Motor Driver Height Speed Status Feedback
typedef struct {
    uint8_t motor_id;
    union
    {
        struct
        {
            struct
            {
                uint8_t high_byte;
                uint8_t low_byte;
            } rpm;
            struct
            {
                uint8_t high_byte;
                uint8_t low_byte;
            } current;
            struct
            {
                uint8_t heighest;
                uint8_t sec_heighest;
                uint8_t sec_lowest;
                uint8_t lowest;
            } moter_pose;
        } status;
        uint8_t raw[8];
    } data;
} MotorDriverHeightSpeedStatusMessage;

// Motor Driver Low Speed Status Feedback
typedef struct {
    uint8_t motor_id;
    union
    {
        struct
        {
            struct
            {
                uint8_t high_byte;
                uint8_t low_byte;
            } driver_voltage;
            struct
            {
                uint8_t high_byte;
                uint8_t low_byte;
            } driver_temperature;
            uint8_t motor_temperature;
            uint8_t driver_status;
            uint8_t reserved0;
            uint8_t reserved1;
        } status;
        uint8_t raw[8];
    } data;
} MotorDriverLowSpeedStatusMessage;

// Motion Control
typedef struct {
    union
    {
        struct
        {
            struct
            {
                uint8_t high_byte;
                uint8_t low_byte;
            } linear_velocity_cmd;
            uint8_t reserved0;
            uint8_t reserved1;
            uint8_t reserved2;
            uint8_t reserved3;
            struct
            {
                uint8_t high_byte;
                uint8_t low_byte;
            } angular_velocity_cmd;
        } cmd;
        uint8_t raw[8];
    } data;
} MotionControlMessage;

// Parking Mode Control
typedef struct {
    union
    {
        struct
        {
           uint8_t packing_mode;
           uint8_t reserved0;
           uint8_t reserved1;
           uint8_t reserved2;
           uint8_t reserved3;
           uint8_t reserved4;
           uint8_t reserved5;
           uint8_t reserved6;
        } cmd;
        uint8_t raw[8];

    } data;
} ParkControlMessage;

// Motion Mode Control
typedef struct {
    union
    {
        struct
        {
            uint8_t mode_control;
            uint8_t reserved0;
            uint8_t reserved1;
            uint8_t reserved2;
            uint8_t reserved3;
            uint8_t reserved4;
            uint8_t reserved5;
            uint8_t reserved6;
        } cmd;
        uint8_t raw[8];
    } data;
} ModSelectMessage;

// For convenience to access status/control message
typedef enum
{
    HunterMsgNone = 0x00,
    // status messages
    HunterMotionStatusMsg = 0x01,
    HunterSystemStatusMsg = 0x02,
    HunterMotorDriverHeightSpeedStatusMsg = 0x03,
    HunterMotorDriverLowSpeedStatusMsg = 0x04,
    // control messages
    HunterMotionControlMsg = 0x21,
    HunterParkControlMsg = 0x22,
    HunterControlModeMsg = 0x23
} HunterMsgType;

typedef struct 
{
    HunterMsgType type;
    union {
        // status messages
        MotionStatusMessage motion_status_msg;
        SystemStatusMessage system_status_msg;
        MotorDriverHeightSpeedStatusMessage motor_driver_height_speed_status_msg;
        MotorDriverLowSpeedStatusMessage motor_driver_low_speed_status_msg;
        // control messages
        MotionControlMessage motion_control_msg;
        ParkControlMessage park_control_msg;
        ModSelectMessage mode_cmd_msg;
    } body;
} HunterMessage;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* HUNTER_PROTOCOL_H */
