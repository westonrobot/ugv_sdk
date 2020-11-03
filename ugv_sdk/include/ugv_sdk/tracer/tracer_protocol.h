/* 
 * tracer_protocol.h
 * 
 * Created on: Apr 14, 2020 10:34
 * Description: 
 * 
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */ 

#ifndef TRACER_PROTOCOL_H
#define TRACER_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define TRACER_CMD_BUF_LEN               32
#define TRACER_STATUS_BUF_LEN            32
#define TRACER_FRAME_SIZE                13

#define TRACER_MOTOR1_ID                 ((uint8_t)0x00)
#define TRACER_MOTOR2_ID                 ((uint8_t)0x01)

// UART Definitions
#define UART_FRAME_SYSTEM_STATUS_ID         ((uint8_t)0x01)
#define UART_FRAME_MOTION_STATUS_ID         ((uint8_t)0x02)
#define UART_FRAME_MOTOR1_DRIVER_STATUS_ID  ((uint8_t)0x03)
#define UART_FRAME_MOTOR2_DRIVER_STATUS_ID  ((uint8_t)0x04)
#define UART_FRAME_LIGHT_STATUS_ID          ((uint8_t)0x07)

#define UART_FRAME_MOTION_CONTROL_ID        ((uint8_t)0x01)
#define UART_FRAME_LIGHT_CONTROL_ID         ((uint8_t)0x02)

// CAN Definitions
#define CAN_MSG_MOTION_CMD_ID            ((uint32_t)0x111)
#define CAN_MSG_MOTION_STATUS_ID         ((uint32_t)0x221)
#define CAN_MSG_LIGHT_CONTROL_CMD_ID     ((uint32_t)0x121)
#define CAN_MSG_LIGHT_CONTROL_STATUS_ID  ((uint32_t)0x231)
#define CAN_MSG_SYSTEM_STATUS_STATUS_ID  ((uint32_t)0x211)
#define CAN_MSG_MOTOR1_DRIVER_STATUS_ID  ((uint32_t)0x251)
#define CAN_MSG_MOTOR2_DRIVER_STATUS_ID  ((uint32_t)0x252)
#define CAN_MSG_COMTROL_MODE_ID          ((uint32_t)0x421)
#define CAN_MSG_ODOMETER_ID             ((uint32_t)0x311)

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
#define FAULT_RESERVED1                 ((uint16_t)0x0010)
#define FAULT_RESERVED2                 ((uint16_t)0x0020)
#define FAULT_MOTOR_DRV_OVERHEAT_F      ((uint16_t)0x0040)
#define FAULT_MOTOR_OVERCURRENT_F       ((uint16_t)0x0080)

/*-------------------- Control/Feedback Messages -----------------------*/

/* No padding in the struct */
// reference: https://stackoverflow.com/questions/3318410/pragma-pack-effect
#pragma pack(push, 1)

// Note: id could be different for UART and CAN protocol

// Motion Control(can)
typedef struct {
    union
    {
        struct
        {
          struct
          {
            int8_t H_byte;
            int8_t L_byte;
          }linear_velocity;
          struct
          {
            int8_t H_byte;
            int8_t L_byte;
          }angular_velocity;
          uint8_t reserved0;
          uint8_t reserved1;
          uint8_t reserved2;
          uint8_t reserved3;
        } cmd;
        uint8_t raw[8];
    } data;
} MotionCmdMessage;

// Motion Control(uart)
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
} UartMotionControlMessage;

typedef struct {
    union
    {
      struct
      {
        uint8_t control_mode;
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
            uint8_t reserved2;
            uint8_t reserved3;
        } status;
        uint8_t raw[8];
    } data;
} MotionStatusMessage;

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
} UartMotionStatusMessage;

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
            uint8_t fault_code;
            uint8_t reserved0;
            uint8_t reserved1;
            uint8_t count;
            //uint8_t checksum;
        } status;
        uint8_t raw[8];
    } data;
} SystemStatusMessage;

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
} UartSystemStatusMessage;

// Light Control
typedef struct {
    union
    {
        struct
        {
            uint8_t light_ctrl_enable;
            uint8_t front_light_mode;
            uint8_t front_light_custom;
            uint8_t reserved0;
            uint8_t reserved1;
            uint8_t reserved2;
            uint8_t reserved3;
            uint8_t count;

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
        } cmd;
        uint8_t raw[8];
    } data;
} UartLightControlMessage;

typedef struct {
    union
    {
        struct
        {
            uint8_t light_ctrl_enable;
            uint8_t front_light_mode;
            uint8_t front_light_custom;
            uint8_t reserved0;
            uint8_t reserved1;
            uint8_t reserved2;
            uint8_t reserved3;
            uint8_t count;          
        } status;
        uint8_t raw[8];
    } data;
} LightStatusMessage;

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
} UartLightStatusMessage;

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
} UartMotorDriverStatusMessage;

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
            } rpm;
            int8_t reserved0;
            int8_t reserved1;
            int8_t reserved2;
            int8_t reserved3;
            int8_t reserved4;
            int8_t reserved5;
        } status;
        uint8_t raw[8];
    } data;
} MotorHeightSpeedStatusMessage;

typedef struct
{
    uint8_t motor_id;
    union {
        struct
        {           
           int8_t reserved0;
           int8_t reserved1;
           int8_t reserved2;
           int8_t reserved3;
           int8_t reserved4;
           int8_t driver_state;
           int8_t reserved5;
           int8_t reserved6;
        } status;
        uint8_t raw[8];
    } data;
} MotorLowSpeedStatusMessage;

typedef struct
{
    uint8_t motor_id;
    union {
        struct
        {
          struct
          {
            uint8_t highest;
            uint8_t sec_highest;
            uint8_t sec_lowest;
            uint8_t lowest;
          }leftodometer;
          struct
          {
            uint8_t highest;
            uint8_t sec_highest;
            uint8_t sec_lowest;
            uint8_t lowest;
          }rightodometer;
        } status;
        uint8_t raw[8];
    } data;
} OdometerMessage;

// For convenience to access status/control message
typedef enum
{
    TracerMsgNone = 0x00,
    // status messages
    TracerMotionStatusMsg = 0x01,
    TracerLightStatusMsg = 0x02,
    TracerSystemStatusMsg = 0x03,
    TracerMotorDriverStatusMsg = 0x04,
    TracerOdometerMsg = 0x05,
    TracerHeighSpeedMsg = 0x06,
    TracerLowSpeedMsg = 0x07,

    // control messages
    TracerMotionCmdMsg = 0x21,
    TracerLightControlMsg = 0x22,
    TracerModeControlMsg = 0x23
} TracerMsgType;

typedef enum
{
    UartTracerMsgNone = 0x00,
    // status messages
    UartTracerMotionStatusMsg = 0x01,
    UartTracerLightStatusMsg = 0x02,
    UartTracerSystemStatusMsg = 0x03,
    UartTracerMotorDriverStatusMsg = 0x04,
    // control messages
    UartTracerMotionControlMsg = 0x21,
    UartTracerLightControlMsg = 0x22
} UartTracerMsgType;

typedef struct
{
    TracerMsgType type;
    union {
        // status messages
        MotionStatusMessage motion_status_msg;
        LightStatusMessage light_status_msg;
        SystemStatusMessage system_status_msg;
        MotorDriverStatusMessage motor_driver_status_msg;
        OdometerMessage odom_msg;
        MotorHeightSpeedStatusMessage motor_heigh_speed_msg;
        MotorLowSpeedStatusMessage motor_low_speed_msg;

        // control messages
        MotionCmdMessage motion_cmd_msg;
        LightControlMessage light_control_msg;
        ModSelectMessage mode_cmd_msg;

    } body;
} TracerMessage;

typedef struct
{
    UartTracerMsgType type;
    union {
        // status messages
        UartMotionStatusMessage motion_status_msg;
        UartLightStatusMessage light_status_msg;
        UartSystemStatusMessage system_status_msg;
        UartMotorDriverStatusMessage motor_driver_status_msg;
        // control messages
        UartMotionControlMessage motion_control_msg;
        UartLightControlMessage light_control_msg;
    } body;
} UartTracerMessage;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* TRACER_PROTOCOL_H */
