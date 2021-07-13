/*
 * agilex_protocol_v1.h
 *
 * Created on: Jul 09, 2021 20:34
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef AGILEX_PROTOCOL_V1_H
#define AGILEX_PROTOCOL_V1_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// define endianess of the platform
#if (!defined(USE_LITTLE_ENDIAN) && !defined(USE_BIG_ENDIAN))
#define USE_LITTLE_ENDIAN
#endif

#ifdef USE_BIG_ENDIAN
#error "BIG ENDIAN IS CURRENTLY NOT SUPPORTED"
#endif

/*---------------------------- Motor IDs -------------------------------*/

#define ACTUATOR1_ID ((uint8_t)0x00)
#define ACTUATOR2_ID ((uint8_t)0x01)
#define ACTUATOR3_ID ((uint8_t)0x02)
#define ACTUATOR4_ID ((uint8_t)0x03)

/*--------------------------- Message IDs ------------------------------*/

// CAN: control group
#define CAN_MSG_MOTION_COMMAND_ID ((uint32_t)0x130)
#define CAN_MSG_LIGHT_COMMAND_ID ((uint32_t)0x140)
#define CAN_MSG_VALUE_SET_COMMAND_ID ((uint32_t)0x210)

// CAN: state feedback group
#define CAN_MSG_MOTION_STATE_ID ((uint32_t)0x131)
#define CAN_MSG_LIGHT_STATE_ID ((uint32_t)0x141)
#define CAN_MSG_SYSTEM_STATE_ID ((uint32_t)0x151)

#define CAN_MSG_VALUE_SET_STATE_ID ((uint32_t)0x211)

#define CAN_MSG_ACTUATOR1_STATE_ID ((uint32_t)0x200)
#define CAN_MSG_ACTUATOR2_STATE_ID ((uint32_t)0x201)
#define CAN_MSG_ACTUATOR3_STATE_ID ((uint32_t)0x202)
#define CAN_MSG_ACTUATOR4_STATE_ID ((uint32_t)0x203)

/*------------------------ Frame Memory Layout -------------------------*/

/* No padding in the struct */
// reference: https://stackoverflow.com/questions/3318410/pragma-pack-effect
#pragma pack(push, 1)

#ifdef USE_LITTLE_ENDIAN
typedef struct {
  uint8_t high_byte;
  uint8_t low_byte;
} struct16_t;
typedef struct {
  uint8_t msb;
  uint8_t high_byte;
  uint8_t low_byte;
  uint8_t lsb;
} struct32_t;
#elif defined(USE_BIG_ENDIAN)
typedef struct {
  uint8_t low_byte;
  uint8_t high_byte;
} struct16_t;
typedef struct {
  uint8_t lsb;
  uint8_t low_byte;
  uint8_t high_byte;
  uint8_t msb;
} struct32_t;
#endif

// Control messages
#define CTRL_MODE_REMOTE ((uint8_t)0x00)
#define CTRL_MODE_CMD_CAN ((uint8_t)0x01)
#define CTRL_MODE_CMD_UART ((uint8_t)0x02)
#define CTRL_MODE_COMMANDED ((uint8_t)0x03)

#define ERROR_CLR_NONE ((uint8_t)0x00)
#define ERROR_CLR_BAT_UNDER_VOL ((uint8_t)0x01)
#define ERROR_CLR_BAT_OVER_VOL ((uint8_t)0x02)
#define ERROR_CLR_MOTOR1_COMM ((uint8_t)0x03)
#define ERROR_CLR_MOTOR2_COMM ((uint8_t)0x04)
#define ERROR_CLR_MOTOR3_COMM ((uint8_t)0x05)
#define ERROR_CLR_MOTOR4_COMM ((uint8_t)0x06)
#define ERROR_CLR_MOTOR_DRV_OVERHEAT ((uint8_t)0x07)
#define ERROR_CLR_MOTOR_OVERCURRENT ((uint8_t)0x08)

typedef struct {
  uint8_t control_mode;
  uint8_t error_clear_byte;
  int8_t linear_percentage;
  int8_t angular_percentage;
  int8_t lateral_percentage;
  uint8_t reserved0;
  uint8_t count;
  uint8_t checksum;
} MotionCommandFrame;

#define LIGHT_ENABLE_CMD_CTRL ((uint8_t)0x01)
#define LIGHT_DISABLE_CMD_CTRL ((uint8_t)0x00)

#define LIGHT_MODE_CONST_OFF ((uint8_t)0x00)
#define LIGHT_MODE_CONST_ON ((uint8_t)0x01)
#define LIGHT_MODE_BREATH ((uint8_t)0x02)
#define LIGHT_MODE_CUSTOM ((uint8_t)0x03)

typedef struct {
  uint8_t enable_cmd_ctrl;
  uint8_t front_mode;
  uint8_t front_custom;
  uint8_t rear_mode;
  uint8_t rear_custom;
  uint8_t reserved0;
  uint8_t count;
  uint8_t checksum;
} LightCommandFrame;

typedef struct {
  uint8_t set_neutral;
  uint8_t reserved0;
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t reserved4;
  uint8_t count;
  uint8_t checksum;
} ValueSetCommandFrame;

// State feedback messages
#define VEHICLE_STATE_NORMAL ((uint8_t)0x00)
#define VEHICLE_STATE_ESTOP ((uint8_t)0x01)
#define VEHICLE_STATE_EXCEPTION ((uint8_t)0x02)

#define ERROR_CAN_CHECKSUM_ERROR ((uint16_t)0x0100)
#define ERROR_MOTOR_DRV_OVERHEAT_W ((uint16_t)0x0200)
#define ERROR_MOTOR_OVERCURRENT_W ((uint16_t)0x0400)
#define ERROR_BAT_UNDER_VOL_W ((uint16_t)0x0800)
#define ERROR_RC_SIGNAL_LOSS ((uint16_t)0x1000)
#define ERROR_HIGH_BYTE_RESERVED2 ((uint16_t)0x2000)
#define ERROR_HIGH_BYTE_RESERVED3 ((uint16_t)0x4000)
#define ERROR_HIGH_BYTE_RESERVED4 ((uint16_t)0x8000)

#define ERROR_BAT_UNDER_VOL_F ((uint16_t)0x0001)
#define ERROR_BAT_OVER_VOL_F ((uint16_t)0x0002)
#define ERROR_MOTOR1_COMM_F ((uint16_t)0x0004)
#define ERROR_MOTOR2_COMM_F ((uint16_t)0x0008)
#define ERROR_MOTOR3_COMM_F ((uint16_t)0x0010)
#define ERROR_MOTOR4_COMM_F ((uint16_t)0x0020)
#define ERROR_MOTOR_DRV_OVERHEAT_F ((uint16_t)0x0040)
#define ERROR_MOTOR_OVERCURRENT_F ((uint16_t)0x0080)

typedef struct {
  uint8_t vehicle_state;
  uint8_t control_mode;
  struct16_t battery_voltage;
  struct16_t error_code;
  uint8_t count;
  uint8_t checksum;
} SystemStateFrame;

typedef struct {
  struct16_t linear;
  struct16_t angular;
  uint8_t reserved0;
  uint8_t reserved1;
  uint8_t count;
  uint8_t checksum;
} MotionStateFrame;

typedef LightCommandFrame LightStateFrame;

typedef struct {
  struct16_t current;
  struct16_t rpm;
  int8_t temperature;
  uint8_t reserved0;
  uint8_t count;
  uint8_t checksum;
} ActuatorStateFrame;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* AGILEX_PROTOCOL_V1_H */
