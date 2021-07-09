/*
 * agilex_message_v1.h
 *
 * Created on: Jul 09, 2021 20:55
 * Description:
 *  all values are using SI units (e.g. meter/second/radian)
 *
 * Re-implemented as a subset of AgxMessage defined in protocol v2
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef AGILEX_MESSAGE_V1_H
#define AGILEX_MESSAGE_V1_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/***************** Control messages *****************/

typedef enum {
  //   CONTROL_MODE_STANDBY = 0x00,
  CONTROL_MODE_RC = 0x00,
  CONTROL_MODE_CAN = 0x01,
  CONTROL_MODE_UART = 0x02
} ControlMode;

// 0x130
typedef struct {
  ControlMode control_mode;
  uint8_t fault_clear;

  float linear;
  float angular;
} MotionCommandMessage;

// 0x140
typedef enum {
  CONST_OFF = 0x00,
  CONST_ON = 0x01,
  BREATH = 0x02,
  CUSTOM = 0x03
} LightMode;

typedef struct {
  LightMode mode;
  uint8_t custom_value;
} LightOperation;

typedef struct {
  bool enable_cmd_ctrl;
  LightOperation front_light;
  LightOperation rear_light;
} LightCommandMessage;

typedef struct {
  bool set_neutral;
} ValueSetCommandMessage;

/**************** Feedback messages *****************/

typedef enum {
  VehicleStateNormal = 0x00,
  VehicleStateEStop = 0x01,
  VehicleStateException = 0x02
} VehicleState;

#define SYSTEM_ERROR_MOTOR_DRIVER_MASK ((uint16_t)0x0100)
#define SYSTEM_ERROR_HL_COMM_MASK ((uint16_t)0x0200)
#define SYSTEM_ERROR_BATTERY_FAULT_MASK ((uint16_t)0x0001)
#define SYSTEM_ERROR_BATTERY_WARNING_MASK ((uint16_t)0x0002)
#define SYSTEM_ERROR_RC_SIGNAL_LOSS_MASK ((uint16_t)0x0004)
#define SYSTEM_ERROR_MOTOR1_COMM_MASK ((uint16_t)0x0008)
#define SYSTEM_ERROR_MOTOR2_COMM_MASK ((uint16_t)0x0010)
#define SYSTEM_ERROR_MOTOR3_COMM_MASK ((uint16_t)0x0020)
#define SYSTEM_ERROR_MOTOR4_COMM_MASK ((uint16_t)0x0040)
#define SYSTEM_ERROR_STEER_ENCODER_MASK ((uint16_t)0x0080)

typedef struct {
  VehicleState vehicle_state;
  ControlMode control_mode;
  float battery_voltage;
  uint16_t error_code;
} SystemStateMessage;

// 0x221
typedef struct {
  float linear_velocity;
  float angular_velocity;  // only valid for differential drivering
  float lateral_velocity;
  float steering_angle;  // only valid for ackermann steering
} MotionStateMessage;

// 0x231
typedef LightCommandMessage LightStateMessage;

// 0x251 - 0x258
typedef struct {
  uint8_t motor_id;
  int16_t rpm;
  float current;
  int32_t pulse_count;
} ActuatorStateMessage;

// 0x261 - 0x264
#define DRIVER_STATE_INPUT_VOLTAGE_LOW_MASK ((uint8_t)0x01)
#define DRIVER_STATE_MOTOR_OVERHEAT_MASK ((uint8_t)0x02)
#define DRIVER_STATE_DRIVER_OVERLOAD_MASK ((uint8_t)0x04)
#define DRIVER_STATE_DRIVER_OVERHEAT_MASK ((uint8_t)0x08)
#define DRIVER_STATE_SENSOR_FAULT_MASK ((uint8_t)0x10)
#define DRIVER_STATE_DRIVER_FAULT_MASK ((uint8_t)0x20)
#define DRIVER_STATE_DRIVER_ENABLED_MASK ((uint8_t)0x40)
#define DRIVER_STATE_DRIVER_RESET_MASK ((uint8_t)0x80)

// 0x291
typedef struct {
  uint8_t motion_mode;
  uint8_t mode_changing;
} MotionModeFeedbackMessage;

typedef struct {
  uint8_t motor_id;
  float driver_voltage;
  float driver_temperature;
  int8_t motor_temperature;
  uint8_t driver_state;
} ActuatorLSStateMessage;

//////////////////////////////////////////////////////

typedef enum {
  AgxMsgUnkonwn = 0x00,
  // command
  AgxMsgMotionCommand,
  AgxMsgLightCommand,
  AgxMsgValueSetCommand,
  // state feedback
  AgxMsgSystemState,
  AgxMsgMotionState,
  AgxMsgLightState,
  AgxMsgActuatorState,
} MsgType;

typedef struct {
  MsgType type;
  union {
    // command
    MotionCommandMessage motion_command_msg;
    LightCommandMessage light_command_msg;
    // state feedback
    SystemStateMessage system_state_msg;
    MotionStateMessage motion_state_msg;
    LightStateMessage light_state_msg;
    ActuatorStateMessage actuator_state_msg;
  } body;
} AgxMessageV1;

#ifdef __cplusplus
}
#endif

#endif /* AGILEX_MESSAGE_V1_H */
