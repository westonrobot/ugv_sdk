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

#include "ugv_sdk/details/interface/agilex_types.h"

/***************** Control messages *****************/

typedef struct {
  ControlMode control_mode;
  bool clear_all_error;
  float linear;
  float angular;
} MotionCommandMessage;

typedef struct {
  bool enable_cmd_ctrl;
  LightOperation front_light;
  LightOperation rear_light;
} LightCommandMessage;

typedef struct {
  bool set_neutral;
} ValueSetCommandMessage;

/**************** Feedback messages *****************/

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

typedef struct {
  float linear_velocity;
  float angular_velocity;
} MotionStateMessage;

typedef LightCommandMessage LightStateMessage;

typedef struct {
  float current;
  int16_t rpm;
  float temperature;
} ActuatorStateMessage;

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
