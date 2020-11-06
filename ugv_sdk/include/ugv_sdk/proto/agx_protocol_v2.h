/*
 * agx_protocol_v2.h
 *
 * Created on: Nov 04, 2020 13:54
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef AGX_PROTOCOL_V2_H
#define AGX_PROTOCOL_V2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*---------------------------- Motor IDs -------------------------------*/

#define ACTUATOR1_ID ((uint8_t)0x00)
#define ACTUATOR2_ID ((uint8_t)0x01)
#define ACTUATOR3_ID ((uint8_t)0x02)
#define ACTUATOR4_ID ((uint8_t)0x03)

/*--------------------------- Message IDs ------------------------------*/

#define CAN_MSG_MOTION_COMMAND_ID ((uint32_t)0x111)
#define CAN_MSG_LIGHT_COMMAND_ID ((uint32_t)0x121)

#define CAN_MSG_SYSTEM_STATE_ID ((uint32_t)0x211)
#define CAN_MSG_MOTION_STATE_ID ((uint32_t)0x221)
#define CAN_MSG_LIGHT_STATE_ID ((uint32_t)0x231)
#define CAN_MSG_RC_STATE_ID ((uint32_t)0x241)

#define CAN_MSG_ACTUATOR1_HS_STATE_ID ((uint32_t)0x251)
#define CAN_MSG_ACTUATOR2_HS_STATE_ID ((uint32_t)0x252)
#define CAN_MSG_ACTUATOR3_HS_STATE_ID ((uint32_t)0x253)
#define CAN_MSG_ACTUATOR4_HS_STATE_ID ((uint32_t)0x254)

#define CAN_MSG_ACTUATOR1_LS_STATE_ID ((uint32_t)0x261)
#define CAN_MSG_ACTUATOR2_LS_STATE_ID ((uint32_t)0x262)
#define CAN_MSG_ACTUATOR3_LS_STATE_ID ((uint32_t)0x263)
#define CAN_MSG_ACTUATOR4_LS_STATE_ID ((uint32_t)0x264)

#define CAN_MSG_ODOMETRY_ID ((uint32_t)0x311)

#define CAN_MSG_VERSION_QUERY_ID ((uint32_t)0x411)
#define CAN_MSG_PLATFORM_VERSION_ID ((uint32_t)0x41a)

#define CAN_MSG_CTRL_MODE_SELECT_ID ((uint32_t)0x421)
#define CAN_MSG_STATE_RESET_ID ((uint32_t)0x441)

/*--------------------- Control/State Constants ------------------------*/

// System State
#define VEHICLE_STATE_NORMAL ((uint8_t)0x00)
#define VEHICLE_STATE_ESTOP ((uint8_t)0x01)
#define VEHICLE_STATE_EXCEPTION ((uint8_t)0x02)

#define FAULT_BATTERY_LOW_ERROR ((uint8_t)0x01)
#define FAULT_BATTERY_LOW_WARN ((uint8_t)0x02)
#define FAULT_RC_SIGNAL_LOSS ((uint8_t)0x04)

#define FAULT_CLR_ALL ((uint8_t)0x00)
#define FAULT_CLR_MOTOR1_COMM ((uint8_t)0x01)
#define FAULT_CLR_MOTOR2_COMM ((uint8_t)0x02)
#define FAULT_CLR_MOTOR3_COMM ((uint8_t)0x03)
#define FAULT_CLR_MOTOR4_COMM ((uint8_t)0x04)

#define QUERY_PLATFORM_VERSION_REQUEST ((uint8_t)0x01)

// Motion Control
#define CTRL_MODE_RC ((uint8_t)0x00)
#define CTRL_MODE_CMD_CAN ((uint8_t)0x01)
#define CTRL_MODE_CMD_UART ((uint8_t)0x02)

// Light Control
#define LIGHT_CTRL_DISABLE ((uint8_t)0x00)
#define LIGHT_CTRL_ENABLE ((uint8_t)0x01)

#define LIGHT_MODE_CONST_OFF ((uint8_t)0x00)
#define LIGHT_MODE_CONST_ON ((uint8_t)0x01)
#define LIGHT_MODE_BREATH ((uint8_t)0x02)
#define LIGHT_MODE_CUSTOM ((uint8_t)0x03)

// Actuator State
#define BATTERY_VOLTAGE_LOW ((uint8_t)0x01)
#define MOTOR_OVERHEAT ((uint8_t)0x02)
#define MOTOR_DRIVER_OVERLOAD ((uint8_t)0x04)
#define MOTOR_DRIVER_OVERHEAT ((uint8_t)0x08)
#define MOTOR_SENSOR_FAULT ((uint8_t)0x10)
#define MOTOR_DRIVER_FAULT ((uint8_t)0x20)
#define MOTOR_DRIVER_ENABLED ((uint8_t)0x40)
#define MOTOR_DRIVER_RESERVED0 ((uint8_t)0x80)

/*-------------------- Control/Feedback Messages -----------------------*/

/* No padding in the struct */
// reference: https://stackoverflow.com/questions/3318410/pragma-pack-effect
#pragma pack(push, 1)

// Control messages
typedef union {
  struct {
    struct {
      int8_t high_byte;
      int8_t low_byte;
    } linear_velocity;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } angular_velocity;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } lateral_velocity;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } steering_angle;
  } cmd;
  uint8_t raw[8];
} MotionCommandMessage;

typedef union {
  struct {
    uint8_t light_ctrl_enabled;
    uint8_t front_light_mode;
    uint8_t front_light_custom;
    uint8_t rear_light_mode;
    uint8_t rear_light_custom;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t count;

  } cmd;
  uint8_t raw[8];
} LightCommandMessage;

typedef union {
  struct {
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
} CtrlModeSelectMessage;

typedef union {
  struct {
    uint8_t fault_byte;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t reserved4;
    uint8_t reserved5;
    uint8_t reserved6;
  } cmd;
  uint8_t raw[8];
} StateResetMessage;

// State feedback messages
typedef union {
  struct {
    uint8_t vehicle_state;
    uint8_t control_mode;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } battery_voltage;
    uint8_t fault_code;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t count;
  } state;
  uint8_t raw[8];
} SystemStateMessage;

typedef union {
  struct {
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } linear_velocity;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } angular_velocity;  // only valid for differential drivering
    uint8_t reserved0;
    uint8_t reserved1;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } steering_angle;  // only valid for ackermann steering
  } state;
  uint8_t raw[8];
} MotionStateMessage;

typedef union {
  struct {
    uint8_t light_ctrl_enabled;
    uint8_t front_light_mode;
    uint8_t front_light_custom;
    uint8_t rear_light_mode;
    uint8_t rear_light_custom;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t count;
  } state;
  uint8_t raw[8];
} LightStateMessage;

typedef union {
  struct {
    uint8_t sws;
    uint8_t right_stick_left_right;
    uint8_t right_stick_up_down;
    uint8_t left_stick_left_right;
    uint8_t left_stick_up_down;
    uint8_t var_a;
    uint8_t reserved0;
    uint8_t count;
  } state;
  uint8_t raw[8];
} RcStateMessage;

typedef struct {
  uint8_t motor_id;
  union {
    struct {
      struct {
        uint8_t high_byte;
        uint8_t low_byte;
      } rpm;
      struct {
        uint8_t high_byte;
        uint8_t low_byte;
      } current;
      struct {
        int8_t msb;
        int8_t high_byte;
        int8_t low_byte;
        int8_t lsb;
      } pulse_count;
    } state;
    uint8_t raw[8];
  } data;
} ActuatorHSStateMessage;

typedef struct {
  uint8_t motor_id;
  union {
    struct {
      struct {
        uint8_t high_byte;
        uint8_t low_byte;
      } driver_voltage;
      struct {
        uint8_t high_byte;
        uint8_t low_byte;
      } driver_temperature;
      int8_t motor_temperature;
      uint8_t driver_state;
      uint8_t reserved0;
      uint8_t reserved1;
    } state;
    uint8_t raw[8];
  } data;
} ActuatorLSStateMessage;

typedef union {
  struct {
    struct {
      uint8_t msb;
      uint8_t high_byte;
      uint8_t low_byte;
      uint8_t lsb;
    } left_wheel;
    struct {
      uint8_t msb;
      uint8_t high_byte;
      uint8_t low_byte;
      uint8_t lsb;
    } right_wheel;
  } state;
  uint8_t raw[8];
} OdometryMessage;

typedef union {
  struct {
    uint8_t request;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t reserved4;
    uint8_t reserved5;
    uint8_t reserved6;
  } state;
  uint8_t raw[8];
} VersionQueryMessage;

typedef union {
  struct {
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } controller_hw_version;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } motor_driver_hw_version;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } controller_sw_version;
    struct {
      uint8_t high_byte;
      uint8_t low_byte;
    } motor_driver_sw_version;
  } state;
  uint8_t raw[8];
} PlatformVersionMessage;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* AGX_PROTOCOL_V2_H */
