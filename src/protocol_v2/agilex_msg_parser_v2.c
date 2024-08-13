/*
 * agx_msg_parser.c
 *
 * Created on: Aug 31, 2019 04:25
 * Description:
 *
 * Copyright (c) 2019 Weston Robot Pte. Ltd.
 */

#include "agilex_protocol_v2.h"
#include "protocol_v2/agilex_msg_parser_v2.h"

#include "stdio.h"
#include "string.h"
#include "math.h"

bool DecodeCanFrameV2(const struct can_frame *rx_frame, AgxMessage *msg) {
  bool ret = true;
  msg->type = AgxMsgUnkonwn;

  switch (rx_frame->can_id) {
    /***************** command frame *****************/
    case CAN_MSG_MOTION_COMMAND_ID: {
      msg->type = AgxMsgMotionCommand;
      // parse frame buffer to message
      MotionCommandFrame *frame = (MotionCommandFrame *)(rx_frame->data);
      msg->body.motion_command_msg.linear_velocity =
          (int16_t)((uint16_t)(frame->linear_velocity.low_byte) |
                    (uint16_t)(frame->linear_velocity.high_byte) << 8) /
          1000.0;
      msg->body.motion_command_msg.angular_velocity =
          (int16_t)((uint16_t)(frame->angular_velocity.low_byte) |
                    (uint16_t)(frame->angular_velocity.high_byte) << 8) /
          1000.0;
      msg->body.motion_command_msg.lateral_velocity =
          (int16_t)((uint16_t)(frame->lateral_velocity.low_byte) |
                    (uint16_t)(frame->lateral_velocity.high_byte) << 8) /
          1000.0;
      msg->body.motion_command_msg.steering_angle =
          (int16_t)((uint16_t)(frame->steering_angle.low_byte) |
                    (uint16_t)(frame->steering_angle.high_byte) << 8) /
          1000.0;
      break;
    }
    case CAN_MSG_SET_MOTION_MODE_ID: {
      msg->type = AgxMsgSetMotionModeCommand;
      SetMotionModeFrame *frame = (SetMotionModeFrame *)(rx_frame->data);
      msg->body.motion_mode_msg.motion_mode = frame->motion_mode;
      break;
    }
    case CAN_MSG_LIGHT_COMMAND_ID: {
      msg->type = AgxMsgLightCommand;
      // parse frame buffer to message
      LightCommandFrame *frame = (LightCommandFrame *)(rx_frame->data);
      msg->body.light_command_msg.enable_cmd_ctrl =
          (frame->enable_cmd_ctrl != 0) ? true : false;
      msg->body.light_command_msg.front_light.mode = frame->front_mode;
      msg->body.light_command_msg.front_light.custom_value =
          frame->front_custom;
      msg->body.light_command_msg.rear_light.mode = frame->rear_mode;
      msg->body.light_command_msg.rear_light.custom_value = frame->rear_custom;
      break;
    }
    case CAN_MSG_BRAKING_COMMAND_ID: {
      msg->type = AgxMsgBrakingCommand;
      // TODO
      break;
    }
    /***************** feedback frame ****************/
    case CAN_MSG_SYSTEM_STATE_ID: {
      msg->type = AgxMsgSystemState;
      SystemStateFrame *frame = (SystemStateFrame *)(rx_frame->data);
      msg->body.system_state_msg.vehicle_state = frame->vehicle_state;
      msg->body.system_state_msg.control_mode = frame->control_mode;
      msg->body.system_state_msg.battery_voltage =
          (int16_t)((uint16_t)(frame->battery_voltage.low_byte) |
                    (uint16_t)(frame->battery_voltage.high_byte) << 8) *
          0.1;
      msg->body.system_state_msg.error_code =
          (uint16_t)(frame->error_code.low_byte) |
          (uint16_t)(frame->error_code.high_byte) << 8;
      break;
    }
    case CAN_MSG_MOTION_STATE_ID: {
      msg->type = AgxMsgMotionState;
      MotionStateFrame *frame = (MotionStateFrame *)(rx_frame->data);
      msg->body.motion_state_msg.linear_velocity =
          (int16_t)((uint16_t)(frame->linear_velocity.low_byte) |
                    (uint16_t)(frame->linear_velocity.high_byte) << 8) /
          1000.0;
      msg->body.motion_state_msg.angular_velocity =
          (int16_t)((uint16_t)(frame->angular_velocity.low_byte) |
                    (uint16_t)(frame->angular_velocity.high_byte) << 8) /
          1000.0;
      msg->body.motion_state_msg.lateral_velocity =
          (int16_t)((uint16_t)(frame->lateral_velocity.low_byte) |
                    (uint16_t)(frame->lateral_velocity.high_byte) << 8) /
          1000.0;
      msg->body.motion_state_msg.steering_angle =
          (int16_t)((uint16_t)(frame->steering_angle.low_byte) |
                    (uint16_t)(frame->steering_angle.high_byte) << 8) /
          1000.0;
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
    case CAN_MSG_RC_STATE_ID: {
      msg->type = AgxMsgRcState;
      RcStateFrame *frame = (RcStateFrame *)(rx_frame->data);
      // switch a
      if ((frame->sws & RC_SWA_MASK) == RC_SWA_UP_MASK)
        msg->body.rc_state_msg.swa = RC_SWITCH_UP;
      else if ((frame->sws & RC_SWA_MASK) == RC_SWA_DOWN_MASK)
        msg->body.rc_state_msg.swa = RC_SWITCH_DOWN;
      // switch b
      if ((frame->sws & RC_SWB_MASK) == RC_SWB_UP_MASK)
        msg->body.rc_state_msg.swb = RC_SWITCH_UP;
      else if ((frame->sws & RC_SWB_MASK) == RC_SWB_MIDDLE_MASK)
        msg->body.rc_state_msg.swb = RC_SWITCH_MIDDLE;
      else if ((frame->sws & RC_SWB_MASK) == RC_SWB_DOWN_MASK)
        msg->body.rc_state_msg.swb = RC_SWITCH_DOWN;
      // switch c
      if ((frame->sws & RC_SWC_MASK) == RC_SWC_UP_MASK)
        msg->body.rc_state_msg.swc = RC_SWITCH_UP;
      else if ((frame->sws & RC_SWC_MASK) == RC_SWC_MIDDLE_MASK)
        msg->body.rc_state_msg.swc = RC_SWITCH_MIDDLE;
      else if ((frame->sws & RC_SWC_MASK) == RC_SWC_DOWN_MASK)
        msg->body.rc_state_msg.swc = RC_SWITCH_DOWN;
      // switch d
      if ((frame->sws & RC_SWD_MASK) == RC_SWD_UP_MASK)
        msg->body.rc_state_msg.swd = RC_SWITCH_UP;
      else if ((frame->sws & RC_SWD_MASK) == RC_SWD_DOWN_MASK)
        msg->body.rc_state_msg.swd = RC_SWITCH_DOWN;
      msg->body.rc_state_msg.stick_right_v = frame->stick_right_v;
      msg->body.rc_state_msg.stick_right_h = frame->stick_right_h;
      msg->body.rc_state_msg.stick_left_v = frame->stick_left_v;
      msg->body.rc_state_msg.stick_left_h = frame->stick_left_h;
      msg->body.rc_state_msg.var_a = frame->var_a;
      break;
    }
    case CAN_MSG_ACTUATOR1_HS_STATE_ID:
    case CAN_MSG_ACTUATOR2_HS_STATE_ID:
    case CAN_MSG_ACTUATOR3_HS_STATE_ID:
    case CAN_MSG_ACTUATOR4_HS_STATE_ID:
    case CAN_MSG_ACTUATOR5_HS_STATE_ID:
    case CAN_MSG_ACTUATOR6_HS_STATE_ID:
    case CAN_MSG_ACTUATOR7_HS_STATE_ID:
    case CAN_MSG_ACTUATOR8_HS_STATE_ID: {
      msg->type = AgxMsgActuatorHSState;
      ActuatorHSStateFrame *frame = (ActuatorHSStateFrame *)(rx_frame->data);
      msg->body.actuator_hs_state_msg.motor_id =
          rx_frame->can_id - CAN_MSG_ACTUATOR1_HS_STATE_ID;
      msg->body.actuator_hs_state_msg.rpm =
          (int16_t)((uint16_t)(frame->rpm.low_byte) |
                    (uint16_t)(frame->rpm.high_byte) << 8);
      msg->body.actuator_hs_state_msg.current =
          (int16_t)((uint16_t)(frame->current.low_byte) |
                    (uint16_t)(frame->current.high_byte) << 8) *
          0.1;
      msg->body.actuator_hs_state_msg.pulse_count =
          (int32_t)((uint32_t)(frame->pulse_count.lsb) |
                    (uint32_t)(frame->pulse_count.low_byte) << 8 |
                    (uint32_t)(frame->pulse_count.high_byte) << 16 |
                    (uint32_t)(frame->pulse_count.msb) << 24);
      break;
    }
    case CAN_MSG_ACTUATOR1_LS_STATE_ID:
    case CAN_MSG_ACTUATOR2_LS_STATE_ID:
    case CAN_MSG_ACTUATOR3_LS_STATE_ID:
    case CAN_MSG_ACTUATOR4_LS_STATE_ID:
    case CAN_MSG_ACTUATOR5_LS_STATE_ID:
    case CAN_MSG_ACTUATOR6_LS_STATE_ID:
    case CAN_MSG_ACTUATOR7_LS_STATE_ID:
    case CAN_MSG_ACTUATOR8_LS_STATE_ID: {
      msg->type = AgxMsgActuatorLSState;
      ActuatorLSStateFrame *frame = (ActuatorLSStateFrame *)(rx_frame->data);
      msg->body.actuator_hs_state_msg.motor_id =
          rx_frame->can_id - CAN_MSG_ACTUATOR1_LS_STATE_ID;
      msg->body.actuator_ls_state_msg.driver_voltage =
          ((uint16_t)(frame->driver_voltage.low_byte) |
           (uint16_t)(frame->driver_voltage.high_byte) << 8) *
          0.1;
      msg->body.actuator_ls_state_msg.driver_temp =
          (int16_t)((uint16_t)(frame->driver_temp.low_byte) |
                    (uint16_t)(frame->driver_temp.high_byte) << 8);
      msg->body.actuator_ls_state_msg.motor_temp = frame->motor_temp;
      msg->body.actuator_ls_state_msg.driver_state = frame->driver_state;
      break;
    }
    case CAN_MSG_CURRENT_CTRL_MODE: {
      msg->type = AgxMsgMotionModeState;
      MotionModeStateFrame *frame = (MotionModeStateFrame *)(rx_frame->data);

      msg->body.motion_mode_state_msg.motion_mode = frame->motion_mode;
      msg->body.motion_mode_state_msg.mode_changing = frame->mode_changing;
      break;
    }
    /****************** sensor frame *****************/
    case CAN_MSG_ODOMETRY_ID: {
      msg->type = AgxMsgOdometry;
      OdometryFrame *frame = (OdometryFrame *)(rx_frame->data);
      msg->body.odometry_msg.left_wheel =
          (int32_t)((uint32_t)(frame->left_wheel.lsb) |
                    (uint32_t)(frame->left_wheel.low_byte) << 8 |
                    (uint32_t)(frame->left_wheel.high_byte) << 16 |
                    (uint32_t)(frame->left_wheel.msb) << 24);
      msg->body.odometry_msg.right_wheel =
          (int32_t)((uint32_t)(frame->right_wheel.lsb) |
                    (uint32_t)(frame->right_wheel.low_byte) << 8 |
                    (uint32_t)(frame->right_wheel.high_byte) << 16 |
                    (uint32_t)(frame->right_wheel.msb) << 24);
      break;
    }
    case CAN_MSG_IMU_ACCEL_ID: {
      msg->type = AgxMsgImuAccel;
      // TODO
      break;
    }
    case CAN_MSG_IMU_GYRO_ID: {
      msg->type = AgxMsgImuGyro;
      // TODO
      break;
    }
    case CAN_MSG_IMU_EULER_ID: {
      msg->type = AgxMsgImuEuler;
      // TODO
      break;
    }
    case CAN_MSG_SAFETY_BUMPER_ID: {
      msg->type = AgxMsgSafetyBumper;
      // TODO
      break;
    }
    case CAN_MSG_ULTRASONIC_1_ID:
    case CAN_MSG_ULTRASONIC_2_ID:
    case CAN_MSG_ULTRASONIC_3_ID:
    case CAN_MSG_ULTRASONIC_4_ID:
    case CAN_MSG_ULTRASONIC_5_ID:
    case CAN_MSG_ULTRASONIC_6_ID:
    case CAN_MSG_ULTRASONIC_7_ID:
    case CAN_MSG_ULTRASONIC_8_ID: {
      msg->type = AgxMsgUltrasonic;
      // TODO
      break;
    }
    case CAN_MSG_UWB_1_ID:
    case CAN_MSG_UWB_2_ID:
    case CAN_MSG_UWB_3_ID:
    case CAN_MSG_UWB_4_ID: {
      msg->type = AgxMsgUwb;
      // TODO
      break;
    }
    case CAN_MSG_BMS_BASIC_ID: {
      msg->type = AgxMsgBmsBasic;
      BmsBasicFrame *frame = (BmsBasicFrame *)(rx_frame->data);
      msg->body.bms_basic_msg.battery_soc = frame->battery_soc;
      msg->body.bms_basic_msg.battery_soh = frame->battery_soh;
      msg->body.bms_basic_msg.voltage =
          (int16_t)((uint16_t)(frame->voltage.low_byte) |
                    (uint16_t)(frame->voltage.high_byte) << 8) *
          0.1f;
      msg->body.bms_basic_msg.current =
          (int16_t)((uint16_t)(frame->current.low_byte) |
                    (uint16_t)(frame->current.high_byte) << 8) *
          0.1f;
      msg->body.bms_basic_msg.temperature =
          (int16_t)((uint16_t)(frame->temperature.low_byte) |
                    (uint16_t)(frame->temperature.high_byte) << 8) *
          0.1f;
      break;
    }
    case CAN_MSG_BMS_EXTENDED_ID: {
      msg->type = AgxMsgBmsExtended;
      BmsExtendedFrame *frame = (BmsExtendedFrame *)(rx_frame->data);
      msg->body.bms_extended_msg.alarm_status_1 = frame->alarm_status_1;
      msg->body.bms_extended_msg.alarm_status_2 = frame->alarm_status_2;
      msg->body.bms_extended_msg.warn_status_1 = frame->warn_status_1;
      msg->body.bms_extended_msg.warn_status_2 = frame->warn_status_2;
      break;
    }
    /*************** query/config frame **************/
    case CAN_MSG_VERSION_REQUEST_ID: {
      msg->type = AgxMsgVersionRequest;
      // TODO
      break;
    }
    case CAN_MSG_VERSION_RESPONSE_ID: {
      msg->type = AgxMsgVersionResponse;
      VersionResponseFrame *frame = (VersionResponseFrame *)(rx_frame->data);
      msg->body.version_response_msg.bytes[0] = frame->res0;
      msg->body.version_response_msg.bytes[1] = frame->res1;
      msg->body.version_response_msg.bytes[2] = frame->res2;
      msg->body.version_response_msg.bytes[3] = frame->res3;
      msg->body.version_response_msg.bytes[4] = frame->res4;
      msg->body.version_response_msg.bytes[5] = frame->res5;
      msg->body.version_response_msg.bytes[6] = frame->res6;
      msg->body.version_response_msg.bytes[7] = frame->res7;
      break;
    }
    case CAN_MSG_CTRL_MODE_CONFIG_ID: {
      msg->type = AgxMsgControlModeConfig;
      ControlModeConfigFrame *frame =
          (ControlModeConfigFrame *)(rx_frame->data);
      msg->body.control_mode_config_msg.mode = frame->mode;
      break;
    }
    case CAN_MSG_STEER_NEUTRAL_REQUEST_ID: {
      msg->type = AgxMsgSteerNeutralRequest;
      // TODO
      break;
    }
    case CAN_MSG_STEER_NEUTRAL_RESPONSE_ID: {
      msg->type = AgxMsgSteerNeutralResponse;
      // TODO
      break;
    }
    case CAN_MSG_STATE_RESET_CONFIG_ID: {
      msg->type = AgxMsgStateResetConfig;
      StateResetConfigFrame *frame = (StateResetConfigFrame *)(rx_frame->data);
      msg->body.state_reset_config_msg.error_clear_byte =
          frame->error_clear_byte;
      break;
    }
    case CAN_MSG_MOTOR_ANGLE_INFO: {
      msg->type = AgxMsgMotorAngle;
      MoterAngleFrame *frame = (MoterAngleFrame *)(rx_frame->data);
      msg->body.motor_angle_msg.angle_5 =
          (int16_t)((uint16_t)(frame->angle_5.low_byte) |
                    (uint16_t)(frame->angle_5.high_byte) << 8) *
          0.001;
      msg->body.motor_angle_msg.angle_6 =
          (int16_t)((uint16_t)(frame->angle_6.low_byte) |
                    (uint16_t)(frame->angle_6.high_byte) << 8) *
          0.001;
      msg->body.motor_angle_msg.angle_7 =
          (int16_t)((uint16_t)(frame->angle_7.low_byte) |
                    (uint16_t)(frame->angle_7.high_byte) << 8) *
          0.001;
      msg->body.motor_angle_msg.angle_8 =
          (int16_t)((uint16_t)(frame->angle_8.low_byte) |
                    (uint16_t)(frame->angle_8.high_byte) << 8) *
          0.001;
      break;
    }
    case CAN_MSG_MOTOR_SPEED_INFO: {
      msg->type = AgxMsgMotorSpeed;
      MoterSpeedFrame *frame = (MoterSpeedFrame *)(rx_frame->data);
      msg->body.motor_speed_msg.speed_1 =
          (int16_t)((uint16_t)(frame->speed_1.low_byte) |
                    (uint16_t)(frame->speed_1.high_byte) << 8) *
          0.001;
      msg->body.motor_speed_msg.speed_2 =
          (int16_t)((uint16_t)(frame->speed_2.low_byte) |
                    (uint16_t)(frame->speed_2.high_byte) << 8) *
          0.001;
      msg->body.motor_speed_msg.speed_3 =
          (int16_t)((uint16_t)(frame->speed_3.low_byte) |
                    (uint16_t)(frame->speed_3.high_byte) << 8) *
          0.001;
      msg->body.motor_speed_msg.speed_4 =
          (int16_t)((uint16_t)(frame->speed_4.low_byte) |
                    (uint16_t)(frame->speed_4.high_byte) << 8) *
          0.001;
      break;
    }
    default:
      ret = false;
      break;
  }

  return ret;
}

bool EncodeCanFrameV2(const AgxMessage *msg, struct can_frame *tx_frame) {
  bool ret = true;
  switch (msg->type) {
    /***************** command frame *****************/
    case AgxMsgMotionCommand: {
      tx_frame->can_id = CAN_MSG_MOTION_COMMAND_ID;
      tx_frame->can_dlc = 8;
      MotionCommandFrame frame;
      int16_t linear_cmd =
          (int16_t)(msg->body.motion_command_msg.linear_velocity * 1000);
      int16_t angular_cmd =
          (int16_t)(msg->body.motion_command_msg.angular_velocity * 1000);
      int16_t lateral_cmd =
          (int16_t)(msg->body.motion_command_msg.lateral_velocity * 1000);
      int16_t steering_cmd =
          (int16_t)(msg->body.motion_command_msg.steering_angle * 1000);
      frame.linear_velocity.high_byte = (uint8_t)(linear_cmd >> 8);
      frame.linear_velocity.low_byte = (uint8_t)(linear_cmd & 0x00ff);
      frame.angular_velocity.high_byte = (uint8_t)(angular_cmd >> 8);
      frame.angular_velocity.low_byte = (uint8_t)(angular_cmd & 0x00ff);
      frame.lateral_velocity.high_byte = (uint8_t)(lateral_cmd >> 8);
      frame.lateral_velocity.low_byte = (uint8_t)(lateral_cmd & 0x00ff);
      frame.steering_angle.high_byte = (uint8_t)(steering_cmd >> 8);
      frame.steering_angle.low_byte = (uint8_t)(steering_cmd & 0x00ff);
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
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
      frame.reserved1 = 0;
      frame.count = count++;
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgBrakingCommand: {
      tx_frame->can_id = CAN_MSG_BRAKING_COMMAND_ID;
      tx_frame->can_dlc = 2;
      BrakingCommandFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgSetMotionModeCommand: {
      tx_frame->can_id = CAN_MSG_SET_MOTION_MODE_ID;
      tx_frame->can_dlc = 8;
      SetMotionModeFrame frame;
      frame.motion_mode = msg->body.motion_mode_msg.motion_mode;
      frame.reserved0 = 0;
      frame.reserved1 = 0;
      frame.reserved2 = 0;
      frame.reserved3 = 0;
      frame.reserved4 = 0;
      frame.reserved5 = 0;
      frame.reserved6 = 0;
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }

    /***************** feedback frame ****************/
    case AgxMsgSystemState: {
      tx_frame->can_id = CAN_MSG_SYSTEM_STATE_ID;
      tx_frame->can_dlc = 8;
      SystemStateFrame frame;
      frame.vehicle_state = msg->body.system_state_msg.vehicle_state;
      frame.control_mode = msg->body.system_state_msg.control_mode;
      uint16_t battery =
          (uint16_t)(msg->body.system_state_msg.battery_voltage * 10);
      frame.battery_voltage.high_byte = (uint8_t)(battery >> 8);
      frame.battery_voltage.low_byte = (uint8_t)(battery & 0x00ff);
      frame.error_code.high_byte =
          (uint8_t)(msg->body.system_state_msg.error_code >> 8);
      frame.error_code.low_byte =
          (uint8_t)(msg->body.system_state_msg.error_code & 0x00ff);
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgMotionState: {
      tx_frame->can_id = CAN_MSG_MOTION_STATE_ID;
      tx_frame->can_dlc = 8;
      MotionStateFrame frame;
      int16_t linear =
          (int16_t)(msg->body.motion_state_msg.linear_velocity * 1000);
      int16_t angular =
          (int16_t)(msg->body.motion_state_msg.angular_velocity * 1000);
      int16_t lateral =
          (int16_t)(msg->body.motion_state_msg.lateral_velocity * 1000);
      int16_t steering =
          (int16_t)(msg->body.motion_state_msg.steering_angle * 1000);
      frame.linear_velocity.high_byte = (uint8_t)(linear >> 8);
      frame.linear_velocity.low_byte = (uint8_t)(linear & 0x00ff);
      frame.angular_velocity.high_byte = (uint8_t)(angular >> 8);
      frame.angular_velocity.low_byte = (uint8_t)(angular & 0x00ff);
      frame.lateral_velocity.high_byte = (uint8_t)(lateral >> 8);
      frame.lateral_velocity.low_byte = (uint8_t)(lateral & 0x00ff);
      frame.steering_angle.high_byte = (uint8_t)(steering >> 8);
      frame.steering_angle.low_byte = (uint8_t)(steering & 0x00ff);
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgMotionModeState: {
      tx_frame->can_id = CAN_MSG_CURRENT_CTRL_MODE;
      tx_frame->can_dlc = 2;
      MotionModeStateFrame frame;
      frame.motion_mode = msg->body.motion_mode_state_msg.motion_mode;
      frame.mode_changing = msg->body.motion_mode_state_msg.mode_changing;
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgLightState: {
      tx_frame->can_id = CAN_MSG_LIGHT_STATE_ID;
      tx_frame->can_dlc = 8;
      LightStateFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgRcState: {
      tx_frame->can_id = CAN_MSG_RC_STATE_ID;
      tx_frame->can_dlc = 8;
      RcStateFrame frame;
      frame.sws = 0;
      // switch a
      if (msg->body.rc_state_msg.swa == RC_SWITCH_UP)
        frame.sws |= RC_SWA_UP_MASK;
      else if (msg->body.rc_state_msg.swa == RC_SWITCH_DOWN)
        frame.sws |= RC_SWA_DOWN_MASK;
      // switch b
      if (msg->body.rc_state_msg.swb == RC_SWITCH_UP)
        frame.sws |= RC_SWB_UP_MASK;
      else if (msg->body.rc_state_msg.swb == RC_SWITCH_MIDDLE)
        frame.sws |= RC_SWB_MIDDLE_MASK;
      else if (msg->body.rc_state_msg.swb == RC_SWITCH_DOWN)
        frame.sws |= RC_SWB_DOWN_MASK;
      // switch c
      if (msg->body.rc_state_msg.swc == RC_SWITCH_UP)
        frame.sws |= RC_SWC_UP_MASK;
      else if (msg->body.rc_state_msg.swc == RC_SWITCH_MIDDLE)
        frame.sws |= RC_SWC_MIDDLE_MASK;
      else if (msg->body.rc_state_msg.swc == RC_SWITCH_DOWN)
        frame.sws |= RC_SWC_DOWN_MASK;
      // switch d
      if (msg->body.rc_state_msg.swd == RC_SWITCH_UP)
        frame.sws |= RC_SWD_UP_MASK;
      else if (msg->body.rc_state_msg.swd == RC_SWITCH_DOWN)
        frame.sws |= RC_SWD_DOWN_MASK;
      frame.stick_right_v = msg->body.rc_state_msg.stick_right_v;
      frame.stick_right_h = msg->body.rc_state_msg.stick_right_h;
      frame.stick_left_v = msg->body.rc_state_msg.stick_left_v;
      frame.stick_left_h = msg->body.rc_state_msg.stick_left_h;
      frame.var_a = msg->body.rc_state_msg.var_a;
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgActuatorHSState: {
      tx_frame->can_id = msg->body.actuator_hs_state_msg.motor_id +
                         CAN_MSG_ACTUATOR1_HS_STATE_ID;
      tx_frame->can_dlc = 8;
      ActuatorHSStateFrame frame;
      int16_t rpm = msg->body.actuator_hs_state_msg.rpm;
      int16_t current = msg->body.actuator_hs_state_msg.current * 10;
      int32_t pulse_count = msg->body.actuator_hs_state_msg.pulse_count;
      frame.rpm.high_byte = (uint8_t)(rpm >> 8);
      frame.rpm.low_byte = (uint8_t)(rpm & 0x00ff);
      frame.current.high_byte = (uint8_t)(current >> 8);
      frame.current.low_byte = (uint8_t)(current & 0x00ff);
      frame.pulse_count.lsb = (uint8_t)(pulse_count & 0x000000ff);
      frame.pulse_count.low_byte = (uint8_t)((pulse_count >> 8) & 0x000000ff);
      frame.pulse_count.high_byte = (uint8_t)((pulse_count >> 16) & 0x000000ff);
      frame.pulse_count.msb = (uint8_t)((pulse_count >> 24) & 0x000000ff);
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgActuatorLSState: {
      tx_frame->can_id = msg->body.actuator_ls_state_msg.motor_id +
                         CAN_MSG_ACTUATOR1_LS_STATE_ID;
      tx_frame->can_dlc = 8;
      ActuatorLSStateFrame frame;
      int16_t driver_voltage =
          (int16_t)(msg->body.actuator_ls_state_msg.driver_voltage * 10);
      int16_t driver_temp =
          (int16_t)(msg->body.actuator_ls_state_msg.driver_temp);
      frame.driver_voltage.high_byte = (uint8_t)(driver_voltage >> 8);
      frame.driver_voltage.low_byte = (uint8_t)(driver_voltage & 0x00ff);
      frame.driver_temp.high_byte = (uint8_t)(driver_temp >> 8);
      frame.driver_temp.low_byte = (uint8_t)(driver_temp & 0x00ff);
      frame.motor_temp = msg->body.actuator_ls_state_msg.motor_temp;
      frame.driver_state = msg->body.actuator_ls_state_msg.driver_state;
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    /****************** sensor frame *****************/
    case AgxMsgOdometry: {
      tx_frame->can_id = CAN_MSG_ODOMETRY_ID;
      tx_frame->can_dlc = 8;
      OdometryFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgImuAccel: {
      tx_frame->can_id = CAN_MSG_IMU_ACCEL_ID;
      tx_frame->can_dlc = 8;
      ImuAccelFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgImuGyro: {
      tx_frame->can_id = CAN_MSG_IMU_GYRO_ID;
      tx_frame->can_dlc = 8;
      ImuGyroFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgImuEuler: {
      tx_frame->can_id = CAN_MSG_IMU_EULER_ID;
      tx_frame->can_dlc = 8;
      ImuEulerFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgSafetyBumper: {
      tx_frame->can_id = CAN_MSG_SAFETY_BUMPER_ID;
      tx_frame->can_dlc = 8;
      SafetyBumperFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgUltrasonic: {
      tx_frame->can_id =
          msg->body.ultrasonic_msg.sensor_id + CAN_MSG_ULTRASONIC_1_ID;
      tx_frame->can_dlc = 8;
      UltrasonicFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgUwb: {
      tx_frame->can_id = msg->body.uwb_msg.sensor_id + CAN_MSG_UWB_1_ID;
      tx_frame->can_dlc = 8;
      UwbFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgBmsBasic: {
      tx_frame->can_id = CAN_MSG_BMS_BASIC_ID;
      tx_frame->can_dlc = 8;
      BmsBasicFrame frame;
      frame.battery_soc = msg->body.bms_basic_msg.battery_soc;
      frame.battery_soh = msg->body.bms_basic_msg.battery_soh;
      int16_t voltage = (int16_t)(msg->body.bms_basic_msg.voltage * 10);
      int16_t current = (int16_t)(msg->body.bms_basic_msg.current * 10);
      int16_t temperature = (int16_t)(msg->body.bms_basic_msg.temperature * 10);
      frame.voltage.high_byte = (uint8_t)(voltage >> 8);
      frame.voltage.low_byte = (uint8_t)(voltage & 0x00ff);
      frame.current.high_byte = (uint8_t)(current >> 8);
      frame.current.low_byte = (uint8_t)(current & 0x00ff);
      frame.temperature.high_byte = (uint8_t)(temperature >> 8);
      frame.temperature.low_byte = (uint8_t)(temperature & 0x00ff);
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgBmsExtended: {
      tx_frame->can_id = CAN_MSG_BMS_EXTENDED_ID;
      tx_frame->can_dlc = 8;
      BmsExtendedFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    /*************** query/config frame **************/
    case AgxMsgVersionRequest: {
      tx_frame->can_id = CAN_MSG_VERSION_REQUEST_ID;
      tx_frame->can_dlc = 8;
      VersionRequestFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgVersionResponse: {
      tx_frame->can_id = CAN_MSG_VERSION_RESPONSE_ID;
      tx_frame->can_dlc = 8;
      VersionResponseFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgControlModeConfig: {
      tx_frame->can_id = CAN_MSG_CTRL_MODE_CONFIG_ID;
      tx_frame->can_dlc = 8;
      ControlModeConfigFrame frame;
      frame.mode = msg->body.control_mode_config_msg.mode;
      frame.reserved0 = 0;
      frame.reserved1 = 0;
      frame.reserved2 = 0;
      frame.reserved3 = 0;
      frame.reserved4 = 0;
      frame.reserved5 = 0;
      frame.reserved6 = 0;
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgBrakeModeConfig: {
      tx_frame->can_id = CAN_MSG_BRAKING_COMMAND_ID;
      tx_frame->can_dlc = 8;
      BrakeModeConfigFrame frame;
      frame.mode = msg->body.control_mode_config_msg.mode;
      frame.reserved0 = 0;
      frame.reserved1 = 0;
      frame.reserved2 = 0;
      frame.reserved3 = 0;
      frame.reserved4 = 0;
      frame.reserved5 = 0;
      frame.reserved6 = 0;
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgSteerNeutralRequest: {
      tx_frame->can_id = CAN_MSG_STEER_NEUTRAL_REQUEST_ID;
      tx_frame->can_dlc = 8;
      SteerNeutralRequestFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgSteerNeutralResponse: {
      tx_frame->can_id = CAN_MSG_STEER_NEUTRAL_RESPONSE_ID;
      tx_frame->can_dlc = 8;
      SteerNeutralResponseFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    case AgxMsgStateResetConfig: {
      tx_frame->can_id = CAN_MSG_STATE_RESET_CONFIG_ID;
      tx_frame->can_dlc = 8;
      StateResetConfigFrame frame;
      // TODO
      memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
      break;
    }
    default: {
      ret = false;
      break;
    }
  }
  return ret;
}

uint8_t CalcCanFrameChecksumV2(uint16_t id, uint8_t *data, uint8_t dlc) {
  uint8_t checksum = 0x00;
  checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
  for (int i = 0; i < (dlc - 1); ++i) checksum += data[i];
  return checksum;
}
