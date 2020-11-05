/*
 * agx_msg_parser.c
 *
 * Created on: Aug 31, 2019 04:25
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "ugv_sdk/proto/agx_msg_parser.h"

#include "string.h"

// #define USE_XOR_CHECKSUM

// #define PRINT_CPP_DEBUG_INFO
// #define PRINT_JLINK_DEBUG_INFO

#ifdef PRINT_CPP_DEBUG_INFO
#define < iostream>
#elif (defined(PRINT_JLINK_DEBUG_INFO))
#include "segger/jlink_rtt.h"
#endif

typedef enum {
  WAIT_FOR_SOF1 = 0,
  WAIT_FOR_SOF2,
  WAIT_FOR_FRAME_LEN,
  WAIT_FOR_FRAME_TYPE,
  WAIT_FOR_FRAME_ID,
  WAIT_FOR_PAYLOAD,
  WAIT_FOR_FRAME_COUNT,
  WAIT_FOR_CHECKSUM
} UartDecodeState;

#define UART_MSG_BUFFER_SIZE 64
#define UART_MSG_SOF_LEN ((uint8_t)2)
#define UART_MSG_FIXED_FIELD_LEN ((uint8_t)4)
#define UART_MSG_SOF1 ((uint8_t)0x5a)
#define UART_MSG_SOF2 ((uint8_t)0xa5)
#define UART_MSG_TYPE_CONTROL ((uint8_t)0x55)
#define UART_MSG_TYPE_STATUS ((uint8_t)0xaa)
#define UART_MSG_NONE_ID ((uint8_t)0x00)

// frame buffer
static struct {
  uint8_t frame_id;
  uint8_t frame_type;
  uint8_t frame_len;
  uint8_t frame_cnt;
  uint8_t frame_checksum;
  uint8_t internal_checksum;
  uint8_t payload_buffer[UART_MSG_BUFFER_SIZE];
  size_t payload_data_pos;
} uart_parsing_data;

// statisctics
typedef struct {
  uint32_t frame_parsed;
  uint32_t frame_with_wrong_checksum;
} UartParsingStats;

static UartParsingStats uart_parsing_stats = {.frame_parsed = true,
                                              .frame_with_wrong_checksum = 123};

// internal functions
static bool ParseChar(uint8_t c, AgxMessage *msg);
static uint8_t CalcBufferedFrameChecksum();
static bool ConstructStatusMessage(AgxMessage *msg);
static bool ConstructControlMessage(AgxMessage *msg);

static void EncodeMotionControlMsgToUART(const MotionControlMessage *msg,
                                         uint8_t *buf, uint8_t *len);
static void EncodeLightControlMsgToUART(const LightControlMessage *msg,
                                        uint8_t *buf, uint8_t *len);

static void EncodeScoutMotionControlMsgToCAN(const MotionControlMessage *msg,
                                             struct can_frame *tx_frame);
static void EncodeScoutLightControlMsgToCAN(const LightControlMessage *msg,
                                            struct can_frame *tx_frame);
static void EncodeScoutnControlModeMsgToCAN(const ModSelectMessage *msg,
                                            struct can_frame *tx_frame);

bool DecodeCanFrame(const struct can_frame *rx_frame, AgxMessage *msg) {
  msg->type = ScoutMsgNone;

  switch (rx_frame->can_id) {
    // in the current implementation, both MsgType and can_frame include 8 *
    // uint8_t
    case CAN_MSG_MOTION_STATE_ID: {
      msg->type = ScoutMotionStatusMsg;
      // msg->motion_status_msg.id = CAN_MSG_MOTION_STATE_ID;
      memcpy(msg->body.motion_status_msg.data.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_LIGHT_CONTROL_STATUS_ID: {
      msg->type = ScoutLightStatusMsg;
      // msg->light_status_msg.id = CAN_MSG_LIGHT_CONTROL_STATUS_ID;
      memcpy(msg->body.light_status_msg.data.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_SYSTEM_STATE_ID: {
      msg->type = ScoutSystemStatusMsg;
      // msg->system_status_msg.id = CAN_MSG_SYSTEM_STATE_ID;
      memcpy(msg->body.system_status_msg.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    // in the current implementation, both MsgType and can_frame include 8 *
    // uint8_t
    case CAN_MSG_MOTION_CONTROL_CMD_ID: {
      msg->type = ScoutMotionControlMsg;
      // msg->motion_control_msg.id = CAN_MSG_MOTION_CONTROL_CMD_ID;
      memcpy(msg->body.motion_control_msg.data.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_LIGHT_CONTROL_CMD_ID: {
      msg->type = ScoutLightControlMsg;
      // msg->light_control_msg.id = CAN_MSG_LIGHT_CONTROL_STATUS_ID;
      memcpy(msg->body.light_control_msg.data.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_MOTOR1_LOW_DRIVER_STATUS_ID: {
      msg->type = ScoutMotorDriverLowSpeedStatusMsg;
      msg->body.motor_driver_low_speed_status_msg.motor_id = SCOUT_MOTOR1_ID;
      memcpy(msg->body.motor_driver_low_speed_status_msg.data.raw,
             rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_MOTOR2_LOW_DRIVER_STATUS_ID: {
      msg->type = ScoutMotorDriverLowSpeedStatusMsg;
      msg->body.motor_driver_low_speed_status_msg.motor_id = SCOUT_MOTOR2_ID;
      memcpy(msg->body.motor_driver_low_speed_status_msg.data.raw,
             rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_MOTOR3_LOW_DRIVER_STATUS_ID: {
      msg->type = ScoutMotorDriverLowSpeedStatusMsg;
      msg->body.motor_driver_low_speed_status_msg.motor_id = SCOUT_MOTOR3_ID;
      memcpy(msg->body.motor_driver_low_speed_status_msg.data.raw,
             rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_MOTOR4_LOW_DRIVER_STATUS_ID: {
      msg->type = ScoutMotorDriverLowSpeedStatusMsg;
      msg->body.motor_driver_low_speed_status_msg.motor_id = SCOUT_MOTOR4_ID;
      memcpy(msg->body.motor_driver_low_speed_status_msg.data.raw,
             rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_MOTOR1_HEIGHT_DRIVER_STATUS_ID: {
      msg->type = ScoutMotorDriverHeightSpeedStatusMsg;
      msg->body.motor_driver_height_speed_status_msg.motor_id = SCOUT_MOTOR1_ID;
      memcpy(msg->body.motor_driver_height_speed_status_msg.data.raw,
             rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_MOTOR2_HEIGHT_DRIVER_STATUS_ID: {
      msg->type = ScoutMotorDriverHeightSpeedStatusMsg;
      msg->body.motor_driver_height_speed_status_msg.motor_id = SCOUT_MOTOR2_ID;
      memcpy(msg->body.motor_driver_height_speed_status_msg.data.raw,
             rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_MOTOR3_HEIGHT_DRIVER_STATUS_ID: {
      msg->type = ScoutMotorDriverHeightSpeedStatusMsg;
      msg->body.motor_driver_height_speed_status_msg.motor_id = SCOUT_MOTOR3_ID;
      memcpy(msg->body.motor_driver_height_speed_status_msg.data.raw,
             rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_MOTOR4_HEIGHT_DRIVER_STATUS_ID: {
      msg->type = ScoutMotorDriverHeightSpeedStatusMsg;
      msg->body.motor_driver_height_speed_status_msg.motor_id = SCOUT_MOTOR4_ID;
      memcpy(msg->body.motor_driver_height_speed_status_msg.data.raw,
             rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
      break;
    }
    case CAN_MSG_ODOMETER_ID: {
      msg->type = ScoutodometerMsg;
      // msg->light_control_msg.id = CAN_MSG_LIGHT_CONTROL_STATUS_ID;
      memcpy(msg->body.odom_msg.data.raw, rx_frame->data,
             rx_frame->can_dlc * sizeof(uint8_t));
      //        printf("%x\t",msg->body.odom_msg.data.status.left.heighest);
      //        printf("%x\t",msg->body.odom_msg.data.status.left.sec_heighest);
      //        printf("%x\t",msg->body.odom_msg.data.status.left.sec_lowest);
      //        printf("%x\r\n",msg->body.odom_msg.data.status.left.lowest);
      //        printf("%x\t",msg->body.odom_msg.data.raw);
      //        printf("%x\r\n",msg->body.odom_msg.data.status);
      break;
    }
    default:
      break;
  }

  return true;
}

void EncodeCanFrame(const AgxMessage *msg, struct can_frame *tx_frame) {
  switch (msg->type) {
    // in the current implementation, both MsgType and can_frame include 8 *
    // uint8_t
    case ScoutMotionStatusMsg: {
      tx_frame->can_id = CAN_MSG_MOTION_STATE_ID;
      tx_frame->can_dlc = 8;
      memcpy(tx_frame->data, msg->body.motion_status_msg.data.raw,
             tx_frame->can_dlc);
      break;
    }
    case ScoutLightStatusMsg: {
      tx_frame->can_id = CAN_MSG_LIGHT_CONTROL_STATUS_ID;
      tx_frame->can_dlc = 8;
      memcpy(tx_frame->data, msg->body.light_status_msg.data.raw,
             tx_frame->can_dlc);
      break;
    }
    case ScoutSystemStatusMsg: {
      tx_frame->can_id = CAN_MSG_SYSTEM_STATE_ID;
      tx_frame->can_dlc = 8;
      memcpy(tx_frame->data, msg->body.system_status_msg.raw,
             tx_frame->can_dlc);
      break;
    }
      //    case ScoutMotorDriverStatusMsg:
      //    {
      //        if (msg->body.motor_driver_status_msg.motor_id ==
      //        SCOUT_MOTOR1_ID)
      //            tx_frame->can_id = CAN_MSG_MOTOR1_DRIVER_STATUS_ID;
      //        else if (msg->body.motor_driver_status_msg.motor_id ==
      //        SCOUT_MOTOR2_ID)
      //            tx_frame->can_id = CAN_MSG_MOTOR2_DRIVER_STATUS_ID;
      //        else if (msg->body.motor_driver_status_msg.motor_id ==
      //        SCOUT_MOTOR3_ID)
      //            tx_frame->can_id = CAN_MSG_MOTOR3_DRIVER_STATUS_ID;
      //        else if (msg->body.motor_driver_status_msg.motor_id ==
      //        SCOUT_MOTOR4_ID)
      //            tx_frame->can_id = CAN_MSG_MOTOR4_DRIVER_STATUS_ID;
      //        tx_frame->can_dlc = 8;
      //        memcpy(tx_frame->data,
      //        msg->body.motor_driver_status_msg.data.raw, tx_frame->can_dlc);
      //        break;
      //    }
    case ScoutMotionControlMsg: {
      EncodeScoutMotionControlMsgToCAN(&(msg->body.motion_control_msg),
                                       tx_frame);
      break;
    }
    // TODO check correctiveness
    // case ScoutControlModeMsg: {
    //   EncodeScoutnControlModeMsgToCAN(&(msg->body.motion_control_msg),
    //                                   tx_frame);
    //   break;
    // }
    case ScoutLightControlMsg: {
      EncodeScoutLightControlMsgToCAN(&(msg->body.light_control_msg), tx_frame);
      break;
    }
    default:
      break;
  }
  //    tx_frame->data[7] = CalcCanFrameChecksum(tx_frame->can_id,
  //    tx_frame->data, tx_frame->can_dlc);
}

void EncodeScoutMotionControlMsgToCAN(const MotionControlMessage *msg,
                                      struct can_frame *tx_frame) {
  tx_frame->can_id = CAN_MSG_MOTION_CONTROL_CMD_ID;
  tx_frame->can_dlc = 8;
  memcpy(tx_frame->data, msg->data.raw, tx_frame->can_dlc);
  tx_frame->data[7] =
      CalcCanFrameChecksum(tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
}
void EncodeScoutnControlModeMsgToCAN(const ModSelectMessage *msg,
                                     struct can_frame *tx_frame) {
  tx_frame->can_id = CAN_MSG_SELECT_CONTROL_MODE_ID;
  tx_frame->can_dlc = 8;
  memcpy(tx_frame->data, msg->data.raw, tx_frame->can_dlc);
  tx_frame->data[7] =
      CalcCanFrameChecksum(tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
}
void EncodeScoutLightControlMsgToCAN(const LightControlMessage *msg,
                                     struct can_frame *tx_frame) {
  tx_frame->can_id = CAN_MSG_LIGHT_CONTROL_CMD_ID;
  tx_frame->can_dlc = 8;
  memcpy(tx_frame->data, msg->data.raw, tx_frame->can_dlc);
  tx_frame->data[7] =
      CalcCanFrameChecksum(tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
}

uint8_t CalcCanFrameChecksum(uint16_t id, uint8_t *data, uint8_t dlc) {
  uint8_t checksum = 0x00;
  checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
  for (int i = 0; i < (dlc - 1); ++i) checksum += data[i];
  return checksum;
}

void EncodeUartData(const AgxMessage *msg, uint8_t *buf, uint8_t *len) {
  // SOF
  buf[0] = UART_MSG_SOF1;
  buf[1] = UART_MSG_SOF2;

  // frame len, type, ID
  buf[2] = 0x0a;
  buf[3] = UART_MSG_TYPE_STATUS;

  switch (msg->type) {
    // in the current implementation, both MsgType and can_frame include 8 *
    // uint8_t
    case ScoutMotionStatusMsg: {
      buf[4] = UART_MSG_MOTION_STATUS_ID;
      buf[5] = msg->body.motion_status_msg.data.cmd.linear_velocity.high_byte;
      buf[6] = msg->body.motion_status_msg.data.cmd.linear_velocity.low_byte;
      buf[7] = msg->body.motion_status_msg.data.cmd.angular_velocity.high_byte;
      buf[8] = msg->body.motion_status_msg.data.cmd.angular_velocity.low_byte;
      buf[9] = 0;
      buf[10] = 0;
      buf[11] = 0;
      break;
    }
    case ScoutLightStatusMsg: {
      buf[4] = UART_MSG_LIGHT_STATUS_ID;
      buf[5] = msg->body.light_status_msg.data.status.light_ctrl_enable;
      buf[6] = msg->body.light_status_msg.data.status.front_light_mode;
      buf[7] = msg->body.light_status_msg.data.status.front_light_custom;
      buf[8] = msg->body.light_status_msg.data.status.rear_light_mode;
      buf[9] = msg->body.light_status_msg.data.status.rear_light_custom;
      buf[10] = 0;
      buf[11] = msg->body.light_status_msg.data.status.count;
      break;
    }
    case ScoutSystemStatusMsg: {
      buf[4] = UART_MSG_SYSTEM_STATUS_ID;
      buf[5] = msg->body.system_status_msg.status.vehicle_state;
      buf[6] = msg->body.system_status_msg.status.control_mode;
      buf[7] =
          msg->body.system_status_msg.status.battery_voltage.high_byte;
      buf[8] = msg->body.system_status_msg.status.battery_voltage.low_byte;
      buf[9] = msg->body.system_status_msg.status.fault_code;
      buf[10] = 0;
      buf[11] = 0;
      break;
    }
    case ScoutMotorDriverStatusMsg: {
      if (msg->body.motor_driver_status_msg.motor_id == SCOUT_MOTOR1_ID)
        buf[4] = UART_MSG_MOTOR1_DRIVER_STATUS_ID;
      else if (msg->body.motor_driver_status_msg.motor_id == SCOUT_MOTOR2_ID)
        buf[4] = UART_MSG_MOTOR2_DRIVER_STATUS_ID;
      else if (msg->body.motor_driver_status_msg.motor_id == SCOUT_MOTOR3_ID)
        buf[4] = UART_MSG_MOTOR3_DRIVER_STATUS_ID;
      else if (msg->body.motor_driver_status_msg.motor_id == SCOUT_MOTOR4_ID)
        buf[4] = UART_MSG_MOTOR4_DRIVER_STATUS_ID;
      buf[5] = msg->body.motor_driver_status_msg.data.status.current.high_byte;
      buf[6] = msg->body.motor_driver_status_msg.data.status.current.low_byte;
      buf[7] = msg->body.motor_driver_status_msg.data.status.rpm.high_byte;
      buf[8] = msg->body.motor_driver_status_msg.data.status.rpm.low_byte;
      buf[9] = msg->body.motor_driver_status_msg.data.status.temperature;
      buf[10] = 0;
      buf[11] = msg->body.motor_driver_status_msg.data.status.count;
      break;
    }
    case ScoutMotionControlMsg: {
      EncodeMotionControlMsgToUART(&(msg->body.motion_control_msg), buf, len);
      break;
    }
    case ScoutLightControlMsg: {
      EncodeLightControlMsgToUART(&(msg->body.light_control_msg), buf, len);
      break;
    }
    default:
      break;
  }

  buf[12] = CalcUartDataChecksum(buf, buf[2] + UART_MSG_SOF_LEN);

  // length: SOF + Frame + Checksum
  *len = buf[2] + UART_MSG_SOF_LEN + 1;
}

bool DecodeUartData(uint8_t c, AgxMessage *msg) {
  static AgxMessage decoded_msg;

  bool result = ParseChar(c, &decoded_msg);
  if (result) *msg = decoded_msg;
  return result;
}

void EncodeMotionControlMsgToUART(const MotionControlMessage *msg, uint8_t *buf,
                                  uint8_t *len) {
  // SOF
  buf[0] = UART_MSG_SOF1;
  buf[1] = UART_MSG_SOF2;

  // frame len, type, ID
  buf[2] = 0x0a;
  buf[3] = UART_MSG_TYPE_CONTROL;
  buf[4] = UART_MSG_MOTION_CONTROL_ID;

  // frame payload
  buf[5] = msg->data.cmd.linear_velocity.high_byte;
  buf[6] = msg->data.cmd.linear_velocity.low_byte;
  buf[7] = msg->data.cmd.angular_velocity.high_byte;
  buf[8] = msg->data.cmd.angular_velocity.low_byte;
  buf[9] = 0x00;
  buf[10] = 0x00;

  // frame count, checksum
  buf[11] = 0x00;
  buf[12] = 0x00;

  // length: SOF + Frame + Checksum
  *len = buf[2] + UART_MSG_SOF_LEN + 1;
}

void EncodeLightControlMsgToUART(const LightControlMessage *msg, uint8_t *buf,
                                 uint8_t *len) {
  // SOF
  buf[0] = UART_MSG_SOF1;
  buf[1] = UART_MSG_SOF2;

  // frame len, type, ID
  buf[2] = 0x0a;
  buf[3] = UART_MSG_TYPE_CONTROL;
  buf[4] = UART_MSG_LIGHT_CONTROL_ID;

  // frame payload
  buf[5] = msg->data.cmd.light_ctrl_enable;
  buf[6] = msg->data.cmd.front_light_mode;
  buf[7] = msg->data.cmd.front_light_custom;
  buf[8] = msg->data.cmd.rear_light_mode;
  buf[9] = msg->data.cmd.rear_light_custom;
  buf[10] = 0x00;

  // frame count, checksum
  buf[11] = msg->data.cmd.count;
  buf[12] = CalcUartDataChecksum(buf, buf[2] + UART_MSG_SOF_LEN);

  // length: SOF + Frame + Checksum
  *len = buf[2] + UART_MSG_SOF_LEN + 1;
}

bool ParseChar(uint8_t c, AgxMessage *msg) {
  static UartDecodeState decode_state = WAIT_FOR_SOF1;

  bool new_frame_parsed = false;
  switch (decode_state) {
    case WAIT_FOR_SOF1: {
      if (c == UART_MSG_SOF1) {
        uart_parsing_data.frame_id = UART_MSG_NONE_ID;
        uart_parsing_data.frame_type = 0;
        uart_parsing_data.frame_len = 0;
        uart_parsing_data.frame_cnt = 0;
        uart_parsing_data.frame_checksum = 0;
        uart_parsing_data.internal_checksum = 0;
        uart_parsing_data.payload_data_pos = 0;
        memset(uart_parsing_data.payload_buffer, 0, UART_MSG_BUFFER_SIZE);

        decode_state = WAIT_FOR_SOF2;
#ifdef PRINT_CPP_DEBUG_INFO
        std::cout << "found sof1" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
        JLinkWriteString(0, "found sof1\n");
#endif
      }
      break;
    }
    case WAIT_FOR_SOF2: {
      if (c == UART_MSG_SOF2) {
        decode_state = WAIT_FOR_FRAME_LEN;
#ifdef PRINT_CPP_DEBUG_INFO
        std::cout << "found sof2" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
        JLinkWriteString(0, "found sof2\n");
#endif
      } else {
        decode_state = WAIT_FOR_SOF1;
#ifdef PRINT_CPP_DEBUG_INFO
        std::cout << "failed to find sof2" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
        JLinkWriteString(0, "failed to find sof2\n");
#endif
      }
      break;
    }
    case WAIT_FOR_FRAME_LEN: {
      uart_parsing_data.frame_len = c;
      decode_state = WAIT_FOR_FRAME_TYPE;
#ifdef PRINT_CPP_DEBUG_INFO
      std::cout << "frame len: " << std::hex << static_cast<int>(frame_len)
                << std::dec << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
      JLinkRTTPrintf(0, "frame len: %d\n", frame_len);
#endif
      break;
    }
    case WAIT_FOR_FRAME_TYPE: {
      switch (c) {
        case UART_MSG_TYPE_CONTROL: {
          uart_parsing_data.frame_type = UART_MSG_TYPE_CONTROL;
          decode_state = WAIT_FOR_FRAME_ID;
#ifdef PRINT_CPP_DEBUG_INFO
          std::cout << "control type frame received" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
          JLinkWriteString(0, "control type frame received\n");
#endif
          break;
        }
        case UART_MSG_TYPE_STATUS: {
          uart_parsing_data.frame_type = UART_MSG_TYPE_STATUS;
          decode_state = WAIT_FOR_FRAME_ID;
#ifdef PRINT_CPP_DEBUG_INFO
          std::cout << "status type frame received" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
          JLinkWriteString(0, "status type frame received\n");
#endif
          break;
        }
        default: {
#ifdef PRINT_CPP_DEBUG_INFO
          std::cerr << "ERROR: Not expecting frame of a type other than "
                       "UART_MSG_TYPE_STATUS"
                    << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
          JLinkWriteString(0,
                           "ERROR: Not expecting frame of a type other than "
                           "UART_MSG_TYPE_STATUS\n");
#endif
          decode_state = WAIT_FOR_SOF1;
        }
      }
      break;
    }
    case WAIT_FOR_FRAME_ID: {
      switch (c) {
        case UART_MSG_SYSTEM_STATUS_ID:
        case UART_MSG_MOTION_STATUS_ID:
        case UART_MSG_MOTOR1_DRIVER_STATUS_ID:
        case UART_MSG_MOTOR2_DRIVER_STATUS_ID:
        case UART_MSG_MOTOR3_DRIVER_STATUS_ID:
        case UART_MSG_MOTOR4_DRIVER_STATUS_ID:
        case UART_MSG_LIGHT_STATUS_ID: {
          uart_parsing_data.frame_id = c;
          decode_state = WAIT_FOR_PAYLOAD;
#ifdef PRINT_CPP_DEBUG_INFO
          std::cout << "frame id: " << std::hex << static_cast<int>(frame_id)
                    << std::dec << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
          JLinkRTTPrintf(0, "frame id: %d\n", frame_id);
#endif
          break;
        }
        default: {
#ifdef PRINT_CPP_DEBUG_INFO
          std::cerr << "ERROR: Unknown frame id" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
          JLinkWriteString(0, "ERROR: Unknown frame id\n");
#endif
          decode_state = WAIT_FOR_SOF1;
        }
      }
      break;
    }
    case WAIT_FOR_PAYLOAD: {
      uart_parsing_data.payload_buffer[uart_parsing_data.payload_data_pos++] =
          c;
#ifdef PRINT_CPP_DEBUG_INFO
      std::cout << "1 byte added: " << std::hex << static_cast<int>(c)
                << std::dec << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
      JLinkRTTPrintf(0, "1 byte added: %d\n", c);
#endif
      if (uart_parsing_data.payload_data_pos ==
          (uart_parsing_data.frame_len - UART_MSG_FIXED_FIELD_LEN))
        decode_state = WAIT_FOR_FRAME_COUNT;
      break;
    }
    case WAIT_FOR_FRAME_COUNT: {
      uart_parsing_data.frame_cnt = c;
      decode_state = WAIT_FOR_CHECKSUM;
#ifdef PRINT_CPP_DEBUG_INFO
      std::cout << "frame count: " << std::hex << static_cast<int>(frame_cnt)
                << std::dec << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
      JLinkRTTPrintf(0, "frame count: %d\n", frame_cnt);
#endif
      break;
    }
    case WAIT_FOR_CHECKSUM: {
      uart_parsing_data.frame_checksum = c;
      uart_parsing_data.internal_checksum = CalcBufferedFrameChecksum();
      new_frame_parsed = true;
      decode_state = WAIT_FOR_SOF1;
#ifdef PRINT_CPP_DEBUG_INFO
      std::cout << "--- frame checksum: " << std::hex
                << static_cast<int>(frame_checksum) << std::dec << std::endl;
      std::cout << "--- internal frame checksum: " << std::hex
                << static_cast<int>(internal_checksum) << std::dec << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
      JLinkRTTPrintf(0, "--- frame checksum: : %d\n", frame_checksum);
      JLinkRTTPrintf(0, "--- internal frame checksum: : %d\n",
                     internal_checksum);
#endif
      break;
    }
    default:
      break;
  }

  if (new_frame_parsed) {
    if (uart_parsing_data.frame_checksum ==
        uart_parsing_data.internal_checksum) {
#ifdef PRINT_CPP_DEBUG_INFO
      std::cout << "checksum correct" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
      JLinkWriteString(0, "checksum correct\n");
#endif
      if (uart_parsing_data.frame_type == UART_MSG_TYPE_STATUS)
        ConstructStatusMessage(msg);
      else if (uart_parsing_data.frame_type == UART_MSG_TYPE_CONTROL)
        ConstructControlMessage(msg);
      ++uart_parsing_stats.frame_parsed;
    } else {
      ++uart_parsing_stats.frame_with_wrong_checksum;
#ifdef PRINT_CPP_DEBUG_INFO
      std::cout << "checksum is NOT correct" << std::endl;
      std::cout << std::hex << static_cast<int>(frame_id) << " , "
                << static_cast<int>(frame_len) << " , "
                << static_cast<int>(frame_cnt) << " , "
                << static_cast<int>(frame_checksum) << " : " << std::dec
                << std::endl;
      std::cout << "payload: ";
      for (int i = 0; i < payload_data_pos; ++i)
        std::cout << std::hex << static_cast<int>(payload_buffer[i]) << std::dec
                  << " ";
      std::cout << std::endl;
      std::cout << "--- frame checksum: " << std::hex
                << static_cast<int>(frame_checksum) << std::dec << std::endl;
      std::cout << "--- internal frame checksum: " << std::hex
                << static_cast<int>(internal_checksum) << std::dec << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
      JLinkWriteString(0, "checksum is NOT correct\n");
#endif
    }
  }

  return new_frame_parsed;
}

bool ConstructControlMessage(AgxMessage *msg) {
  if (msg == NULL) return false;

  switch (uart_parsing_data.frame_id) {
    case UART_MSG_MOTION_CONTROL_ID: {
      msg->type = ScoutMotionControlMsg;
      msg->body.motion_control_msg.data.cmd.linear_velocity.high_byte =
          uart_parsing_data.payload_buffer[0];
      msg->body.motion_control_msg.data.cmd.linear_velocity.low_byte =
          uart_parsing_data.payload_buffer[1];
      msg->body.motion_control_msg.data.cmd.angular_velocity.high_byte =
          uart_parsing_data.payload_buffer[2];
      msg->body.motion_control_msg.data.cmd.angular_velocity.low_byte =
          uart_parsing_data.payload_buffer[3];
      msg->body.motion_control_msg.data.cmd.reserved0 =
          uart_parsing_data.payload_buffer[4];
      msg->body.motion_control_msg.data.cmd.reserved1 =
          uart_parsing_data.payload_buffer[5];
      msg->body.motion_control_msg.data.cmd.reserved2 =
          uart_parsing_data.payload_buffer[6];
      msg->body.motion_control_msg.data.cmd.reserved3 =
          uart_parsing_data.payload_buffer[7];
      break;
    }
    case UART_MSG_LIGHT_CONTROL_ID: {
      msg->type = ScoutLightControlMsg;
      msg->body.light_control_msg.data.cmd.light_ctrl_enable =
          uart_parsing_data.payload_buffer[0];
      msg->body.light_control_msg.data.cmd.front_light_mode =
          uart_parsing_data.payload_buffer[1];
      msg->body.light_control_msg.data.cmd.front_light_custom =
          uart_parsing_data.payload_buffer[2];
      msg->body.light_control_msg.data.cmd.rear_light_mode =
          uart_parsing_data.payload_buffer[3];
      msg->body.light_control_msg.data.cmd.rear_light_custom =
          uart_parsing_data.payload_buffer[4];
      msg->body.light_control_msg.data.cmd.reserved0 =
          uart_parsing_data.payload_buffer[5];
      msg->body.light_control_msg.data.cmd.count = uart_parsing_data.frame_cnt;
      // msg->body.light_control_msg.data.cmd.checksum =
      // uart_parsing_data.frame_checksum;
      break;
    }
  }
  return true;
}

bool ConstructStatusMessage(AgxMessage *msg) {
  if (msg == NULL) return false;

  switch (uart_parsing_data.frame_id) {
    case UART_MSG_SYSTEM_STATUS_ID: {
      msg->type = ScoutSystemStatusMsg;
      msg->body.system_status_msg.status.vehicle_state =
          uart_parsing_data.payload_buffer[0];
      msg->body.system_status_msg.status.control_mode =
          uart_parsing_data.payload_buffer[1];
      msg->body.system_status_msg.status.battery_voltage.high_byte =
          uart_parsing_data.payload_buffer[2];
      msg->body.system_status_msg.status.battery_voltage.low_byte =
          uart_parsing_data.payload_buffer[3];
      msg->body.system_status_msg.status.fault_code =
          uart_parsing_data.payload_buffer[4];
      msg->body.system_status_msg.status.reserved0 =
          uart_parsing_data.payload_buffer[5];
      msg->body.system_status_msg.status.reserved1 =
          uart_parsing_data.frame_cnt;
      msg->body.system_status_msg.status.count =
          uart_parsing_data.frame_checksum;
      break;
    }
    case UART_MSG_MOTION_STATUS_ID: {
      msg->type = ScoutMotionStatusMsg;
      msg->body.motion_status_msg.data.cmd.linear_velocity.high_byte =
          uart_parsing_data.payload_buffer[0];
      msg->body.motion_status_msg.data.cmd.linear_velocity.low_byte =
          uart_parsing_data.payload_buffer[1];
      msg->body.motion_status_msg.data.cmd.angular_velocity.high_byte =
          uart_parsing_data.payload_buffer[2];
      msg->body.motion_status_msg.data.cmd.angular_velocity.low_byte =
          uart_parsing_data.payload_buffer[3];
      msg->body.motion_status_msg.data.cmd.reserved0 = 0x00;
      msg->body.motion_status_msg.data.cmd.reserved1 = 0x00;
      msg->body.motion_status_msg.data.cmd.reserved2 = 0x00;
      msg->body.motion_status_msg.data.cmd.reserved3 = 0x00;
      break;
    }
    case UART_MSG_MOTOR1_DRIVER_STATUS_ID: {
      msg->type = ScoutMotorDriverStatusMsg;
      msg->body.motor_driver_status_msg.motor_id = SCOUT_MOTOR1_ID;
      msg->body.motor_driver_status_msg.data.status.current.high_byte =
          uart_parsing_data.payload_buffer[0];
      msg->body.motor_driver_status_msg.data.status.current.low_byte =
          uart_parsing_data.payload_buffer[1];
      msg->body.motor_driver_status_msg.data.status.rpm.high_byte =
          uart_parsing_data.payload_buffer[2];
      msg->body.motor_driver_status_msg.data.status.rpm.low_byte =
          uart_parsing_data.payload_buffer[3];
      msg->body.motor_driver_status_msg.data.status.temperature =
          uart_parsing_data.payload_buffer[4];
      msg->body.motor_driver_status_msg.data.status.reserved0 = 0x00;
      msg->body.motor_driver_status_msg.data.status.count =
          uart_parsing_data.frame_cnt;
      msg->body.motor_driver_status_msg.data.status.checksum =
          uart_parsing_data.frame_checksum;
      break;
    }
    case UART_MSG_MOTOR2_DRIVER_STATUS_ID: {
      msg->type = ScoutMotorDriverStatusMsg;
      msg->body.motor_driver_status_msg.motor_id = SCOUT_MOTOR2_ID;
      msg->body.motor_driver_status_msg.data.status.current.high_byte =
          uart_parsing_data.payload_buffer[0];
      msg->body.motor_driver_status_msg.data.status.current.low_byte =
          uart_parsing_data.payload_buffer[1];
      msg->body.motor_driver_status_msg.data.status.rpm.high_byte =
          uart_parsing_data.payload_buffer[2];
      msg->body.motor_driver_status_msg.data.status.rpm.low_byte =
          uart_parsing_data.payload_buffer[3];
      msg->body.motor_driver_status_msg.data.status.temperature =
          uart_parsing_data.payload_buffer[4];
      msg->body.motor_driver_status_msg.data.status.reserved0 = 0x00;
      msg->body.motor_driver_status_msg.data.status.count =
          uart_parsing_data.frame_cnt;
      msg->body.motor_driver_status_msg.data.status.checksum =
          uart_parsing_data.frame_checksum;
      break;
    }
    case UART_MSG_MOTOR3_DRIVER_STATUS_ID: {
      msg->type = ScoutMotorDriverStatusMsg;
      msg->body.motor_driver_status_msg.motor_id = SCOUT_MOTOR3_ID;
      msg->body.motor_driver_status_msg.data.status.current.high_byte =
          uart_parsing_data.payload_buffer[0];
      msg->body.motor_driver_status_msg.data.status.current.low_byte =
          uart_parsing_data.payload_buffer[1];
      msg->body.motor_driver_status_msg.data.status.rpm.high_byte =
          uart_parsing_data.payload_buffer[2];
      msg->body.motor_driver_status_msg.data.status.rpm.low_byte =
          uart_parsing_data.payload_buffer[3];
      msg->body.motor_driver_status_msg.data.status.temperature =
          uart_parsing_data.payload_buffer[4];
      msg->body.motor_driver_status_msg.data.status.reserved0 = 0x00;
      msg->body.motor_driver_status_msg.data.status.count =
          uart_parsing_data.frame_cnt;
      msg->body.motor_driver_status_msg.data.status.checksum =
          uart_parsing_data.frame_checksum;
      break;
    }
    case UART_MSG_MOTOR4_DRIVER_STATUS_ID: {
      msg->type = ScoutMotorDriverStatusMsg;
      msg->body.motor_driver_status_msg.motor_id = SCOUT_MOTOR4_ID;
      msg->body.motor_driver_status_msg.data.status.current.high_byte =
          uart_parsing_data.payload_buffer[0];
      msg->body.motor_driver_status_msg.data.status.current.low_byte =
          uart_parsing_data.payload_buffer[1];
      msg->body.motor_driver_status_msg.data.status.rpm.high_byte =
          uart_parsing_data.payload_buffer[2];
      msg->body.motor_driver_status_msg.data.status.rpm.low_byte =
          uart_parsing_data.payload_buffer[3];
      msg->body.motor_driver_status_msg.data.status.temperature =
          uart_parsing_data.payload_buffer[4];
      msg->body.motor_driver_status_msg.data.status.reserved0 = 0x00;
      msg->body.motor_driver_status_msg.data.status.count =
          uart_parsing_data.frame_cnt;
      msg->body.motor_driver_status_msg.data.status.checksum =
          uart_parsing_data.frame_checksum;
      break;
    }
    case UART_MSG_LIGHT_STATUS_ID: {
      msg->type = ScoutLightStatusMsg;
      msg->body.light_status_msg.data.status.light_ctrl_enable =
          uart_parsing_data.payload_buffer[0];
      msg->body.light_status_msg.data.status.front_light_mode =
          uart_parsing_data.payload_buffer[1];
      msg->body.light_status_msg.data.status.front_light_custom =
          uart_parsing_data.payload_buffer[2];
      msg->body.light_status_msg.data.status.rear_light_mode =
          uart_parsing_data.payload_buffer[3];
      msg->body.light_status_msg.data.status.rear_light_custom =
          uart_parsing_data.payload_buffer[4];
      msg->body.light_status_msg.data.status.reserved0 = 0x00;
      msg->body.light_status_msg.data.status.count =
          uart_parsing_data.frame_cnt;
      msg->body.light_status_msg.data.status.reserved1 = 0x00;
      break;
    }
  }
  return true;
}

uint8_t CalcUartDataChecksum(uint8_t *buf, uint8_t len) {
  uint8_t checksum = 0;

#ifdef USE_XOR_CHECKSUM
  for (int i = 0; i < len; ++i) checksum ^= buf[i];
#else
  for (int i = 0; i < len; ++i) checksum += buf[i];
#endif

  return checksum;
}

uint8_t CalcBufferedFrameChecksum() {
  uint8_t checksum = 0x00;

#ifdef USE_XOR_CHECKSUM
  checksum ^= UART_MSG_SOF1;
  checksum ^= UART_MSG_SOF2;
  checksum ^= uart_parsing_data.frame_len;
  checksum ^= uart_parsing_data.frame_type;
  checksum ^= uart_parsing_data.frame_id;
  for (size_t i = 0; i < uart_parsing_data.payload_data_pos; ++i)
    checksum ^= uart_parsing_data.payload_buffer[i];
  checksum ^= uart_parsing_data.frame_cnt;
#else
  checksum += UART_MSG_SOF1;
  checksum += UART_MSG_SOF2;
  checksum += uart_parsing_data.frame_len;
  checksum += uart_parsing_data.frame_type;
  checksum += uart_parsing_data.frame_id;
  for (size_t i = 0; i < uart_parsing_data.payload_data_pos; ++i)
    checksum += uart_parsing_data.payload_buffer[i];
  checksum += uart_parsing_data.frame_cnt;
#endif

  return checksum;
}
