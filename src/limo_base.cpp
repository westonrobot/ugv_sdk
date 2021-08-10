#include "ugv_sdk/limo_base.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <ratio>
#include <string>
#include <thread>

#include "stopwatch.hpp"

#include "ugv_sdk/details/agilex_msg_parser.h"

using namespace std::placeholders;

namespace westonrobot {
void LimoBase::Connect(std::string dev_name, uint32_t bouadrate) {
  AgilexBaseSerialPort::Connect(
      dev_name, std::bind(&LimoBase::ParseSerialFrame, this, ::_1, ::_2, ::_3),
      bouadrate);
}

void LimoBase::SetMotionCommand(double linear_vel, double steer_angle,
                                 double lateral_vel, double angular_vel) {
  AgilexBaseSerialPort::SetMotionCommand(linear_vel, angular_vel, lateral_vel,
                                         steer_angle);
}

void LimoBase::SetLightCommand(const LimoLightCmd &cmd) {
  if (cmd.cmd_ctrl_allowed) {
    AgilexBaseSerialPort::SendLightCommand(
        cmd.front_mode, cmd.front_custom_value, LightMode::CONST_OFF, 0);
  }
}

void LimoBase::SetMotionMode(uint8_t mode) {
  AgilexBaseSerialPort::SetMotionMode(mode);
}

LimoState LimoBase::GetLimoState() {
  std::lock_guard<std::mutex> guard(state_mutex_);
  return limo_state_;
}

void LimoBase::ParseCANFrame(can_frame *rx_frame) {
  AgxMessage status_msg;
  DecodeCanFrame(rx_frame, &status_msg);
  std::lock_guard<std::mutex> guard(state_mutex_);
  UpdateLimoState(status_msg, limo_state_);
}

void LimoBase::UpdateLimoState(const AgxMessage &status_msg,
                                 LimoState &state) {
  switch (status_msg.type) {
    case AgxMsgSystemState: {
      state.system_state = status_msg.body.system_state_msg;
      break;
    }
    case AgxMsgMotionState: {
      state.motion_state = status_msg.body.motion_state_msg;
      // std::cout << "motion state in system state: " <<
      // state.system_state.motion_mode << std::endl;
      break;
    }
    case AgxMsgLightState: {
      state.light_state = status_msg.body.light_state_msg;
      break;
    }
    case AgxMsgRcState: {
      state.rc_state = status_msg.body.rc_state_msg;
      break;
    }
    case AgxMsgActuatorHSState: {
      state.actuator_hs_state[status_msg.body.actuator_hs_state_msg.motor_id] =
          status_msg.body.actuator_hs_state_msg;
      break;
    }
    case AgxMsgActuatorLSState: {
      // std::cout << "actuator ls feedback received" << std::endl;
      state.actuator_ls_state[status_msg.body.actuator_ls_state_msg.motor_id] =
          status_msg.body.actuator_ls_state_msg;
      break;
    }
    case AgxMsgMotionModeState: {
      state.current_motion_mode = status_msg.body.motion_mode_feedback_msg;
      // std::cout << "mode feedback : " <<
      // state.current_motion_mode.motion_mode << std::endl; std::cout <<
      // "changing feedback : " << state.current_motion_mode.mode_changing <<
      // std::endl;
      break;
    }
    /* sensor feedback */
    case AgxMsgOdometry: {
      // std::cout << "Odometer msg feedback received: " <<
      // state.odometry.left_wheel <<" , " << state.odometry.right_wheel <<
      // std::endl;
      state.odometry = status_msg.body.odometry_msg;
      break;
    }
    case AgxMsgImuAccel: {
      state.imu_accel_ = status_msg.body.imu_accel_msg;
      break;
    }
    case AgxMsgImuGyro: {
      state.imu_gyro_ = status_msg.body.imu_gyro_msg;
      break;
    }
    case AgxMsgImuEuler: {
      state.imu_euler_ = status_msg.body.imu_euler_msg;
      break;
    }
    default:
      break;
  }
}
void LimoBase::ParseSerialFrame(uint8_t *data, const size_t bufsize,
                                 size_t len) {
  std::lock_guard<std::mutex> lock(serial_callback_mutex_);
  if (serial_raw_data_.size() >= (MAX_SERIAL_BUFFER_SIZE - len)) {
    std::queue<uint8_t> empty;
    std::swap(serial_raw_data_, empty);
  }

  for (size_t i = 0; i < len; i++) {
    serial_raw_data_.push(data[i]);
  }

  while (serial_raw_data_.size() > 0 && serial_raw_data_.front() != 0x55) {
    serial_raw_data_.pop();
  }

  if (serial_raw_data_.size() < 0x0e - 1) {
    return;
  }

  serial_raw_data_.pop();  // 0x55
  if (serial_raw_data_.front() == 0x0e) {
    serial_raw_data_.pop();  // 0x0e
    can_frame frame;
    uint16_t high = serial_raw_data_.front() << 8;
    serial_raw_data_.pop();  // hight 8 bit
    uint16_t low = serial_raw_data_.front();
    serial_raw_data_.pop();  // low 8 bit

    frame.can_id = high + low;
    frame.can_dlc = 0x0e;

    for (size_t j = 0; j < 8; j++) {
      frame.data[j] = serial_raw_data_.front();
      serial_raw_data_.pop();
    }
    uint8_t frame_id = serial_raw_data_.front();
    serial_raw_data_.pop();
    uint8_t crc = serial_raw_data_.front();
    serial_raw_data_.pop();

    // get a can frame, decode it
    ParseCANFrame(&frame);
  } else {
    serial_raw_data_.pop();
  }
}
}  // namespace westonrobot
