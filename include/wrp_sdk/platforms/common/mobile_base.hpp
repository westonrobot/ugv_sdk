/*
 * mobile_base.hpp
 *
 * Created on: Jun 17, 2020 11:23
 * Description:
 *
 * Generic mobile base: this class handles the communication
 * logic that is similar across different mobile platforms
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef MOBILE_BASE_HPP
#define MOBILE_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <functional>

#include "wrp_sdk/asyncio/async_can.hpp"
#include "wrp_sdk/asyncio/async_serial.hpp"

namespace westonrobot {
class MobileBase {
 public:
  MobileBase() = default;
  virtual ~MobileBase();

  // do not allow copy or assignment
  MobileBase(const MobileBase &hunter) = delete;
  MobileBase &operator=(const MobileBase &hunter) = delete;

  // connect to roboot from CAN or serial
  void Connect(std::string dev_name, int32_t baud_rate = 0);

  // disconnect from roboot, only valid for serial port
  void Disconnect();

  // cmd thread runs at 100Hz (10ms) by default
  void SetCmdThreadPeriodMs(int32_t period_ms) {
    cmd_thread_period_ms_ = period_ms;
  };

  // motion control
  void SetMotionCommand(double linear_vel, double steering_angle,
                        HunterMotionCmd::FaultClearFlag fault_clr_flag =
                            HunterMotionCmd::FaultClearFlag::NO_FAULT);

  // get robot state
  HunterState GetHunterState();

 protected:
  // hardware communication interface
  std::shared_ptr<ASyncCAN> can_if_;
  std::shared_ptr<ASyncSerial> serial_if_;

  // CAN priority higher than serial if both connected
  bool can_connected_ = false;
  bool serial_connected_ = false;

  // serial port related variables
  uint8_t tx_cmd_len_;
  uint8_t tx_buffer_[HUNTER_CMD_BUF_LEN];

  // cmd/status update related variables
  std::thread cmd_thread_;
  std::mutex hunter_state_mutex_;
  std::mutex motion_cmd_mutex_;

  HunterState hunter_state_;
  HunterMotionCmd current_motion_cmd_;

  int32_t cmd_thread_period_ms_ = 10;
  bool cmd_thread_started_ = false;

  // internal functions
  void ConfigureCANBus(const std::string &can_if_name = "can1");
  void ConfigureSerial(const std::string uart_name = "/dev/ttyUSB0",
                       int32_t baud_rate = 115200);

  void StartCmdThread();
  void ControlLoop(int32_t period_ms);

  void SendMotionCmd(uint8_t count);

  void ParseCANFrame(can_frame *rx_frame);
  void ParseUARTBuffer(uint8_t *buf, const size_t bufsize,
                       size_t bytes_received);

  void NewStatusMsgReceivedCallback(const HunterMessage &msg);
};
}  // namespace westonrobot

#endif /* MOBILE_BASE_HPP */
