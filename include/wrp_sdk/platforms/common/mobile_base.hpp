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

  void SetCmdTimeout(bool enable, uint32_t timeout_ms = 100);

  // connect to roboot from CAN or serial
  void Connect(std::string dev_name, int32_t baud_rate = 0);

  // disconnect from roboot, only valid for serial port
  void Disconnect();

  // cmd thread runs at 100Hz (10ms) by default
  void SetCmdThreadPeriodMs(int32_t period_ms) {
    cmd_thread_period_ms_ = period_ms;
  };

 protected:
  // communication interface
  bool can_connected_ = false;
  bool serial_connected_ = false;

  std::shared_ptr<AsyncCAN> can_if_;
  std::shared_ptr<AsyncSerial> serial_if_;

  // timeout to be implemented in each vehicle
  bool enable_timeout_ = true;
  uint32_t timeout_ms_ = 100;
  uint32_t watchdog_counter_ = 0;
  virtual void CheckCmdTimeout();

  // command thread
  std::thread cmd_thread_;
  int32_t cmd_thread_period_ms_ = 10;
  bool cmd_thread_started_ = false;

  // internal functions
  void ConfigureCAN(const std::string &can_if_name = "can0");
  void ConfigureSerial(const std::string uart_name = "/dev/ttyUSB0",
                       int32_t baud_rate = 115200);

  void StartCmdThread();
  void ControlLoop(int32_t period_ms);

  // functions that must/may be implemented by child classes
  virtual void SendRobotCmd() = 0;
  virtual void ParseCANFrame(can_frame *rx_frame){};
  virtual void ParseUARTBuffer(uint8_t *buf, const size_t bufsize,
                               size_t bytes_received){};
};
}  // namespace westonrobot

#endif /* MOBILE_BASE_HPP */
