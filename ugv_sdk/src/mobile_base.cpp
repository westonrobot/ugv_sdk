/*
 * mobile_base.cpp
 *
 * Created on: Jun 17, 2020 11:26
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "ugv_sdk/mobile_base.hpp"

#include <cstring>
#include <iostream>
#include <algorithm>

#include "stopwatch.hpp"

namespace westonrobot {
MobileBase::~MobileBase() {
  // release resource if occupied
  Disconnect();
  // joint cmd thread
  if (cmd_thread_.joinable()) cmd_thread_.join();
}

void MobileBase::Connect(std::string dev_name, int32_t baud_rate) {
  if (baud_rate == 0) {
    ConfigureCAN(dev_name);
  } else {
    ConfigureSerial(dev_name, baud_rate);
    if (!serial_connected_)
      std::cerr << "ERROR: Failed to connect to serial port" << std::endl;
  }
}

void MobileBase::Disconnect() {
  if (can_connected_) can_if_->StopService();
  if (serial_connected_ && serial_if_->IsOpened()) {
    serial_if_->StopService();
  }
}

void MobileBase::ConfigureCAN(const std::string &can_if_name) {
  can_if_ = std::make_shared<AsyncCAN>(can_if_name);
  can_if_->SetReceiveCallback(
      std::bind(&MobileBase::ParseCANFrame, this, std::placeholders::_1));
  can_if_->StartListening();
  can_connected_ = true;
}

void MobileBase::ConfigureSerial(const std::string uart_name,
                                 int32_t baud_rate) {
  serial_if_ = std::make_shared<AsyncSerial>(uart_name, baud_rate);
  serial_if_->SetReceiveCallback(
      std::bind(&MobileBase::ParseUARTBuffer, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  serial_if_->StartListening();
  if (serial_if_->IsOpened()) serial_connected_ = true;
}

void MobileBase::StartCmdThread() {
  cmd_thread_ = std::thread(
      std::bind(&MobileBase::ControlLoop, this, cmd_thread_period_ms_));
  cmd_thread_started_ = true;
}

void MobileBase::ControlLoop(int32_t period_ms) {
  Timer ctrl_timer;
  uint32_t timeout_iter_num;

  if (enable_timeout_) {
    if (timeout_ms_ < period_ms) timeout_ms_ = period_ms;
    timeout_iter_num = static_cast<uint32_t>(timeout_ms_ / period_ms);
    std::cout << "Timeout iteration number: " << timeout_iter_num << std::endl;
  }

  while (true) {
    ctrl_timer.reset();
    if (enable_timeout_) {
      if (watchdog_counter_ < timeout_iter_num) {
        SendRobotCmd();
        ++watchdog_counter_;
      } else {
        std::cout << "Warning: cmd timeout, old cmd not sent to robot" << std::endl;
      }
    } else {
      SendRobotCmd();
    }
    ctrl_timer.sleep_until_ms(period_ms);
  }
}
}  // namespace westonrobot
