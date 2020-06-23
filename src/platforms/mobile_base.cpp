/*
 * mobile_base.cpp
 *
 * Created on: Jun 17, 2020 11:26
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "wrp_sdk/platforms/common/mobile_base.hpp"

#include <cstring>
#include <iostream>
#include <algorithm>

#include "stopwatch.h"

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
  if (serial_connected_ && serial_if_->is_open()) {
    serial_if_->close();
  }
}

void MobileBase::ConfigureCAN(const std::string &can_if_name) {
  can_if_ = std::make_shared<ASyncCAN>(can_if_name);
  can_if_->set_receive_callback(
      std::bind(&MobileBase::ParseCANFrame, this, std::placeholders::_1));
  can_connected_ = true;
}

void MobileBase::ConfigureSerial(const std::string uart_name,
                                 int32_t baud_rate) {
  serial_if_ = std::make_shared<ASyncSerial>(uart_name, baud_rate);
  serial_if_->open();
  if (serial_if_->is_open()) serial_connected_ = true;
  serial_if_->set_receive_callback(
      std::bind(&MobileBase::ParseUARTBuffer, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
}

void MobileBase::StartCmdThread() {
  cmd_thread_ = std::thread(
      std::bind(&MobileBase::ControlLoop, this, cmd_thread_period_ms_));
  cmd_thread_started_ = true;
}

void MobileBase::ControlLoop(int32_t period_ms) {
  StopWatch ctrl_sw;
  bool print_loop_freq = false;
  while (true) {
    ctrl_sw.tic();
    SendRobotCmd();
    ctrl_sw.sleep_until_ms(period_ms);
    if (print_loop_freq)
      std::cout << "control loop frequency: " << 1.0 / ctrl_sw.toc()
                << std::endl;
  }
}
}  // namespace westonrobot
