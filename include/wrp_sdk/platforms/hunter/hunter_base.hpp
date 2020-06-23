/*
 * hunter_base.hpp
 *
 * Created on: Apr 01, 2020 09:43
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef HUNTER_BASE_HPP
#define HUNTER_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>

#include "wrp_sdk/platforms/common/mobile_base.hpp"

#include "wrp_sdk/platforms/hunter/hunter_protocol.h"
#include "wrp_sdk/platforms/hunter/hunter_can_parser.h"
#include "wrp_sdk/platforms/hunter/hunter_types.hpp"

namespace westonrobot {
class HunterBase : public MobileBase {
 public:
  HunterBase() : MobileBase(){};
  ~HunterBase() = default;

  // get robot state
  HunterState GetHunterState();

  // motion control
  void SetMotionCommand(double linear_vel, double steering_angle,
                        HunterMotionCmd::FaultClearFlag fault_clr_flag =
                            HunterMotionCmd::FaultClearFlag::NO_FAULT);

 private:
  // serial port related variables
  uint8_t tx_cmd_len_;
  uint8_t tx_buffer_[HUNTER_CMD_BUF_LEN];

  // cmd/status update related variables
  std::mutex hunter_state_mutex_;
  std::mutex motion_cmd_mutex_;

  HunterState hunter_state_;
  HunterMotionCmd current_motion_cmd_;

  // internal functions
  void ConfigureCANBus(const std::string &can_if_name = "can1");
  void ConfigureSerial(const std::string uart_name = "/dev/ttyUSB0",
                       int32_t baud_rate = 115200);

  void SendRobotCmd() override;
  void ParseCANFrame(can_frame *rx_frame) override;
  void ParseUARTBuffer(uint8_t *buf, const size_t bufsize,
                       size_t bytes_received) override{};

  void SendMotionCmd(uint8_t count);
  void NewStatusMsgReceivedCallback(const HunterMessage &msg);

 public:
  static void UpdateHunterState(const HunterMessage &status_msg,
                                HunterState &state);
};
}  // namespace westonrobot

#endif /* HUNTER_BASE_HPP */
