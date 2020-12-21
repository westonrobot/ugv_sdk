/*
 * scout_base.hpp
 *
 * Created on: Jun 04, 2019 01:22
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_BASE_HPP
#define SCOUT_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <functional>

#include "ugv_sdk/mobile_base.hpp"

#include "ugv_sdk/scout/scout_protocol.h"
#include "ugv_sdk/scout/scout_can_parser.h"
#include "ugv_sdk/scout/scout_uart_parser.h"
#include "ugv_sdk/scout/scout_types.hpp"

namespace westonrobot {
class ScoutBase : public MobileBase {
 public:
  ScoutBase(bool is_scout_mini = false)
      : MobileBase(), is_scout_mini_(is_scout_mini){};
  ~ScoutBase() = default;

 public:
  // motion control
  void SetMotionCommand(double linear_vel, double angular_vel,
                        ScoutMotionCmd::FaultClearFlag fault_clr_flag =
                            ScoutMotionCmd::FaultClearFlag::NO_FAULT);

  // light control
  void SetLightCommand(ScoutLightCmd cmd);
  void DisableLightCmdControl();

  // get robot state
  ScoutState GetScoutState();

 private:
  bool is_scout_mini_ = false;

  // serial port buffer
  uint8_t tx_cmd_len_;
  uint8_t tx_buffer_[SCOUT_CMD_BUF_LEN];

  // cmd/status update related variables
  std::mutex scout_state_mutex_;
  std::mutex motion_cmd_mutex_;
  std::mutex light_cmd_mutex_;

  ScoutState scout_state_;
  ScoutMotionCmd current_motion_cmd_;
  ScoutLightCmd current_light_cmd_;

  bool light_ctrl_enabled_ = false;
  bool light_ctrl_requested_ = false;

  // internal functions
  void SendRobotCmd() override;
  void ParseCANFrame(can_frame *rx_frame);
  void ParseUARTBuffer(uint8_t *buf, const size_t bufsize,
                       size_t bytes_received);

  void SendMotionCmd(uint8_t count);
  void SendLightCmd(uint8_t count);
  void NewStatusMsgReceivedCallback(const ScoutMessage &msg);

 public:
  static void UpdateScoutState(const ScoutMessage &status_msg,
                               ScoutState &state);
};
}  // namespace westonrobot

#endif /* SCOUT_BASE_HPP */
