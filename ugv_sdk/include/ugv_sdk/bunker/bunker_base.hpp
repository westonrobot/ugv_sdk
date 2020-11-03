/*
 * bunker_base.hpp
 *
 * Created on: Jun 04, 2019 01:22
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef BUNKER_BASE_HPP
#define BUNKER_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <functional>

#include "ugv_sdk/mobile_base.hpp"

#include "ugv_sdk/bunker/bunker_protocol.h"
#include "ugv_sdk/bunker/bunker_can_parser.h"
#include "ugv_sdk/bunker/bunker_types.hpp"

namespace westonrobot {
class BunkerBase : public MobileBase {
 public:
  BunkerBase() : MobileBase(){};
  ~BunkerBase() = default;

 public:
  // motion control
  void SetMotionCommand(double linear_vel, double angular_vel,
                        BunkerMotionCmd::FaultClearFlag fault_clr_flag =
                            BunkerMotionCmd::FaultClearFlag::NO_FAULT);

  // get robot state
  BunkerState GetBunkerState();

 private:


  // cmd/status update related variables
  std::mutex bunker_state_mutex_;
  std::mutex motion_cmd_mutex_;

  BunkerState bunker_state_;
  BunkerMotionCmd current_motion_cmd_;



  // internal functions
  void SendRobotCmd() override;
  void ParseCANFrame(can_frame *rx_frame) override;
  void ParseUARTBuffer(uint8_t *buf, const size_t bufsize,
                       size_t bytes_received) override{};

  void SendMotionCmd(uint8_t count);
  void NewStatusMsgReceivedCallback(const BunkerMessage &msg);

 public:
  static void UpdateBunkerState(const BunkerMessage &status_msg,
                               BunkerState &state);
};
}  // namespace westonrobot

#endif /* BUNKER_BASE_HPP */
