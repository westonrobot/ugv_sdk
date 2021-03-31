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
#include "ugv_sdk/proto/agx_msg_parser.h"
#include "ugv_sdk/scout/scout_types.hpp"

namespace westonrobot {
class ScoutBase : public MobileBase {
 public:
  ScoutBase() : MobileBase(){};
  ~ScoutBase() = default;

  // get robot state
  ScoutState GetScoutState();

  void EnableCommandedMode();

  // motion control
  void SetMotionCommand(double linear_vel, double angular_vel);

  // light control
  void SetLightCommand(const ScoutLightCmd &cmd);

 private:
  // cmd/status update related variables
  std::mutex scout_state_mutex_;
  std::mutex motion_cmd_mutex_;

  ScoutState scout_state_;
  ScoutMotionCmd current_motion_cmd_;

  // internal functions
  void SendMotionCmd(uint8_t count);
  void SendLightCmd(const ScoutLightCmd &cmd, uint8_t count);

  void SendRobotCmd() override;
  void ParseCANFrame(can_frame *rx_frame) override;

  void NewStatusMsgReceivedCallback(const AgxMessage &msg);
  static void UpdateScoutState(const AgxMessage &status_msg, ScoutState &state);
};
}  // namespace westonrobot

#endif /* SCOUT_BASE_HPP */
