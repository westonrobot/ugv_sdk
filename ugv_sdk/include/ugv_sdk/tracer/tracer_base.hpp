/*
 * tracer_base.hpp
 *
 * Created on: Apr 14, 2020 10:21
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef TRACER_BASE_HPP
#define TRACER_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>

#include "ugv_sdk/mobile_base.hpp"
#include "ugv_sdk/proto/agx_msg_parser.h"
#include "ugv_sdk/tracer/tracer_types.hpp"

namespace westonrobot {
class TracerBase : public MobileBase {
 public:
  TracerBase() : MobileBase(){};
  ~TracerBase() = default;

  // get robot state
  TracerState GetTracerState();

  void EnableCommandedMode();

  // motion control
  void SetMotionCommand(double linear_vel, double angular_vel);

  // light control
  void SetLightCommand(const TracerLightCmd &cmd);

 private:
  // cmd/status update related variables
  std::mutex tracer_state_mutex_;
  std::mutex motion_cmd_mutex_;

  TracerState tracer_state_;
  TracerMotionCmd current_motion_cmd_;

  void SendMotionCmd(uint8_t count);
  void SendLightCmd(const TracerLightCmd &cmd, uint8_t count);

  void SendRobotCmd() override;
  void ParseCANFrame(can_frame *rx_frame) override;

  void NewStatusMsgReceivedCallback(const AgxMessage &msg);
  static void UpdateTracerState(const AgxMessage &status_msg,
                                TracerState &state);
};
}  // namespace westonrobot

#endif /* TRACER_BASE_HPP */
