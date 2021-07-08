/*
 * tracer_base.hpp
 *
 * Created on: Apr 14, 2020 10:21
 * Description:
 *
 * Copyright (c) 2020 Weston Robot Pte. Ltd.
 */

#ifndef TRACER_BASE_HPP
#define TRACER_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>

#include "ugv_sdk/interface/tracer_interface.hpp"
#include "ugv_sdk/protocol_v2/agilex_base.hpp"

namespace westonrobot {
class TracerBase : public AgilexBase, public TracerInterface {
 public:
  TracerBase() : AgilexBase(){};
  ~TracerBase() = default;

  // set up connection
  void Connect(std::string can_name) override;
  void Connect(std::string uart_name, uint32_t baudrate) override;

  // robot control
  void SetMotionCommand(double linear_vel, double angular_vel);
  void SetLightCommand(const TracerLightCmd &cmd);

  // get robot state
  TracerState GetTracerState();

 private:
  TracerState tracer_state_;

  void ParseCANFrame(can_frame *rx_frame) override;
  void UpdateTracerState(const AgxMessage &status_msg, TracerState &state);
};
}  // namespace westonrobot

#endif /* TRACER_BASE_HPP */
