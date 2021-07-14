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

#include "ugv_sdk/details/interface/tracer_interface.hpp"
#include "ugv_sdk/details/robot_base/agilex_base.hpp"

#include "ugv_sdk/details/protocol_v2/protocol_v2_parser.hpp"

namespace westonrobot {
class TracerBaseV2 : public AgilexBase<ProtocolV2Parser>,
                     public TracerInterface {
 public:
  TracerBaseV2() : AgilexBase<ProtocolV2Parser>(){};
  ~TracerBaseV2() = default;

  // set up connection
  void Connect(std::string can_name) override {
    AgilexBase<ProtocolV2Parser>::ConnectPort(
        can_name,
        std::bind(&TracerBaseV2::ParseCANFrame, this, std::placeholders::_1));
  }
  void Connect(std::string uart_name, uint32_t baudrate) override {
    // TODO
  }

  // robot control
  void SetMotionCommand(double linear_vel, double angular_vel) override {
    AgilexBase<ProtocolV2Parser>::SendMotionCommand(linear_vel, angular_vel,
                                                    0.0, 0.0);
  }

  void SetLightCommand(LightMode f_mode, uint8_t f_value) override {
    AgilexBase<ProtocolV2Parser>::SendLightCommand(f_mode, f_value, CONST_OFF,
                                                   0);
  }

  // get robot state
  TracerState GetRobotState() override {
    // std::lock_guard<std::mutex>
    // guard(AgilexBase<ProtocolV2Parser>::state_mutex_); return tracer_state_;
  }

  void ResetRobotState() override {
    // TODO
  }
};
}  // namespace westonrobot

#endif /* TRACER_BASE_HPP */
