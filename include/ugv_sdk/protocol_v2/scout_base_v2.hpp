/*
 * scout_base.hpp
 *
 * Created on: Dec 23, 2020 14:39
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef SCOUT_BASE_HPP
#define SCOUT_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>

#include "ugv_sdk/interface/scout_interface.hpp"

#include "ugv_sdk/mobile_base/agilex_base.hpp"
#include "ugv_sdk/protocol_v2/protocol_v2_parser.hpp"

namespace westonrobot {
class ScoutBase : public AgilexBase<ProtocolV2Parser>, public ScoutInterface {
 public:
  ScoutBase() : AgilexBase(){};
  ~ScoutBase() = default;

  // set up connection
  void Connect(std::string can_name) override;
  void Connect(std::string uart_name, uint32_t baudrate) override;

  // robot control
  void SetMotionCommand(double linear_vel, double angular_vel) override;
  void SetLightCommand(LightMode f_mode, uint8_t f_value, LightMode r_mode,
                       uint8_t r_value) override;

  // get robot state
  ScoutState GetRobotState() override;

  void ResetRobotState() override {}

 private:
  ScoutState scout_state_;

  void ParseCANFrame(can_frame *rx_frame) override;
  void UpdateScoutState(const AgxMessage &status_msg, ScoutState &state);
};
}  // namespace westonrobot

#endif /* SCOUT_BASE_HPP */
