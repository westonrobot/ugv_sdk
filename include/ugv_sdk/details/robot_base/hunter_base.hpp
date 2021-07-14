/*
 * hunter_base.hpp
 *
 * Created on: Jul 14, 2021 23:23
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef HUNTER_BASE_HPP
#define HUNTER_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>

#include "ugv_sdk/details/interface/scout_interface.hpp"
#include "ugv_sdk/details/robot_base/agilex_base.hpp"

namespace westonrobot {
template <typename ParserType>
class HunterBase : public AgilexBase<ParserType>, public ScoutInterface {
 public:
  HunterBase() : AgilexBase<ParserType>(){};
  ~HunterBase() = default;

  // set up connection
  void Connect(std::string can_name) override {
    AgilexBase<ParserType>::Connect(can_name);
  }

  // robot control
  void SetMotionCommand(double linear_vel, double angular_vel) override {
    AgilexBase<ParserType>::SendMotionCommand(linear_vel, 0.0, 0.0,
                                              angular_vel);
  }

  void SetLightCommand(LightMode f_mode, uint8_t f_value, LightMode r_mode,
                       uint8_t r_value) override {
    AgilexBase<ParserType>::SendLightCommand(f_mode, f_value, r_mode, r_value);
  }

  // get robot state
  ScoutCoreState GetRobotState() override {
    auto state = AgilexBase<ParserType>::GetRobotCoreStateMsgGroup();

    ScoutCoreState scout_state;
    scout_state.system_state = state.system_state;
    scout_state.motion_state = state.motion_state;
    scout_state.light_state = state.light_state;
    scout_state.rc_state = state.rc_state;
    return scout_state;
  }

  ScoutActuatorState GetActuatorState() override {
    auto actuator = AgilexBase<ParserType>::GetActuatorStateMsgGroup();

    ScoutActuatorState scout_actuator;
    for (int i = 0; i < 4; ++i) {
      scout_actuator.actuator_hs_state[i] = actuator.actuator_hs_state[i];
      scout_actuator.actuator_ls_state[i] = actuator.actuator_ls_state[i];
      scout_actuator.actuator_state[i] = actuator.actuator_state[i];
    }
    return scout_actuator;
  }
};
}  // namespace westonrobot

#include "ugv_sdk/details/protocol_v1/protocol_v1_parser.hpp"
#include "ugv_sdk/details/protocol_v2/protocol_v2_parser.hpp"

namespace westonrobot {
using HunterBaseV1 = HunterBase<HunterProtocolV1Parser>;
using HunterBaseV2 = HunterBase<ProtocolV2Parser>;
}  // namespace westonrobot

#endif /* HUNTER_BASE_HPP */
