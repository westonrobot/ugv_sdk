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

#include "ugv_sdk/details/interface/hunter_interface.hpp"
#include "ugv_sdk/details/robot_base/agilex_base.hpp"

namespace westonrobot {
template <typename ParserType>
class HunterBase : public AgilexBase<ParserType>, public HunterInterface {
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

  // get robot state
  HunterCoreState GetRobotState() override {
    auto state = AgilexBase<ParserType>::GetRobotCoreStateMsgGroup();

    HunterCoreState hunter_state;
    hunter_state.time_stamp = state.time_stamp;
    hunter_state.system_state = state.system_state;
    hunter_state.motion_state = state.motion_state;
    hunter_state.rc_state = state.rc_state;
    return hunter_state;
  }

  HunterActuatorState GetActuatorState() override {
    auto actuator = AgilexBase<ParserType>::GetActuatorStateMsgGroup();

    HunterActuatorState hunter_actuator;
    hunter_actuator.time_stamp = actuator.time_stamp;
    for (int i = 0; i < 3; ++i) {
      hunter_actuator.actuator_hs_state[i] = actuator.actuator_hs_state[i];
      hunter_actuator.actuator_ls_state[i] = actuator.actuator_ls_state[i];
      hunter_actuator.actuator_state[i] = actuator.actuator_state[i];
    }
    return hunter_actuator;
  }

  void ActivateBrake() override {
    AgilexBase<ParserType>::SetBrakeMode(BrakeMode::BRAKE_MODE_LOCK);
  }

  void ReleaseBrake() override {
    AgilexBase<ParserType>::SetBrakeMode(BrakeMode::BRAKE_MODE_UNLOCK);
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
