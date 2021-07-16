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

#include "ugv_sdk/details/interface/scout_interface.hpp"
#include "ugv_sdk/details/robot_base/agilex_base.hpp"

namespace westonrobot {
template <typename ParserType>
class ScoutBase : public AgilexBase<ParserType>, public ScoutInterface {
 public:
  ScoutBase() : AgilexBase<ParserType>(){};
  ~ScoutBase() = default;

  // set up connection
  void Connect(std::string can_name) override {
    AgilexBase<ParserType>::Connect(can_name);
  }

  void Connect(std::string uart_name, uint32_t baudrate) override {
    // TODO
  }

  // robot control
  void SetMotionCommand(double linear_vel, double angular_vel) override {
    AgilexBase<ParserType>::SendMotionCommand(linear_vel, angular_vel, 0.0,
                                              0.0);
  }

  void SetLightCommand(LightMode f_mode, uint8_t f_value,
                       LightMode r_mode = LightMode::CONST_ON,
                       uint8_t r_value = 0) override {
    AgilexBase<ParserType>::SendLightCommand(f_mode, f_value, r_mode, r_value);
  }

  // get robot state
  ScoutCoreState GetRobotState() override {
    auto state = AgilexBase<ParserType>::GetRobotCoreStateMsgGroup();

    ScoutCoreState scout_state;
    scout_state.time_stamp = state.time_stamp;
    scout_state.system_state = state.system_state;
    scout_state.motion_state = state.motion_state;
    scout_state.light_state = state.light_state;
    scout_state.rc_state = state.rc_state;
    return scout_state;
  }

  ScoutActuatorState GetActuatorState() override {
    auto actuator = AgilexBase<ParserType>::GetActuatorStateMsgGroup();

    ScoutActuatorState scout_actuator;
    scout_actuator.time_stamp = actuator.time_stamp;
    for (int i = 0; i < 4; ++i) {
      scout_actuator.actuator_hs_state[i] = actuator.actuator_hs_state[i];
      scout_actuator.actuator_ls_state[i] = actuator.actuator_ls_state[i];
      scout_actuator.actuator_state[i] = actuator.actuator_state[i];
    }
    return scout_actuator;
  }
};

template <typename ParserType>
class ScoutMiniOmniBase : public ScoutBase<ParserType>,
                          public ScoutOmniInterface {
 public:
  void SetMotionCommand(double linear_vel, double angular_vel,
                        double lateral_velocity) override {
    AgilexBase<ParserType>::SendMotionCommand(linear_vel, angular_vel,
                                              lateral_velocity, 0.0);
  }

 private:
  using ScoutBase<ParserType>::SetMotionCommand;
};
}  // namespace westonrobot

#include "ugv_sdk/details/protocol_v1/protocol_v1_parser.hpp"
#include "ugv_sdk/details/protocol_v2/protocol_v2_parser.hpp"

namespace westonrobot {
using ScoutBaseV1 = ScoutBase<ScoutProtocolV1Parser>;
using ScoutMiniBaseV1 = ScoutBase<ScoutMiniProtocolV1Parser>;
using ScoutMiniOmniBaseV1 = ScoutMiniOmniBase<ScoutMiniProtocolV1Parser>;

using ScoutBaseV2 = ScoutBase<ProtocolV2Parser>;
using ScoutMiniBaseV2 = ScoutBase<ProtocolV2Parser>;
using ScoutMiniOmniBaseV2 = ScoutMiniOmniBase<ProtocolV2Parser>;
}  // namespace westonrobot

#endif /* SCOUT_BASE_HPP */
