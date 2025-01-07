/*
 * bunker_base.hpp
 *
 * Created on: Jul 14, 2021 23:05
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef BUNKER_BASE_HPP
#define BUNKER_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>

#include "ugv_sdk/details/interface/bunker_interface.hpp"
#include "ugv_sdk/details/robot_base/agilex_base.hpp"

namespace westonrobot {
template <typename ParserType>
class BunkerBase : public AgilexBase<ParserType>, public BunkerInterface {
 public:
  BunkerBase() : AgilexBase<ParserType>() {};
  ~BunkerBase() = default;

  // set up connection
  bool Connect(std::string can_name) override {
    return AgilexBase<ParserType>::Connect(can_name);
  }

  // robot control
  void SetMotionCommand(double linear_vel, double angular_vel) override {
    AgilexBase<ParserType>::SendMotionCommand(linear_vel, angular_vel, 0.0,
                                              0.0);
  }

  // get robot state
  BunkerCoreState GetRobotState() override {
    auto state = AgilexBase<ParserType>::GetRobotCoreStateMsgGroup();

    BunkerCoreState bunker_state;
    bunker_state.time_stamp = state.time_stamp;
    bunker_state.system_state = state.system_state;
    bunker_state.motion_state = state.motion_state;
    bunker_state.rc_state = state.rc_state;
    return bunker_state;
  }

  BunkerActuatorState GetActuatorState() override {
    auto actuator = AgilexBase<ParserType>::GetActuatorStateMsgGroup();

    BunkerActuatorState bunker_actuator;
    bunker_actuator.time_stamp = actuator.time_stamp;
    for (int i = 0; i < 2; ++i) {
      bunker_actuator.actuator_hs_state[i] = actuator.actuator_hs_state[i];
      bunker_actuator.actuator_ls_state[i] = actuator.actuator_ls_state[i];
      bunker_actuator.actuator_state[i] = actuator.actuator_state[i];
    }
    return bunker_actuator;
  }

  BunkerCommonSensorState GetCommonSensorState() override {
    return BunkerCommonSensorState();
  }
};
}  // namespace westonrobot

#include "ugv_sdk/details/protocol_v1/protocol_v1_parser.hpp"
#include "ugv_sdk/details/protocol_v2/protocol_v2_parser.hpp"

namespace westonrobot {
using BunkerBaseV1 = BunkerBase<BunkerProtocolV1Parser>;
using BunkerBaseV2 = BunkerBase<ProtocolV2Parser>;
using BunkerPro = BunkerBase<ProtocolV2Parser>;

class BunkerMini : public BunkerBase<ProtocolV2Parser> {
  BunkerCommonSensorState GetCommonSensorState() override {
    auto sensor = AgilexBase<ProtocolV2Parser>::GetCommonSensorStateMsgGroup();

    BunkerCommonSensorState bunker_sensor;
    bunker_sensor.time_stamp = sensor.time_stamp;
    bunker_sensor.bms_basic_state = sensor.bms_basic_state;
    return bunker_sensor;
  }
};
}  // namespace westonrobot

#endif /* BUNKER_BASE_HPP */
