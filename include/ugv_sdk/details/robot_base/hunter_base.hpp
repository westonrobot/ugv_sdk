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
  bool Connect(std::string can_name) override {
    return AgilexBase<ParserType>::Connect(can_name);
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

  HunterCommonSensorState GetCommonSensorState() override {
    auto common_sensor =
        AgilexBase<ParserType>::GetCommonSensorStateMsgGroup();

    HunterCommonSensorState hunter_bms;

    hunter_bms.time_stamp = common_sensor.time_stamp;

    hunter_bms.bms_basic_state.current = common_sensor.bms_basic_state.current;
    // Note: BMS CAN message definition is not consistent across AgileX robots.
    // Robots with steering mechanism should additionally divide the voltage by
    // 10.
    hunter_bms.bms_basic_state.voltage =
        common_sensor.bms_basic_state.voltage * 0.1f;
    hunter_bms.bms_basic_state.battery_soc =
        common_sensor.bms_basic_state.battery_soc;
    hunter_bms.bms_basic_state.battery_soh =
        common_sensor.bms_basic_state.battery_soh;
    hunter_bms.bms_basic_state.temperature =
        common_sensor.bms_basic_state.temperature;
    hunter_bms.bms_extend_state.alarm_status_1 =
        common_sensor.bms_extend_state.alarm_status_1;
    hunter_bms.bms_extend_state.alarm_status_2 =
        common_sensor.bms_extend_state.alarm_status_2;
    hunter_bms.bms_extend_state.warn_status_1 =
        common_sensor.bms_extend_state.warn_status_1;
    hunter_bms.bms_extend_state.warn_status_2 =
        common_sensor.bms_extend_state.warn_status_2;

    return hunter_bms;
  }
  void ActivateBrake() override {
    AgilexBase<ParserType>::SetBrakeMode(AgxBrakeMode::BRAKE_MODE_LOCK);
  }

  void ReleaseBrake() override {
    AgilexBase<ParserType>::SetBrakeMode(AgxBrakeMode::BRAKE_MODE_UNLOCK);
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
