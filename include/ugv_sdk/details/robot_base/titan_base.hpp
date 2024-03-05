/**
 * @Author    : Junyu Luo
 * @Date      : 2024-03-4
 * @FileName  : titan_base.hpp
 * @Mail      : wangzheqie@qq.com
 * Copyright  : AgileX Robotics
 **/

#ifndef TITAN_BASE_HPP
#define TITAN_BASE_HPP

#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "ugv_sdk/details/interface/titan_interface.hpp"
#include "ugv_sdk/details/robot_base/agilex_base.hpp"

#include "ugv_sdk/details/protocol_v2/protocol_v2_parser.hpp"

namespace westonrobot {
class TitanBase : public AgilexBase<ProtocolV2Parser>,
                     public TitanInterface {
 public:
  TitanBase() : AgilexBase<ProtocolV2Parser>(){};
  virtual ~TitanBase() = default;

  // set up connection
  bool Connect(std::string can_name) override {
    return AgilexBase<ProtocolV2Parser>::Connect(can_name);
  }

  // robot control
  void SetMotionCommand(double linear_vel, double steer_angle) override {
    AgilexBase<ProtocolV2Parser>::SendMotionCommand(linear_vel, 0.0,
                                                    0.0, steer_angle);
  }

  // get robot state
  TitanCoreState GetRobotState() override {
    auto state = AgilexBase<ProtocolV2Parser>::GetRobotCoreStateMsgGroup();

    TitanCoreState titan_state;
    titan_state.time_stamp = state.time_stamp;
    titan_state.system_state = state.system_state;
    titan_state.motion_state = state.motion_state;
    titan_state.rc_state = state.rc_state;
    titan_state.motion_mode_state = state.motion_mode_state;

    return titan_state;
  }

  TitanActuatorState GetActuatorState() override {
    auto actuator = AgilexBase<ProtocolV2Parser>::GetActuatorStateMsgGroup();

    TitanActuatorState titan_actuator;
    titan_actuator.time_stamp = actuator.time_stamp;
    for (int i = 0; i < 4; ++i) {
      titan_actuator.actuator_hs_state[i] = actuator.actuator_hs_state[i];
      titan_actuator.actuator_ls_state[i] = actuator.actuator_ls_state[i];
    }
    return titan_actuator;
  }

  TitanCommonSensorState GetCommonSensorState() override {
    auto common_sensor =
        AgilexBase<ProtocolV2Parser>::GetCommonSensorStateMsgGroup();

    TitanCommonSensorState titan_bms;

    titan_bms.time_stamp = common_sensor.time_stamp;

    titan_bms.bms_basic_state.current = common_sensor.bms_basic_state.current;
    // Note: BMS CAN message definition is not consistent across AgileX robots.
    // Robots with steering mechanism should additionally divide the voltage by
    // 10.
    titan_bms.bms_basic_state.voltage =
        common_sensor.bms_basic_state.voltage * 0.1f;
    titan_bms.bms_basic_state.battery_soc =
        common_sensor.bms_basic_state.battery_soc;
    titan_bms.bms_basic_state.battery_soh =
        common_sensor.bms_basic_state.battery_soh;
    titan_bms.bms_basic_state.temperature =
        common_sensor.bms_basic_state.temperature;

    return titan_bms;
  }
  void ActivateBrake() override {
    AgilexBase<ProtocolV2Parser>::SetBrakeMode(AgxBrakeMode::BRAKE_MODE_UNLOCK);
  }

  void ReleaseBrake() override {
    AgilexBase<ProtocolV2Parser>::SetBrakeMode(AgxBrakeMode::BRAKE_MODE_LOCK);
  }

};
}  // namespace westonrobot
#endif /* TITAN_BASE_HPP */
