/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-04-19  19:12:52
 * @FileName  : ranger_base.hpp
 * @Mail      : wangzheqie@qq.com
 * Copyright  : AgileX Robotics
 **/

#ifndef RANGER_BASE_HPP
#define RANGER_BASE_HPP

#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "ugv_sdk/details/interface/ranger_interface.hpp"
#include "ugv_sdk/details/robot_base/agilex_base.hpp"

#include "ugv_sdk/details/protocol_v2/protocol_v2_parser.hpp"

namespace westonrobot {
class RangerBase : public AgilexBase<ProtocolV2Parser>, public RangerInterface {
 public:
  RangerBase() : AgilexBase<ProtocolV2Parser>(){};
  ~RangerBase() = default;

  // set up connection
  bool Connect(std::string dev_name) override {
    return AgilexBase::ConnectPort(
        dev_name,
        std::bind(&RangerBase::ParseCANFrame, this, std::placeholders::_1));
  }

  // robot control
  void SetMotionCommand(double linear_vel, double steer_angle,
                        double lateral_vel = 0.0,
                        double angular_vel = 0.0) override {
    AgilexBase<ProtocolV2Parser>::SendMotionCommand(
        linear_vel, angular_vel, lateral_vel, steer_angle / 10.0);
  }

  void SetLightCommand(AgxLightMode f_mode, uint8_t f_value, AgxLightMode r_mode,
                       uint8_t r_value) override {
    AgilexBase<ProtocolV2Parser>::SendLightCommand(f_mode, f_value, r_mode,
                                                   r_value);
  }

  void SetMotionMode(uint8_t mode) { AgilexBase::SetMotionMode(mode); }

  // get robot state
  RangerCoreState GetRobotState() override {
    auto state = AgilexBase<ProtocolV2Parser>::GetRobotCoreStateMsgGroup();

    RangerCoreState ranger_state;
    ranger_state.time_stamp = state.time_stamp;
    ranger_state.system_state = state.system_state;
    ranger_state.motion_state = state.motion_state;
    ranger_state.light_state = state.light_state;
    ranger_state.rc_state = state.rc_state;
    ranger_state.current_motion_mode = state.motion_mode_state;
    return ranger_state;
  }

  RangerActuatorState GetActuatorState() override {
    auto actuator = AgilexBase<ProtocolV2Parser>::GetActuatorStateMsgGroup();

    RangerActuatorState ranger_actuator;
    ranger_actuator.time_stamp = actuator.time_stamp;
    for (int i = 0; i < 8; ++i) {
      ranger_actuator.actuator_hs_state[i] = actuator.actuator_hs_state[i];
      ranger_actuator.actuator_ls_state[i] = actuator.actuator_ls_state[i];
    }
    return ranger_actuator;
  }

  RangerMotorState GetMotorState() override {
    auto motor = AgilexBase<ProtocolV2Parser>::GetMotorMsgGroup();

    RangerMotorState ranger_motor;
    ranger_motor.motor_speed_state.speed_1 = motor.MoterSpeed.speed_1;
    ranger_motor.motor_speed_state.speed_2 = motor.MoterSpeed.speed_2;
    ranger_motor.motor_speed_state.speed_3 = motor.MoterSpeed.speed_3;
    ranger_motor.motor_speed_state.speed_4 = motor.MoterSpeed.speed_4;
    ranger_motor.motor_angle_state.angle_5 = motor.MoterAngle.angle_5;
    ranger_motor.motor_angle_state.angle_6 = motor.MoterAngle.angle_6;
    ranger_motor.motor_angle_state.angle_7 = motor.MoterAngle.angle_7;
    ranger_motor.motor_angle_state.angle_8 = motor.MoterAngle.angle_8;

    return ranger_motor;
  }

  RangerBmsState GetBmsState() override{
    auto motor = AgilexBase<ProtocolV2Parser>::GetCommonSensorStateMsgGroup();

    RangerBmsState ranger_bms;
    ranger_bms.bmsbasic.current = motor.bms_basic_state.current;
    ranger_bms.bmsbasic.voltage = motor.bms_basic_state.voltage;
    ranger_bms.bmsbasic.battery_soc = motor.bms_basic_state.battery_soc;
    ranger_bms.bmsbasic.battery_soh = motor.bms_basic_state.battery_soh;
    ranger_bms.bmsbasic.temperature = motor.bms_basic_state.temperature;

    return ranger_bms;
  }


};

using RangerBaseV2 = RangerBase;
}  // namespace westonrobot
#endif  // RANGER_BASE_HPP
