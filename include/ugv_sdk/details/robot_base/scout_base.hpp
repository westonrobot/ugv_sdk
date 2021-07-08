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
template <typename Parser>
class ScoutBase : public AgilexBase<Parser>, public ScoutInterface {
 public:
  ScoutBase() : AgilexBase<Parser>(){};
  ~ScoutBase() = default;

  // set up connection
  void Connect(std::string can_name) override {
    AgilexBase<Parser>::ConnectPort(
        can_name, std::bind(&ScoutBase<Parser>::ParseCANFrame, this,
                            std::placeholders::_1));
  }

  void Connect(std::string uart_name, uint32_t baudrate) override {
    // TODO
  }

  // robot control
  void SetMotionCommand(double linear_vel, double angular_vel) override {
    AgilexBase<Parser>::SendMotionCommand(linear_vel, angular_vel, 0.0, 0.0);
  }

  void SetLightCommand(LightMode f_mode, uint8_t f_value, LightMode r_mode,
                       uint8_t r_value) override {
    AgilexBase<Parser>::SendLightCommand(f_mode, f_value, r_mode, r_value);
  }

  // get robot state
  ScoutState GetRobotState() override {
    std::lock_guard<std::mutex> guard(AgilexBase<Parser>::state_mutex_);
    return scout_state_;
  }

  void ResetRobotState() override {
    // TODO
  }

 private:
  ScoutState scout_state_;

  void ParseCANFrame(can_frame *rx_frame) override {
    AgxMessage status_msg;
    DecodeCanFrame(rx_frame, &status_msg);
    std::lock_guard<std::mutex> guard(AgilexBase<Parser>::state_mutex_);
    UpdateScoutState(status_msg, scout_state_);
  }

  void UpdateScoutState(const AgxMessage &status_msg, ScoutState &state) {
    switch (status_msg.type) {
      case AgxMsgSystemState: {
        //   std::cout << "system status feedback received" << std::endl;
        state.system_state = status_msg.body.system_state_msg;
        break;
      }
      case AgxMsgMotionState: {
        // std::cout << "motion control feedback received" << std::endl;
        state.motion_state = status_msg.body.motion_state_msg;
        break;
      }
      case AgxMsgLightState: {
        // std::cout << "light control feedback received" << std::endl;
        state.light_state = status_msg.body.light_state_msg;
        break;
      }
      case AgxMsgRcState: {
        state.rc_state = status_msg.body.rc_state_msg;
        break;
      }
      case AgxMsgActuatorHSState: {
        // std::cout << "actuator hs feedback received" << std::endl;
        state
            .actuator_hs_state[status_msg.body.actuator_hs_state_msg.motor_id] =
            status_msg.body.actuator_hs_state_msg;
        break;
      }
      case AgxMsgActuatorLSState: {
        // std::cout << "actuator ls feedback received" << std::endl;
        state
            .actuator_ls_state[status_msg.body.actuator_ls_state_msg.motor_id] =
            status_msg.body.actuator_ls_state_msg;
        break;
      }
      /* sensor feedback */
      case AgxMsgOdometry: {
        // std::cout << "Odometer msg feedback received" << std::endl;
        state.odometry = status_msg.body.odometry_msg;
      }
      default:
        break;
    }
  }
};
}  // namespace westonrobot

#include "ugv_sdk/details/protocol_v1/scout_protocol_v1_parser.hpp"
#include "ugv_sdk/details/protocol_v2/protocol_v2_parser.hpp"

namespace westonrobot {
using ScoutBaseV1 = ScoutBase<ScoutProtocolV1Parser>;
using ScoutBaseV2 = ScoutBase<ProtocolV2Parser>;
}  // namespace westonrobot

#endif /* SCOUT_BASE_HPP */
