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
class TracerBaseV2 : public AgilexBase<ProtocolV2Parser>, public TracerInterface {
 public:
  TracerBaseV2() : AgilexBase<ProtocolV2Parser>(){};
  ~TracerBaseV2() = default;

  // set up connection
  void Connect(std::string can_name) override {
    AgilexBase<ProtocolV2Parser>::ConnectPort(
        can_name, std::bind(&TracerBaseV2::ParseCANFrame, this,
                            std::placeholders::_1));
  }
  void Connect(std::string uart_name, uint32_t baudrate) override {
    // TODO
  }

  // robot control
  void SetMotionCommand(double linear_vel, double angular_vel) override {
    AgilexBase<ProtocolV2Parser>::SendMotionCommand(linear_vel, angular_vel, 0.0, 0.0);
  }

  void SetLightCommand(LightMode f_mode, uint8_t f_value) override {
    AgilexBase<ProtocolV2Parser>::SendLightCommand(f_mode, f_value, CONST_OFF, 0);
  }

  // get robot state
  TracerState GetRobotState() override {
    std::lock_guard<std::mutex> guard(AgilexBase<ProtocolV2Parser>::state_mutex_);
    return tracer_state_;
  }

  void ResetRobotState() override {
    // TODO
  }
  
 private:
  TracerState tracer_state_;

  void ParseCANFrame(can_frame *rx_frame) override {
    AgxMessage status_msg;
    if (AgilexBase<ProtocolV2Parser>::parser_.DecodeMessage(rx_frame, &status_msg)) {
      UpdateTracerState(status_msg, tracer_state_);
    }
  }
  
  void UpdateTracerState(const AgxMessage &status_msg, TracerState &state) {
    std::lock_guard<std::mutex> guard(AgilexBase<ProtocolV2Parser>::state_mutex_);  
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
        //   case AgxMsgRcState: {
        //     state.rc_state = status_msg.body.rc_state_msg;
        //     break;
        //   }
        /* sensor feedback */
        //   case AgxMsgOdometry: {
        //     // std::cout << "Odometer msg feedback received" << std::endl;
        //     state.odometry = status_msg.body.odometry_msg;
        //   }
      default:
        break;
    }
  }
};
}  // namespace westonrobot

#endif /* TRACER_BASE_HPP */
