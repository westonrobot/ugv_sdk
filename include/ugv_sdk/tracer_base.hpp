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

#include "ugv_sdk/agilex_base.hpp"

namespace westonrobot {
struct TracerState {
  // system state
  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  LightStateMessage light_state;

  RcStateMessage rc_state;

  ActuatorHSStateMessage actuator_hs_state[2];
  ActuatorLSStateMessage actuator_ls_state[2];

  // sensor data
  OdometryMessage odometry;
};

struct TracerMotionCmd {
  double linear_velocity;
  double angular_velocity;
};

struct TracerLightCmd {
  TracerLightCmd() = default;
  TracerLightCmd(LightMode f_mode, uint8_t f_value)
      : cmd_ctrl_allowed(true),
        front_mode(f_mode),
        front_custom_value(f_value) {}

  bool cmd_ctrl_allowed = false;
  LightMode front_mode;
  uint8_t front_custom_value;
};

/////////////////////////////////////////////////////////////////////////

class TracerBase : public AgilexBase {
 public:
  TracerBase() : AgilexBase(){};
  ~TracerBase() = default;

  // set up connection
  void Connect(std::string dev_name) override;

  // robot control
  void SetMotionCommand(double linear_vel, double angular_vel);
  void SetLightCommand(const TracerLightCmd &cmd);

  // get robot state
  TracerState GetTracerState();

 private:
  TracerState tracer_state_;

  void ParseCANFrame(can_frame *rx_frame) override;
  void UpdateTracerState(const AgxMessage &status_msg, TracerState &state);
};
}  // namespace westonrobot

#endif /* TRACER_BASE_HPP */
