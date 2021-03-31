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

#include "ugv_sdk/agilex_base.hpp"

namespace westonrobot {
struct ScoutState {
  // system state
  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  LightStateMessage light_state;

  RcStateMessage rc_state;

  ActuatorHSStateMessage actuator_hs_state[4];
  ActuatorLSStateMessage actuator_ls_state[4];

  // sensor data
  OdometryMessage odometry;
};

struct ScoutMotionCmd {
  double linear_velocity;
  double angular_velocity;
};

struct ScoutLightCmd {
  ScoutLightCmd() = default;
  ScoutLightCmd(LightMode f_mode, uint8_t f_value, LightMode r_mode,
                uint8_t r_value)
      : cmd_ctrl_allowed(true),
        front_mode(f_mode),
        front_custom_value(f_value),
        rear_mode(r_mode),
        rear_custom_value(r_value) {}

  bool cmd_ctrl_allowed = false;
  LightMode front_mode;
  uint8_t front_custom_value;
  LightMode rear_mode;
  uint8_t rear_custom_value;
};

/////////////////////////////////////////////////////////////////////////

class ScoutBase : public AgilexBase {
 public:
  ScoutBase() : AgilexBase(){};
  ~ScoutBase() = default;

  // set up connection
  void Connect(std::string dev_name) override;

  // robot control
  void SetMotionCommand(double linear_vel, double angular_vel);
  void SetLightCommand(const ScoutLightCmd &cmd);

  // get robot state
  ScoutState GetScoutState();

 private:
  ScoutState scout_state_;

  void ParseCANFrame(can_frame *rx_frame) override;
  void UpdateScoutState(const AgxMessage &status_msg, ScoutState &state);
};
}  // namespace westonrobot

#endif /* SCOUT_BASE_HPP */
