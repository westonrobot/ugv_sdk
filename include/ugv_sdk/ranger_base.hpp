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

#include "ugv_sdk/agilex_base.hpp"

namespace westonrobot {
struct RangerState {
  // system state
  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  LightStateMessage light_state;



  RcStateMessage rc_state;

  ActuatorHSStateMessage actuator_hs_state[8];
  ActuatorLSStateMessage actuator_ls_state[8];
  MotionModeFeedbackMessage current_motion_mode;

  // sensor data
  OdometryMessage odometry;
};

struct RangerMotionCmd {
  double linear_velocity;
  double angular_velocity;
};

struct RangerLightCmd {
  RangerLightCmd() = default;
  RangerLightCmd(LightMode f_mode, uint8_t f_value, LightMode r_mode,
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

class RangerBase : public AgilexBase {
 public:
  RangerBase() : AgilexBase(){};
  ~RangerBase() = default;

  // set up connection
  void Connect(std::string dev_name) override;

  // robot control
  void SetMotionCommand(double linear_vel, double steer_angle,
                        double lateral_vel = 0.0, double angular_vel = 0.0);
  void SetLightCommand(const RangerLightCmd &cmd);
  void SetMotionMode(uint8_t mode);

  // get robot state
  RangerState GetRangerState();

 private:
  RangerState ranger_state_;

  void ParseCANFrame(can_frame *rx_frame) override;
  void UpdateRangerState(const AgxMessage &status_msg, RangerState &state);
};
}  // namespace westonrobot
#endif  // RANGER_BASE_HPP
