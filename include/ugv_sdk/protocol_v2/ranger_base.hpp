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

#include "ugv_sdk/interface/ranger_interface.hpp"
#include "ugv_sdk/protocol_v2/agilex_base.hpp"

namespace westonrobot {
class RangerBase : public AgilexBase, public RangerInterface {
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
