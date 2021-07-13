/*
 * scout_robot.hpp
 *
 * Created on: Jul 08, 2021 10:59
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef SCOUT_ROBOT_HPP
#define SCOUT_ROBOT_HPP

#include <memory>

#include "ugv_sdk/details/interface/robot_interface.hpp"
#include "ugv_sdk/details/interface/scout_interface.hpp"

namespace westonrobot {
class ScoutRobot : public RobotInterface, public ScoutInterface {
 public:
  ScoutRobot(ProtocolVersion protocol = ProtocolVersion::AGX_V2,
             bool is_mini_model = false);
  ~ScoutRobot();

  void Connect(std::string can_name) override;
  void Connect(std::string uart_name, uint32_t baudrate) override;

  void EnableCommandedMode() override;

  void SetMotionCommand(double linear_vel, double angular_vel) override;
  void SetLightCommand(LightMode f_mode, uint8_t f_value, LightMode r_mode,
                       uint8_t r_value) override;
  void DisableLightControl() override;

  void ResetRobotState() override;

  ProtocolVersion GetProtocolVersion() override;

  // get robot state
  ScoutState GetRobotState() override;

 private:
  RobotInterface* robot_;
};
}  // namespace westonrobot

#endif /* SCOUT_ROBOT_HPP */
