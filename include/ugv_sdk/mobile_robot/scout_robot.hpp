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

#include "ugv_sdk/details/interface/robot_common_interface.hpp"
#include "ugv_sdk/details/interface/scout_interface.hpp"

namespace westonrobot {
class ScoutRobot : public RobotCommonInterface, public ScoutInterface {
 public:
  ScoutRobot(ProtocolVersion protocol = ProtocolVersion::AGX_V2,
             bool is_mini_model = false);
  virtual ~ScoutRobot();

  bool Connect(std::string can_name) override;

  void EnableCommandedMode() override;
  std::string RequestVersion(int timeout_sec = 3) override;

  void SetMotionCommand(double linear_vel, double angular_vel) override;
  void SetLightCommand(AgxLightMode f_mode, uint8_t f_value, AgxLightMode r_mode,
                       uint8_t r_value) override;
  void DisableLightControl() override;

  void ResetRobotState() override;

  ProtocolVersion GetParserProtocolVersion() override;

  // get robot state
  ScoutCoreState GetRobotState() override;
  ScoutActuatorState GetActuatorState() override;
  ScoutCommonSensorState GetCommonSensorState() override;

 protected:
  RobotCommonInterface* robot_;
};

///////////////////////////////////////////////////////////////////////////

class ScoutMiniOmniRobot : public ScoutRobot, public ScoutOmniInterface {
 public:
  ScoutMiniOmniRobot(ProtocolVersion protocol = ProtocolVersion::AGX_V2);

  void SetMotionCommand(double linear_vel, double angular_vel,
                        double lateral_velocity) override;

 private:
  using ScoutRobot::SetMotionCommand;
};
}  // namespace westonrobot

#endif /* SCOUT_ROBOT_HPP */
