/*
 * scout_interface.hpp
 *
 * Created on: Jul 07, 2021 09:10
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef SCOUT_INTERFACE_HPP
#define SCOUT_INTERFACE_HPP

namespace westonrobot {
class ScoutInterface {
 public:
  // set up connection
  void Connect(std::string dev_name) override;

  // robot control
  void SetMotionCommand(double linear_vel, double angular_vel);
  void SetLightCommand(const ScoutLightCmd &cmd);
  void DisableLightCmdControl();

  // get robot state
  ScoutState GetScoutState();
};
}  // namespace westonrobot

#endif /* SCOUT_INTERFACE_HPP */
