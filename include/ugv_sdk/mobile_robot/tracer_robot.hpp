/*
 * tracer_robot.hpp
 *
 * Created on: Jul 13, 2021 21:59
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef TRACER_ROBOT_HPP
#define TRACER_ROBOT_HPP

#include <memory>

#include "ugv_sdk/details/interface/robot_interface.hpp"
#include "ugv_sdk/details/interface/tracer_interface.hpp"

// #include "ugv_sdk/details/robot_base/tracer_base.hpp"

namespace westonrobot {
// using TracerRobot = TracerBaseV2();
class TracerRobot : public RobotInterface, public TracerInterface {
 public:
  TracerRobot();
  ~TracerRobot();

  void Connect(std::string can_name) override;
  void Connect(std::string uart_name, uint32_t baudrate) override;

  void EnableCommandedMode() override;

  void SetMotionCommand(double linear_vel, double angular_vel) override;
  void SetLightCommand(LightMode f_mode, uint8_t f_value) override;
  void DisableLightControl() override;

  void ResetRobotState() override;

  ProtocolVersion GetProtocolVersion() override;

  // get robot state
  TracerState GetRobotState() override;

 private:
  RobotInterface* robot_;
};
}  // namespace westonrobot

#endif /* TRACER_ROBOT_HPP */
