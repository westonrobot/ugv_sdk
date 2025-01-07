/*
 * bunker_robot.hpp
 *
 * Created on: Jul 14, 2021 23:08
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef BUNKER_ROBOT_HPP
#define BUNKER_ROBOT_HPP

#include <memory>

#include "ugv_sdk/details/interface/robot_common_interface.hpp"
#include "ugv_sdk/details/interface/bunker_interface.hpp"

namespace westonrobot {
class BunkerRobot : public RobotCommonInterface, public BunkerInterface {
 public:
  enum class Variant {
    kBunkerV1 = 0,
    kBunkerV2,
    kBunkerPro,
    kBunkerMini,
  };
  BunkerRobot(Variant variant);
  ~BunkerRobot();

  bool Connect(std::string can_name) override;

  void EnableCommandedMode() override;
  std::string RequestVersion(int timeout_sec = 3) override;

  void SetMotionCommand(double linear_vel, double angular_vel) override;

  void ResetRobotState() override;

  ProtocolVersion GetParserProtocolVersion() override;

  // get robot state
  BunkerCoreState GetRobotState() override;
  BunkerActuatorState GetActuatorState() override;
  BunkerCommonSensorState GetCommonSensorState() override;

 private:
  RobotCommonInterface* robot_;
};
}  // namespace westonrobot

#endif /* BUNKER_ROBOT_HPP */
