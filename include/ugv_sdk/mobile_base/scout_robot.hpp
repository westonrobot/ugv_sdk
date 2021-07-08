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

#include "ugv_sdk/mobile_base/common.hpp"

namespace westonrobot {
class ScoutRobot {
 public:
  ScoutRobot(AgilexProtocol protocol);

 private:
  std::unique_ptr<ScoutInterface> robot_;
};
}  // namespace westonrobot

#endif /* SCOUT_ROBOT_HPP */
