/*
 * robot_interface.hpp
 *
 * Created on: Jul 08, 2021 11:48
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef ROBOT_INTERFACE_HPP
#define ROBOT_INTERFACE_HPP

#include <string>
#include <chrono>
#include <stdexcept>

#include "ugv_sdk/details/interface/agilex_message.h"
#include "ugv_sdk/details/parser_base.hpp"

#define AGX_MAX_ACTUATOR_NUM 8

namespace westonrobot {
using SdkClock = std::chrono::steady_clock;
using SdkTimePoint = std::chrono::time_point<SdkClock>;

class RobotCommonInterface {
 public:
  virtual ~RobotCommonInterface() = default;

  // functions to be implemented by class AgilexBase
  virtual void EnableCommandedMode() = 0;
  virtual std::string RequestVersion(int timeout_sec) = 0;

  // functions to be implemented by each robot class
  virtual bool Connect(std::string can_name) = 0;

  virtual void ResetRobotState() = 0;

  virtual ProtocolVersion GetParserProtocolVersion() = 0;
};
}  // namespace westonrobot

#endif /* ROBOT_INTERFACE_HPP */
