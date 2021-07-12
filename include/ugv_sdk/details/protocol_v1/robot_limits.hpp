/*
 * robot_limits.hpp
 *
 * Created on: Jul 12, 2021 14:01
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef ROBOT_LIMITS_HPP
#define ROBOT_LIMITS_HPP

namespace westonrobot {
struct ScoutV2Limits {
  static constexpr double max_linear = 1.5;  // m/s
  static constexpr double min_linear = -max_linear;
  static constexpr double max_angular = 0.5235;  // rad/s
  static constexpr double min_angular = -max_angular;
};

struct ScoutMiniLimits {
  static constexpr double max_linear = 3.0;  // m/s
  static constexpr double min_linear = -max_linear;
  static constexpr double max_angular = 2.5235;  // rad/s
  static constexpr double min_angular = -max_angular;
};

struct HunterV1Limits {
  static constexpr double max_linear = 1.5;  // m/s
  static constexpr double min_linear = -max_linear;
  static constexpr double max_angular = 25.5;  // degree
  static constexpr double min_angular = -max_angular;
};
}  // namespace westonrobot

#endif /* ROBOT_LIMITS_HPP */
