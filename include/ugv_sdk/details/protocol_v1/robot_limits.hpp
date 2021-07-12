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
  static constexpr double max_linear_velocity = 1.5;  // 1.5 m/s
  static constexpr double min_linear_velocity = -max_linear_velocity;
  static constexpr double max_angular_velocity = 0.5235;  // 0.5235 rad/s
  static constexpr double min_angular_velocity = -max_angular_velocity;
};

struct ScoutMiniLimits {
  static constexpr double max_linear_velocity = 3.0;  // 3.0 m/s
  static constexpr double min_linear_velocity = -max_linear_velocity;
  static constexpr double max_angular_velocity = 2.5235;  // 2.5235 rad/s
  static constexpr double min_angular_velocity = -max_angular_velocity;
};
}  // namespace westonrobot

#endif /* ROBOT_LIMITS_HPP */
