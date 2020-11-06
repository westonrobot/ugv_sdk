/*
 * scout_state.hpp
 *
 * Created on: Jun 11, 2019 08:48
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_STATE_HPP
#define SCOUT_STATE_HPP

#include <cstdint>
#include <iostream>

namespace westonrobot {
struct ScoutState {
  enum MotorID {
    FRONT_RIGHT = 0,
    FRONT_LEFT = 1,
    REAR_LEFT = 2,
    REAR_RIGHT = 3
  };

  struct ActuatorState {
    double motor_current = 0;  // in A
    uint16_t motor_rpm = 0;
    uint16_t motor_pulses = 0;
    double motor_temperature = 0;

    double driver_voltage = 0;
    double driver_temperature = 0;
    uint8_t driver_state = 0;
  };

  struct LightState {
    uint8_t mode = 0;
    uint8_t custom_value = 0;
  };

  // base state
  uint8_t base_state = 0;
  uint8_t control_mode = 0;
  uint8_t fault_code = 0;
  double battery_voltage = 0.0;

  // motor state
  static constexpr uint8_t motor_num = 4;
  ActuatorState actuator_states[motor_num];

  // light state
  bool light_control_enabled = false;
  LightState front_light_state;
  LightState rear_light_state;

  // motion state
  double linear_velocity = 0;
  double angular_velocity = 0;

  // odometer state
  double left_odometry = 0;
  double right_odometry = 0;
};

struct ScoutMotionCmd {
  ScoutMotionCmd(double linear = 0.0, double angular = 0.0)
      : linear_velocity(linear), angular_velocity(angular) {}

  double linear_velocity;
  double angular_velocity;

  static constexpr double max_linear_velocity = 1.5;       // 1.5 m/s
  static constexpr double min_linear_velocity = -1.5;      // -1.5 m/s
  static constexpr double max_angular_velocity = 0.5235;   // 0.5235 rad/s
  static constexpr double min_angular_velocity = -0.5235;  // -0.5235 rad/s
};

struct ScoutLightCmd {
  enum class LightMode {
    CONST_OFF = 0x00,
    CONST_ON = 0x01,
    BREATH = 0x02,
    CUSTOM = 0x03
  };

  ScoutLightCmd() = default;
  ScoutLightCmd(LightMode f_mode, uint8_t f_value, LightMode r_mode,
                uint8_t r_value)
      : enable_ctrl(true),
        front_mode(f_mode),
        front_custom_value(f_value),
        rear_mode(r_mode),
        rear_custom_value(r_value) {}

  bool enable_ctrl = false;
  LightMode front_mode;
  uint8_t front_custom_value;
  LightMode rear_mode;
  uint8_t rear_custom_value;
};
}  // namespace westonrobot

#endif /* SCOUT_STATE_HPP */
