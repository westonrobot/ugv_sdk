/*
 * tracer_types.hpp
 *
 * Created on: Apr 14, 2020 10:22
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef TRACER_TYPES_HPP
#define TRACER_TYPES_HPP

#include <cstdint>
#include <iostream>

namespace westonrobot {
struct TracerState {
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

  // actuator state
  ActuatorState actuator_states[2];

  // light state
  bool light_control_enabled = false;
  LightState front_light_state;

  // motion state
  double linear_velocity = 0;
  double angular_velocity = 0;

  // odometer state
  double left_odometry = 0;
  double right_odometry = 0;
};

struct TracerMotionCmd {
  TracerMotionCmd(double linear = 0.0, double angular = 0.0)
      : linear_velocity(linear), angular_velocity(angular) {}

  double linear_velocity;
  double angular_velocity;

  static constexpr double max_linear_velocity = 1.8;  // m/s
  static constexpr double min_linear_velocity = -1.8;
  static constexpr double max_angular_velocity = 1.0;  //  rad/s
  static constexpr double min_angular_velocity = -1.0;
};

struct TracerLightCmd {
  enum class LightMode {
    CONST_OFF = 0x00,
    CONST_ON = 0x01,
    BREATH = 0x02,
    CUSTOM = 0x03
  };

  TracerLightCmd() = default;
  TracerLightCmd(LightMode f_mode, uint8_t f_value)
      : enable_ctrl(true), front_mode(f_mode), front_custom_value(f_value) {}

  bool enable_ctrl = false;
  LightMode front_mode;
  uint8_t front_custom_value;
  LightMode rear_mode = LightMode::CONST_OFF;
  uint8_t rear_custom_value = 0;
};
}  // namespace westonrobot

#endif /* TRACER_TYPES_HPP */
