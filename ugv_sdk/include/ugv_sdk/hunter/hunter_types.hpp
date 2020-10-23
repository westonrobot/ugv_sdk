/* 
 * hunter_types.hpp
 * 
 * Created on: Apr 01, 2020 09:43
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef HUNTER_TYPES_HPP
#define HUNTER_TYPES_HPP

#include <cstdint>
#include <iostream>

namespace westonrobot
{
struct HunterState
{
    struct MotorState
    {
        double current = 0; // in A
        double rpm = 0;
        double temperature = 0;
    };

    // base state
    uint8_t base_state = 0;
    uint8_t control_mode = 0;
    uint16_t fault_code = 0;
    double battery_voltage = 0.0;

    uint8_t set_zero_steering = 0;

    // motor state
    static constexpr uint8_t motor_num = 3;
    MotorState motor_states[motor_num];

    // motion state
    double linear_velocity = 0;
    double steering_angle = 0;
};

struct HunterMotionCmd
{
    enum class FaultClearFlag
    {
        NO_FAULT = 0x00,
        BAT_UNDER_VOL = 0x01,
        BAT_OVER_VOL = 0x02,
        MOTOR1_COMM = 0x03,
        MOTOR2_COMM = 0x04,
        MOTOR3_COMM = 0x05,
        MOTOR4_COMM = 0x06,
        MOTOR_DRV_OVERHEAT = 0x07,
        MOTOR_OVERCURRENT = 0x08
    };

    HunterMotionCmd(int8_t linear = 0, int8_t angular = 0,
                   FaultClearFlag fault_clr_flag = FaultClearFlag::NO_FAULT)
        : linear_velocity(linear), angular_velocity(angular),
          fault_clear_flag(fault_clr_flag) {}

    int8_t linear_velocity;
    int8_t angular_velocity;
    FaultClearFlag fault_clear_flag;

    static constexpr double max_linear_velocity = 1.5;      // 1.5 m/s
    static constexpr double min_linear_velocity = -1.5;     // -1.5 m/s
    static constexpr double max_steering_angle = 0.4622;    // 0.4622 rad ~= 26.5 degree
    static constexpr double min_steering_angle = -0.4622;   // -0.4622 rad
};
} // namespace westonrobot

#endif /* HUNTER_TYPES_HPP */
