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
    struct MotorHighSpeedState
    {
        double current = 0; // in A
        double rpm = 0;
        double motor_pose = 0;
    };
    struct MotorLowSpeedState
    {
        double driver_voltage = 0;
        double driver_temperature = 0;
        double motor_temperature = 0;
        uint8_t driver_state = 0;
    };

    // base state
    uint8_t base_state = 0;
    uint8_t control_mode = 0;
    uint8_t park_mode = 1;
    uint16_t fault_code = 0;
    double battery_voltage = 0.0;

    uint8_t set_zero_steering = 0;

    // motor state
    static constexpr uint8_t motor_num = 3;
    MotorHighSpeedState motor_hs_state[motor_num];
    MotorLowSpeedState motor_ls_state[motor_num];

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

    HunterMotionCmd(int8_t linear_height_byte = 0, int8_t linear_low_byte = 0, int8_t angular_height_byte = 0, int8_t angular_low_byte = 0,
                   FaultClearFlag fault_clr_flag = FaultClearFlag::NO_FAULT)
        : linear_velocity_height_byte(linear_height_byte),linear_velocity_low_byte(linear_low_byte), angular_velocity_height_byte(angular_height_byte),angular_velocity_low_byte(angular_low_byte),
          fault_clear_flag(fault_clr_flag) {}

    int8_t linear_velocity_height_byte;
    int8_t linear_velocity_low_byte;
    int8_t angular_velocity_height_byte;
    int8_t angular_velocity_low_byte;
    FaultClearFlag fault_clear_flag;

    static constexpr double max_linear_velocity = 1.5;      // 1.5 m/s
    static constexpr double min_linear_velocity = -1.5;     // -1.5 m/s
    static constexpr double max_steering_angle = 0.576;    // 0.576 rad ~= 30.00 degree
    static constexpr double min_steering_angle = -0.576;   // -0.576 rad

};
} // namespace westonrobot

#endif /* HUNTER_TYPES_HPP */
