/* 
 * scout_can_protocol_test.cpp
 * 
 * Created on: Jun 11, 2019 23:26
 * Description: 
 * 
 * Reference:
 * [1] https://cryptii.com/pipes/integer-encoder
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include <stdio.h>
#include <vector>

#include "gtest/gtest.h"

#include "wrp_sdk/platforms/scout/scout_base.hpp"

using namespace westonrobot;

struct ScoutCANProtocolTest : testing::Test
{
    ScoutCANProtocolTest()
    {
        // control frames
        motion_ctrl_frame.can_id = CAN_MSG_MOTION_CONTROL_CMD_ID;
        motion_ctrl_frame.can_dlc = 8;
        motion_ctrl_frame.data[0] = 0x01;
        motion_ctrl_frame.data[1] = 0x00;
        motion_ctrl_frame.data[2] = 0x0a;
        motion_ctrl_frame.data[3] = 0x00;
        motion_ctrl_frame.data[4] = 0x00;
        motion_ctrl_frame.data[5] = 0x00;
        motion_ctrl_frame.data[6] = 0x00;
        motion_ctrl_frame.data[7] = CalcScoutCANChecksum(motion_ctrl_frame.can_id,
                                                         motion_ctrl_frame.data,
                                                         motion_ctrl_frame.can_dlc);

        light_ctrl_frame.can_id = CAN_MSG_LIGHT_CONTROL_CMD_ID;
        light_ctrl_frame.can_dlc = 8;
        light_ctrl_frame.data[0] = 0x01;
        light_ctrl_frame.data[1] = 0x00;
        light_ctrl_frame.data[2] = 0x0a;
        light_ctrl_frame.data[3] = 0x00;
        light_ctrl_frame.data[4] = 0x00;
        light_ctrl_frame.data[5] = 0x00;
        light_ctrl_frame.data[6] = 0x00;
        light_ctrl_frame.data[7] = CalcScoutCANChecksum(light_ctrl_frame.can_id,
                                                        light_ctrl_frame.data,
                                                        light_ctrl_frame.can_dlc);

        // feedback frames
        motion_status_frame.can_id = CAN_MSG_MOTION_CONTROL_STATUS_ID;
        motion_status_frame.can_dlc = 8;
        motion_status_frame.data[0] = 0x04;
        motion_status_frame.data[1] = 0xe8; // 1.256
        motion_status_frame.data[2] = 0x00;
        motion_status_frame.data[3] = 0x7b; // 0.123
        motion_status_frame.data[4] = 0x00;
        motion_status_frame.data[5] = 0x00;
        motion_status_frame.data[6] = 0x00;
        motion_status_frame.data[7] = CalcScoutCANChecksum(motion_status_frame.can_id,
                                                           motion_status_frame.data,
                                                           motion_status_frame.can_dlc);

        motion_status_frame2.can_id = CAN_MSG_MOTION_CONTROL_STATUS_ID;
        motion_status_frame2.can_dlc = 8;
        motion_status_frame2.data[0] = 0xfb;
        motion_status_frame2.data[1] = 0x18; // -1.256
        motion_status_frame2.data[2] = 0xff;
        motion_status_frame2.data[3] = 0x85; // -0.123
        motion_status_frame2.data[4] = 0x00;
        motion_status_frame2.data[5] = 0x00;
        motion_status_frame2.data[6] = 0x00;
        motion_status_frame2.data[7] = CalcScoutCANChecksum(motion_status_frame2.can_id,
                                                            motion_status_frame2.data,
                                                            motion_status_frame2.can_dlc);

        light_status_frame.can_id = CAN_MSG_LIGHT_CONTROL_STATUS_ID;
        light_status_frame.can_dlc = 8;
        light_status_frame.data[0] = 0x01;
        light_status_frame.data[1] = 0x03;
        light_status_frame.data[2] = 0x55; // 85
        light_status_frame.data[3] = 0x02;
        light_status_frame.data[4] = 0x32; // 50
        light_status_frame.data[5] = 0x00;
        light_status_frame.data[6] = 0x00;
        light_status_frame.data[7] = CalcScoutCANChecksum(light_status_frame.can_id,
                                                          light_status_frame.data,
                                                          light_status_frame.can_dlc);

        system_status_frame.can_id = CAN_MSG_SYSTEM_STATUS_STATUS_ID;
        system_status_frame.can_dlc = 8;
        system_status_frame.data[0] = 0x01;
        system_status_frame.data[1] = 0x01;
        system_status_frame.data[2] = 0x01;
        system_status_frame.data[3] = 0x09; // 26.5
        system_status_frame.data[4] = 0xf0;
        system_status_frame.data[5] = 0xff;
        system_status_frame.data[6] = 0x00;
        system_status_frame.data[7] = CalcScoutCANChecksum(system_status_frame.can_id,
                                                           system_status_frame.data,
                                                           system_status_frame.can_dlc);

        motor1_driver_status_frame.can_id = CAN_MSG_MOTOR1_DRIVER_STATUS_ID;
        motor1_driver_status_frame.can_dlc = 8;
        motor1_driver_status_frame.data[0] = 0x00;
        motor1_driver_status_frame.data[1] = 0x7d; // 12.5
        motor1_driver_status_frame.data[2] = 0x04;
        motor1_driver_status_frame.data[3] = 0xd2; // 1234
        motor1_driver_status_frame.data[4] = 0x38; // 56
        motor1_driver_status_frame.data[5] = 0x00;
        motor1_driver_status_frame.data[6] = 0x00;
        motor1_driver_status_frame.data[7] = CalcScoutCANChecksum(motor1_driver_status_frame.can_id,
                                                                  motor1_driver_status_frame.data,
                                                                  motor1_driver_status_frame.can_dlc);

        motor2_driver_status_frame = motor1_driver_status_frame;
        motor2_driver_status_frame.can_id = CAN_MSG_MOTOR2_DRIVER_STATUS_ID;
        motor2_driver_status_frame.data[2] = 0xfb;
        motor2_driver_status_frame.data[3] = 0x2e; // -1234
        motor2_driver_status_frame.data[4] = 0xc8; // -56
        motor2_driver_status_frame.data[7] = CalcScoutCANChecksum(motor2_driver_status_frame.can_id,
                                                                  motor2_driver_status_frame.data,
                                                                  motor2_driver_status_frame.can_dlc);

        motor3_driver_status_frame = motor1_driver_status_frame;
        motor3_driver_status_frame.can_id = CAN_MSG_MOTOR3_DRIVER_STATUS_ID;
        motor3_driver_status_frame.data[7] = CalcScoutCANChecksum(motor3_driver_status_frame.can_id,
                                                                  motor3_driver_status_frame.data,
                                                                  motor3_driver_status_frame.can_dlc);

        motor4_driver_status_frame = motor2_driver_status_frame;
        motor4_driver_status_frame.can_id = CAN_MSG_MOTOR4_DRIVER_STATUS_ID;
        motor4_driver_status_frame.data[7] = CalcScoutCANChecksum(motor4_driver_status_frame.can_id,
                                                                  motor4_driver_status_frame.data,
                                                                  motor4_driver_status_frame.can_dlc);
    }

    ScoutBase scout_base;

    can_frame motion_ctrl_frame;
    can_frame light_ctrl_frame;

    can_frame motion_status_frame;
    can_frame motion_status_frame2;

    can_frame light_status_frame;
    can_frame system_status_frame;
    can_frame motor1_driver_status_frame;
    can_frame motor2_driver_status_frame;
    can_frame motor3_driver_status_frame;
    can_frame motor4_driver_status_frame;
};

TEST_F(ScoutCANProtocolTest, MotionStatusMsg)
{
    ScoutState state;
    ScoutMessage msg;
    DecodeScoutMsgFromCAN(&motion_status_frame, &msg);
    ScoutBase::UpdateScoutState(msg, state);

    ASSERT_FLOAT_EQ(state.linear_velocity, 1.256);
    ASSERT_FLOAT_EQ(state.angular_velocity, 0.123);

    ScoutState state2;
    ScoutMessage msg2;
    DecodeScoutMsgFromCAN(&motion_status_frame2, &msg2);
    ScoutBase::UpdateScoutState(msg2, state2);

    ASSERT_FLOAT_EQ(state2.linear_velocity, -1.256);
    ASSERT_FLOAT_EQ(state2.angular_velocity, -0.123);
}

TEST_F(ScoutCANProtocolTest, LightStatusMsg)
{
    ScoutState state;
    ScoutMessage msg;
    DecodeScoutMsgFromCAN(&light_status_frame, &msg);
    ScoutBase::UpdateScoutState(msg, state);

    ASSERT_EQ(state.light_control_enabled, true);
    ASSERT_EQ(state.front_light_state.mode, 0x03);
    ASSERT_EQ(state.front_light_state.custom_value, 85);
    ASSERT_EQ(state.rear_light_state.mode, 0x02);
    ASSERT_EQ(state.rear_light_state.custom_value, 50);
}

TEST_F(ScoutCANProtocolTest, SystemStatusMsg)
{
    ScoutState state;
    ScoutMessage msg;
    DecodeScoutMsgFromCAN(&system_status_frame, &msg);
    ScoutBase::UpdateScoutState(msg, state);

    ASSERT_EQ(state.base_state, true);
    ASSERT_EQ(state.control_mode, 0x01);
    ASSERT_FLOAT_EQ(state.battery_voltage, 26.5);
    ASSERT_EQ(state.fault_code, 0xf0ff);
}

TEST_F(ScoutCANProtocolTest, MotorDriverStatusMsg)
{
    ScoutState state;
    ScoutMessage msg1, msg2, msg3, msg4;
    DecodeScoutMsgFromCAN(&motor1_driver_status_frame, &msg1);
    DecodeScoutMsgFromCAN(&motor2_driver_status_frame, &msg2);
    DecodeScoutMsgFromCAN(&motor3_driver_status_frame, &msg3);
    DecodeScoutMsgFromCAN(&motor4_driver_status_frame, &msg4);
    ScoutBase::UpdateScoutState(msg1, state);
    ScoutBase::UpdateScoutState(msg2, state);
    ScoutBase::UpdateScoutState(msg3, state);
    ScoutBase::UpdateScoutState(msg4, state);

    ASSERT_FLOAT_EQ(state.motor_states[0].current, 12.5);
    ASSERT_FLOAT_EQ(state.motor_states[0].rpm, 1234);
    ASSERT_FLOAT_EQ(state.motor_states[0].temperature, 56);

    ASSERT_FLOAT_EQ(state.motor_states[1].current, 12.5);
    ASSERT_FLOAT_EQ(state.motor_states[1].rpm, -1234);
    ASSERT_FLOAT_EQ(state.motor_states[1].temperature, -56);

    ASSERT_FLOAT_EQ(state.motor_states[2].current, 12.5);
    ASSERT_FLOAT_EQ(state.motor_states[2].rpm, 1234);
    ASSERT_FLOAT_EQ(state.motor_states[2].temperature, 56);

    ASSERT_FLOAT_EQ(state.motor_states[3].current, 12.5);
    ASSERT_FLOAT_EQ(state.motor_states[3].rpm, -1234);
    ASSERT_FLOAT_EQ(state.motor_states[3].temperature, -56);
}
