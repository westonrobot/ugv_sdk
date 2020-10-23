/* 
 * scout_monitor.cpp
 * 
 * Created on: Jun 12, 2019 01:19
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

/*
 * Coordinate System:
 *
 *		o --------------------> x
 *		|
 *		|
 *		|
 *		|
 *		|
 *		|
 *		v
 *		y
 */

#include "monitor/scout_monitor.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <ratio>
#include <thread>

#include "monitor/nshapes.hpp"
#include "monitor/ncolors.hpp"

namespace
{
// reference: https://thispointer.com/c-convert-double-to-string-and-manage-precision-scientific-notation/
std::string ConvertFloatToString(double vel, int digit_num = 3)
{
    std::ostringstream streamObj;
    streamObj << std::fixed;
    streamObj << std::setprecision(digit_num);
    streamObj << vel;
    return streamObj.str();
}

// source: https://github.com/rxdu/stopwatch
struct StopWatch
{
    using Clock = std::chrono::high_resolution_clock;
    using time_point = typename Clock::time_point;
    using duration = typename Clock::duration;

    StopWatch() { tic_point = Clock::now(); };

    time_point tic_point;

    void tic()
    {
        tic_point = Clock::now();
    };

    double toc()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count() / 1000000.0;
    };

    // for different precisions
    double stoc()
    {
        return std::chrono::duration_cast<std::chrono::seconds>(Clock::now() - tic_point).count();
    };

    double mtoc()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tic_point).count();
    };

    double utoc()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count();
    };

    double ntoc()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - tic_point).count();
    };

    // you have to call tic() before calling this function
    void sleep_until_ms(int64_t period_ms)
    {
        int64_t duration = period_ms - std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tic_point).count();

        if (duration > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(duration));
    };

    void sleep_until_us(int64_t period_us)
    {
        int64_t duration = period_us - std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count();

        if (duration > 0)
            std::this_thread::sleep_for(std::chrono::microseconds(duration));
    };
};
} // namespace

namespace westonrobot
{
ScoutMonitor::ScoutMonitor()
{
    // init ncurses
    initscr();
    // raw();
    cbreak();
    noecho();
    nonl();
    curs_set(FALSE);
    intrflush(stdscr, FALSE);
    keypad(stdscr, TRUE);

    CalcDimensions();

    // setup sub-windows
    body_info_win_ = newwin(bi_win_sy_, bi_win_sx_, bi_origin_y_, bi_origin_x_);
    system_info_win_ = newwin(si_win_sy_, si_win_sx_, si_origin_y_, si_origin_x_);

    scout_state_.linear_velocity = 0;
    scout_state_.angular_velocity = 0;

    NColors::InitColors();
}

ScoutMonitor::~ScoutMonitor()
{
    delwin(body_info_win_);
    delwin(system_info_win_);
    endwin();
}

void ScoutMonitor::UpdateAll()
{
    ClearAll();

    CalcDimensions();
    if (resizing_detected_)
        HandleResizing();

    UpdateScoutBodyInfo();
    UpdateScoutSystemInfo();
}

void ScoutMonitor::SetTestStateData()
{
    scout_state_.base_state = BASE_STATE_NORMAL;
    scout_state_.battery_voltage = 28.5;

    scout_state_.linear_velocity = 1.234;
    scout_state_.angular_velocity = -0.6853;

    // scout_state_.fault_code |= FAULT_MOTOR_DRV_OVERHEAT_W;
    // scout_state_.fault_code |= FAULT_MOTOR_OVERCURRENT_W;
    // scout_state_.fault_code |= FAULT_MOTOR_DRV_OVERHEAT_F;
    // scout_state_.fault_code |= FAULT_MOTOR_OVERCURRENT_F;
    // scout_state_.fault_code |= FAULT_BAT_UNDER_VOL_W;
    // scout_state_.fault_code |= FAULT_BAT_UNDER_VOL_F;
    scout_state_.fault_code = 0x0000;
    // scout_state_.fault_code = 0xffff;

    // scout_state_.front_light_state.mode = CONST_ON;
    scout_state_.front_light_state.mode = LIGHT_MODE_CUSTOM;
    scout_state_.front_light_state.custom_value = 50;

    scout_state_.rear_light_state.mode = LIGHT_MODE_CONST_ON;

    scout_state_.motor_states[0].current = 10.1;
    scout_state_.motor_states[0].rpm = 2000;
    scout_state_.motor_states[0].temperature = 35;

    scout_state_.motor_states[1].current = 10.1;
    scout_state_.motor_states[1].rpm = 2000;
    scout_state_.motor_states[1].temperature = 35;

    scout_state_.motor_states[2].current = 10.1;
    scout_state_.motor_states[2].rpm = 2000;
    scout_state_.motor_states[2].temperature = 35;

    scout_state_.motor_states[3].current = 10.1;
    scout_state_.motor_states[3].rpm = 2000;
    scout_state_.motor_states[3].temperature = 35;
}

void ScoutMonitor::Run(std::string device_name, int32_t baud_rate)
{
    if (device_name != "")
        test_mode_ = false;

    if (test_mode_)
        SetTestStateData();
    else
        scout_base_.Connect(device_name, baud_rate);

    StopWatch sw;
    while (keep_running_)
    {
        // label starting point of iteration
        sw.tic();

        // query for latest robot state
        if (!test_mode_)
            scout_state_ = scout_base_.GetScoutState();

        // update window contents
        UpdateAll();

        // manage window refresh rate
        sw.sleep_until_ms(100);
    }
}

void ScoutMonitor::CalcDimensions()
{
    int sy, sx;
    getmaxyx(stdscr, sy, sx);

    if (sy != term_sy_ || sx != term_sx_)
    {
        resizing_detected_ = true;

        term_sy_ = sy;
        term_sx_ = sx;

        bi_win_sy_ = term_sy_;
        bi_win_sx_ = term_sx_ * 15 / 24;
        bi_origin_y_ = 0;
        bi_origin_x_ = 0;

        si_win_sy_ = term_sy_;
        si_win_sx_ = term_sx_ * 9 / 24;
        si_origin_y_ = 0;
        si_origin_x_ = bi_win_sx_;
    }
}

void ScoutMonitor::HandleResizing()
{
    delwin(body_info_win_);
    delwin(system_info_win_);

    body_info_win_ = newwin(bi_win_sy_, bi_win_sx_, bi_origin_y_, bi_origin_x_);
    system_info_win_ = newwin(si_win_sy_, si_win_sx_, si_origin_y_, si_origin_x_);

    resizing_detected_ = false;
}

void ScoutMonitor::ClearAll()
{
    wclear(body_info_win_);
    wclear(system_info_win_);
}

void ScoutMonitor::ShowVehicleState(int y, int x)
{
    // show linear velocity
    const int linear_axis_x = x + vehicle_fp_offset_x_;
    const int linear_axis_tip_y = y + 2;
    const int linear_axis_origin_y = linear_axis_tip_y + linear_axis_length_;
    const int linear_axis_negative_y = linear_axis_origin_y + linear_axis_length_ + 1;
    mvwprintw(body_info_win_, linear_axis_tip_y - 1, linear_axis_x, "^");
    for (int i = linear_axis_tip_y; i < linear_axis_origin_y; ++i)
        mvwprintw(body_info_win_, i, linear_axis_x, "-");
    mvwprintw(body_info_win_, linear_axis_origin_y, linear_axis_x, "x");
    for (int i = linear_axis_origin_y + 1; i < linear_axis_negative_y; ++i)
        mvwprintw(body_info_win_, i, linear_axis_x, "-");
    mvwprintw(body_info_win_, linear_axis_negative_y, linear_axis_x, "v");
    double linear_percentage = scout_state_.linear_velocity / ScoutMotionCmd::max_linear_velocity;
    int linear_bars = std::abs(static_cast<int>(linear_percentage * 5)) + 1;
    if (std::abs(scout_state_.linear_velocity) < 0.001)
        linear_bars = 0;
    if (linear_bars > 5)
        linear_bars = 5;
    if (scout_state_.linear_velocity > 0)
    {
        for (int i = linear_axis_origin_y - linear_bars; i < linear_axis_origin_y; ++i)
        {
            NColors::WSetColor(body_info_win_, NColors::BLACK, NColors::CYAN);
            mvwprintw(body_info_win_, i, linear_axis_x, "-");
            NColors::WUnsetColor(body_info_win_, NColors::BLACK, NColors::CYAN);
        }
    }
    else if (scout_state_.linear_velocity < 0)
    {
        for (int i = linear_axis_origin_y + linear_bars; i > linear_axis_origin_y; --i)
        {
            NColors::WSetColor(body_info_win_, NColors::BLACK, NColors::CYAN);
            mvwprintw(body_info_win_, i, linear_axis_x, "-");
            NColors::WUnsetColor(body_info_win_, NColors::BLACK, NColors::CYAN);
        }
    }

    // show angular velocity
    const int angular_axis_y = linear_axis_origin_y;
    const int angular_axis_origin_x = linear_axis_x;
    const int angular_axis_positive_x = angular_axis_origin_x + angular_axis_length_ + 1;
    const int angular_axis_negative_x = angular_axis_origin_x - angular_axis_length_;
    mvwprintw(body_info_win_, angular_axis_y, angular_axis_negative_x - 1, "<");
    for (int i = angular_axis_negative_x; i < angular_axis_origin_x; ++i)
        mvwprintw(body_info_win_, angular_axis_y, i, "-");
    mvwprintw(body_info_win_, linear_axis_origin_y, linear_axis_x, "x");
    for (int i = angular_axis_origin_x + 1; i < angular_axis_positive_x; ++i)
        mvwprintw(body_info_win_, angular_axis_y, i, "-");
    mvwprintw(body_info_win_, angular_axis_y, angular_axis_positive_x, ">");

    double angular_percentage = scout_state_.angular_velocity / ScoutMotionCmd::max_angular_velocity;
    int angular_bars = std::abs(static_cast<int>(angular_percentage * 5)) + 1;
    if (std::abs(scout_state_.angular_velocity) < 0.001)
        angular_bars = 0;
    if (angular_bars > 5)
        angular_bars = 5;
    if (scout_state_.angular_velocity < 0)
    {
        for (int i = angular_axis_origin_x + angular_bars; i > angular_axis_origin_x; --i)
        {
            NColors::WSetColor(body_info_win_, NColors::BLACK, NColors::MAGENTA);
            mvwprintw(body_info_win_, angular_axis_y, i, "-");
            NColors::WUnsetColor(body_info_win_, NColors::BLACK, NColors::MAGENTA);
        }
    }
    else if (scout_state_.angular_velocity > 0)
    {
        for (int i = angular_axis_origin_x - angular_bars; i < angular_axis_origin_x; ++i)
        {
            NColors::WSetColor(body_info_win_, NColors::BLACK, NColors::MAGENTA);
            mvwprintw(body_info_win_, angular_axis_y, i, "-");
            NColors::WUnsetColor(body_info_win_, NColors::BLACK, NColors::MAGENTA);
        }
    }

    // show velocity values
    std::string linear_vel_str = "linear : " + ConvertFloatToString(scout_state_.linear_velocity);
    mvwprintw(body_info_win_, linear_axis_negative_y + 2, angular_axis_negative_x - 2, linear_vel_str.c_str());

    std::string angular_vel_str = "angular: " + ConvertFloatToString(scout_state_.angular_velocity);
    mvwprintw(body_info_win_, linear_axis_negative_y + 3, angular_axis_negative_x - 2, angular_vel_str.c_str());

    // show vehicle base
    NShapes::WDrawRectangle(body_info_win_, linear_axis_tip_y - 2, angular_axis_negative_x - 4,
                            linear_axis_negative_y + 4, angular_axis_positive_x + 3);

    // show vehicle wheels
    NShapes::WDrawRectangle(body_info_win_, linear_axis_tip_y - 1, angular_axis_negative_x - 9,
                            linear_axis_tip_y + 4, angular_axis_negative_x - 5);
    NShapes::WDrawRectangle(body_info_win_, linear_axis_negative_y - 2, angular_axis_negative_x - 9,
                            linear_axis_negative_y + 3, angular_axis_negative_x - 5);
    NShapes::WDrawRectangle(body_info_win_, linear_axis_tip_y - 1, angular_axis_positive_x + 4,
                            linear_axis_tip_y + 4, angular_axis_positive_x + 8);
    NShapes::WDrawRectangle(body_info_win_, linear_axis_negative_y - 2, angular_axis_positive_x + 4,
                            linear_axis_negative_y + 3, angular_axis_positive_x + 8);

    // front right motor
    ShowMotorInfo(linear_axis_tip_y - 1, angular_axis_positive_x + 4, scout_state_.motor_states[0].current,
                  scout_state_.motor_states[0].rpm, scout_state_.motor_states[0].temperature, true);
    // front left motor
    ShowMotorInfo(linear_axis_tip_y - 1, angular_axis_negative_x - 9, scout_state_.motor_states[1].current,
                  scout_state_.motor_states[1].rpm, scout_state_.motor_states[1].temperature, false);
    // rear left motor
    ShowMotorInfo(linear_axis_negative_y - 2, angular_axis_negative_x - 9, scout_state_.motor_states[2].current,
                  scout_state_.motor_states[2].rpm, scout_state_.motor_states[2].temperature, false);
    // rear right motor
    ShowMotorInfo(linear_axis_negative_y - 2, angular_axis_positive_x + 4, scout_state_.motor_states[3].current,
                  scout_state_.motor_states[3].rpm, scout_state_.motor_states[3].temperature, true);

    // show vehicle lights
    std::string front_mode_str = "Mode: ";
    if (scout_state_.front_light_state.mode == LIGHT_MODE_CONST_ON)
        front_mode_str += "ON";
    else if (scout_state_.front_light_state.mode == LIGHT_MODE_CONST_OFF)
        front_mode_str += "OFF";
    else if (scout_state_.front_light_state.mode == LIGHT_MODE_BREATH)
        front_mode_str += "BREATH";
    else if (scout_state_.front_light_state.mode == LIGHT_MODE_CUSTOM)
        front_mode_str += "CUSTOM";
    mvwprintw(body_info_win_, linear_axis_tip_y - 4, angular_axis_origin_x - 13, front_mode_str.c_str());
    std::string front_custom_str = "Custom: " + ConvertFloatToString(scout_state_.front_light_state.custom_value, 0);
    mvwprintw(body_info_win_, linear_axis_tip_y - 4, angular_axis_origin_x + 3, front_custom_str.c_str());
    if (scout_state_.front_light_state.mode != LIGHT_MODE_CONST_OFF &&
        !(scout_state_.front_light_state.mode == LIGHT_MODE_CUSTOM && scout_state_.front_light_state.custom_value == 0))
    {
        NColors::WSetColor(body_info_win_, NColors::BRIGHT_YELLOW);
        for (int i = angular_axis_origin_x - 5; i < angular_axis_origin_x - 1; ++i)
            mvwprintw(body_info_win_, linear_axis_tip_y - 3, i, "v");
        mvwprintw(body_info_win_, linear_axis_tip_y - 3, angular_axis_origin_x, "v");
        for (int i = angular_axis_origin_x + 2; i <= angular_axis_origin_x + 5; ++i)
            mvwprintw(body_info_win_, linear_axis_tip_y - 3, i, "v");
        NColors::WUnsetColor(body_info_win_, NColors::BRIGHT_YELLOW);
    }

    std::string rear_mode_str = "Mode: ";
    if (scout_state_.rear_light_state.mode == LIGHT_MODE_CONST_ON)
        rear_mode_str += "ON";
    else if (scout_state_.rear_light_state.mode == LIGHT_MODE_CONST_OFF)
        rear_mode_str += "OFF";
    else if (scout_state_.rear_light_state.mode == LIGHT_MODE_BREATH)
        rear_mode_str += "BREATH";
    else if (scout_state_.rear_light_state.mode == LIGHT_MODE_CUSTOM)
        rear_mode_str += "CUSTOM";
    mvwprintw(body_info_win_, linear_axis_negative_y + 6, angular_axis_origin_x - 13, rear_mode_str.c_str());
    std::string rear_custom_str = "Custom: " + ConvertFloatToString(scout_state_.rear_light_state.custom_value, 0);
    mvwprintw(body_info_win_, linear_axis_negative_y + 6, angular_axis_origin_x + 3, rear_custom_str.c_str());
    if (scout_state_.rear_light_state.mode != LIGHT_MODE_CONST_OFF &&
        !(scout_state_.rear_light_state.mode == LIGHT_MODE_CUSTOM && scout_state_.rear_light_state.custom_value == 0))
    {
        NColors::WSetColor(body_info_win_, NColors::BRIGHT_RED);
        for (int i = angular_axis_origin_x - 5; i < angular_axis_origin_x - 1; ++i)
            mvwprintw(body_info_win_, linear_axis_negative_y + 5, i, "^");
        mvwprintw(body_info_win_, linear_axis_negative_y + 5, angular_axis_origin_x, "^");
        for (int i = angular_axis_origin_x + 2; i <= angular_axis_origin_x + 5; ++i)
            mvwprintw(body_info_win_, linear_axis_negative_y + 5, i, "^");
        NColors::WUnsetColor(body_info_win_, NColors::BRIGHT_RED);
    }
}

void ScoutMonitor::UpdateScoutBodyInfo()
{
    // for (int i = 0; i < bi_win_sx_; i++)
    //     mvwprintw(body_info_win_, bi_win_sy_ - 1, i, "-");

    ShowVehicleState(bi_win_sy_ / 2 - vehicle_fp_offset_y_, bi_win_sx_ / 2 - vehicle_fp_offset_x_);

    wrefresh(body_info_win_);
}

void ScoutMonitor::UpdateScoutSystemInfo()
{
    for (int i = 0; i < si_win_sy_; i++)
        mvwprintw(system_info_win_, i, 0, "|");

    const int state_title_col = (si_win_sx_ - 24) / 2;
    const int state_value_col = state_title_col + 20;
    const int state_div_col = state_value_col - 2;

    // system state
    const int sec1 = static_cast<int>(std::round((si_win_sy_ - 20) / 2.0));
    ShowStatusItemName(sec1, state_title_col, "System state");

    if (scout_state_.base_state == BASE_STATE_NORMAL)
    {
        NColors::WSetColor(system_info_win_, NColors::GREEN);
        mvwprintw(system_info_win_, sec1, state_value_col, "NORMAL");
        NColors::WUnsetColor(system_info_win_, NColors::GREEN);
    }
    else if (scout_state_.base_state == BASE_STATE_ESTOP)
    {
        NColors::WSetColor(system_info_win_, NColors::YELLOW);
        mvwprintw(system_info_win_, sec1, state_value_col, "ESTOP");
        NColors::WUnsetColor(system_info_win_, NColors::YELLOW);
    }
    else if (scout_state_.base_state == BASE_STATE_EXCEPTION)
    {
        NColors::WSetColor(system_info_win_, NColors::RED);
        mvwprintw(system_info_win_, sec1, state_value_col, "EXCEPT");
        NColors::WUnsetColor(system_info_win_, NColors::RED);
    }

    // control mode
    ShowStatusItemName(sec1 + 1, state_title_col, "Control mode");
    if (scout_state_.control_mode == CTRL_MODE_REMOTE)
        mvwprintw(system_info_win_, sec1 + 1, state_value_col, "REMOTE");
    else if (scout_state_.control_mode == CTRL_MODE_CMD_CAN)
        mvwprintw(system_info_win_, sec1 + 1, state_value_col, "CMD");
    else if (scout_state_.control_mode == CTRL_MODE_CMD_UART)
        mvwprintw(system_info_win_, sec1 + 1, state_value_col, "CMD");
    // mvwprintw(system_info_win_, sec1 + 1, state_value_col, std::to_string(scout_state_.control_mode).c_str());

    // battery voltage
    ShowStatusItemName(sec1 + 2, state_title_col, "Battery voltage");
    std::string bat_vol_str = ConvertFloatToString(scout_state_.battery_voltage, 1) + " v";
    mvwprintw(system_info_win_, sec1 + 2, state_value_col, bat_vol_str.c_str());

    const int fault_col_1 = state_value_col;
    const int fault_col_2 = fault_col_1 + 2;
    const int fault_col_3 = fault_col_2 + 2;

    const int sec2 = sec1 + 4;
    mvwprintw(system_info_win_, sec2, state_title_col, "System faults");

    // motor driver over heat;
    ShowStatusItemName(sec2 + 1, state_title_col, "-Drv over-heat");
    if ((scout_state_.fault_code & FAULT_MOTOR_DRV_OVERHEAT_W) == 0 &&
        (scout_state_.fault_code & FAULT_MOTOR_DRV_OVERHEAT_F) == 0)
    {
        NColors::WSetColor(system_info_win_, NColors::GREEN);
        mvwprintw(system_info_win_, sec2 + 1, fault_col_1, "N");
        NColors::WUnsetColor(system_info_win_, NColors::GREEN);
    }
    else
    {
        if (scout_state_.fault_code & FAULT_MOTOR_DRV_OVERHEAT_W)
        {
            NColors::WSetColor(system_info_win_, NColors::YELLOW);
            mvwprintw(system_info_win_, sec2 + 1, fault_col_2, "W");
            NColors::WUnsetColor(system_info_win_, NColors::YELLOW);
        }
        if (scout_state_.fault_code & FAULT_MOTOR_DRV_OVERHEAT_F)
        {
            NColors::WSetColor(system_info_win_, NColors::RED);
            mvwprintw(system_info_win_, sec2 + 1, fault_col_3, "P");
            NColors::WUnsetColor(system_info_win_, NColors::RED);
        }
    }

    // motor driver over current
    ShowStatusItemName(sec2 + 2, state_title_col, "-Mt over-current");
    if ((scout_state_.fault_code & FAULT_MOTOR_OVERCURRENT_W) == 0 &&
        (scout_state_.fault_code & FAULT_MOTOR_OVERCURRENT_F) == 0)
    {
        NColors::WSetColor(system_info_win_, NColors::GREEN);
        mvwprintw(system_info_win_, sec2 + 2, fault_col_1, "N");
        NColors::WUnsetColor(system_info_win_, NColors::GREEN);
    }
    else
    {
        if (scout_state_.fault_code & FAULT_MOTOR_OVERCURRENT_W)
        {
            NColors::WSetColor(system_info_win_, NColors::YELLOW);
            mvwprintw(system_info_win_, sec2 + 2, fault_col_2, "W");
            NColors::WUnsetColor(system_info_win_, NColors::YELLOW);
        }
        if (scout_state_.fault_code & FAULT_MOTOR_OVERCURRENT_F)
        {
            NColors::WSetColor(system_info_win_, NColors::RED);
            mvwprintw(system_info_win_, sec2 + 2, fault_col_3, "P");
            NColors::WUnsetColor(system_info_win_, NColors::RED);
        }
    }

    // battery under voltage
    ShowStatusItemName(sec2 + 3, state_title_col, "-Bat under volt");
    if ((scout_state_.fault_code & FAULT_BAT_UNDER_VOL_W) == 0 &&
        (scout_state_.fault_code & FAULT_BAT_UNDER_VOL_F) == 0)
    {
        NColors::WSetColor(system_info_win_, NColors::GREEN);
        mvwprintw(system_info_win_, sec2 + 3, fault_col_1, "N");
        NColors::WUnsetColor(system_info_win_, NColors::GREEN);
    }
    else
    {
        if (scout_state_.fault_code & FAULT_BAT_UNDER_VOL_W)
        {
            NColors::WSetColor(system_info_win_, NColors::YELLOW);
            mvwprintw(system_info_win_, sec2 + 3, fault_col_2, "W");
            NColors::WUnsetColor(system_info_win_, NColors::YELLOW);
        }
        if (scout_state_.fault_code & FAULT_BAT_UNDER_VOL_F)
        {
            NColors::WSetColor(system_info_win_, NColors::RED);
            mvwprintw(system_info_win_, sec2 + 3, fault_col_3, "F");
            NColors::WUnsetColor(system_info_win_, NColors::RED);
        }
    }

    // battery over voltage
    ShowStatusItemName(sec2 + 4, state_title_col, "-Bat over volt");
    ShowFault(sec2 + 4, fault_col_1, (scout_state_.fault_code & FAULT_BAT_OVER_VOL_F) == 0);

    ShowStatusItemName(sec2 + 5, state_title_col, "-RC signal loss");
    ShowFault(sec2 + 5, fault_col_1, (scout_state_.fault_code & FAULT_RC_SIGNAL_LOSS) == 0);

    const int sec3 = sec2 + 7;
    mvwprintw(system_info_win_, sec3, state_title_col, "Comm faults");

    // CAN cmd checksum
    ShowStatusItemName(sec3 + 1, state_title_col, "-CAN cmd checksum");
    ShowFault(sec3 + 1, fault_col_1, (scout_state_.fault_code & FAULT_CAN_CHECKSUM_ERROR) == 0);

    // motor comm
    ShowStatusItemName(sec3 + 2, state_title_col, "-Motor 1 comm");
    ShowFault(sec3 + 2, fault_col_1, (scout_state_.fault_code & FAULT_MOTOR1_COMM_F) == 0);

    ShowStatusItemName(sec3 + 3, state_title_col, "-Motor 2 comm");
    ShowFault(sec3 + 3, fault_col_1, (scout_state_.fault_code & FAULT_MOTOR2_COMM_F) == 0);

    ShowStatusItemName(sec3 + 4, state_title_col, "-Motor 3 comm");
    ShowFault(sec3 + 4, fault_col_1, (scout_state_.fault_code & FAULT_MOTOR3_COMM_F) == 0);

    ShowStatusItemName(sec3 + 5, state_title_col, "-Motor 4 comm");
    ShowFault(sec3 + 5, fault_col_1, (scout_state_.fault_code & FAULT_MOTOR4_COMM_F) == 0);

    const int sec4 = sec3 + 8;
    NColors::WSetColor(system_info_win_, NColors::GREEN);
    mvwprintw(system_info_win_, sec4, state_title_col + 1, "N: Normal");
    NColors::WUnsetColor(system_info_win_, NColors::GREEN);

    NColors::WSetColor(system_info_win_, NColors::YELLOW);
    mvwprintw(system_info_win_, sec4, state_title_col + 12, "W: Warning");
    NColors::WUnsetColor(system_info_win_, NColors::YELLOW);

    NColors::WSetColor(system_info_win_, NColors::RED);
    mvwprintw(system_info_win_, sec4 + 1, state_title_col + 1, "F: Fault   P: Protection");
    NColors::WUnsetColor(system_info_win_, NColors::RED);

    wrefresh(system_info_win_);
}

void ScoutMonitor::ShowStatusItemName(int y, int x, std::string name)
{
    const int state_value_col = x + 20;
    const int state_div_col = state_value_col - 2;

    mvwprintw(system_info_win_, y, x, name.c_str());
    mvwprintw(system_info_win_, y, state_div_col, ":");
}

void ScoutMonitor::ShowFault(int y, int x, bool no_fault)
{
    const int fault_col_1 = x;
    const int fault_col_2 = x + 2;
    const int fault_col_3 = fault_col_2 + 2;

    if (no_fault)
    {
        NColors::WSetColor(system_info_win_, NColors::GREEN);
        mvwprintw(system_info_win_, y, fault_col_1, "N");
        NColors::WUnsetColor(system_info_win_, NColors::GREEN);
    }
    else
    {
        NColors::WSetColor(system_info_win_, NColors::RED);
        mvwprintw(system_info_win_, y, fault_col_3, "F");
        NColors::WUnsetColor(system_info_win_, NColors::RED);
    }
}

// (y,x): position of the top left point of corresponding wheel
void ScoutMonitor::ShowMotorInfo(int y, int x, double cur, int rpm, int temp, bool is_right)
{
    int col_title = x;
    if (is_right)
        col_title += 6;
    else
        col_title -= 9;

    std::string cur_str = "CUR:" + ConvertFloatToString(cur, 1);
    mvwprintw(body_info_win_, y + 1, col_title, cur_str.c_str());

    std::string rpm_str = "RPM:" + ConvertFloatToString(rpm, 0);
    mvwprintw(body_info_win_, y + 2, col_title, rpm_str.c_str());

    std::string temp_str = "TMP:" + ConvertFloatToString(temp, 0);
    mvwprintw(body_info_win_, y + 3, col_title, temp_str.c_str());
}
} // namespace westonrobot
