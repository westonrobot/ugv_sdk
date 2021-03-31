/* 
 * scout_monitor.hpp
 * 
 * Created on: Jun 12, 2019 01:19
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_MONITOR_HPP
#define SCOUT_MONITOR_HPP

#include "ugv_sdk/scout_base.hpp"

#include <ncurses.h>

namespace westonrobot
{
class ScoutMonitor
{
public:
    ScoutMonitor();
    ~ScoutMonitor();

    void Run(std::string device_name = "", int32_t baud_rate = 0);
    void Terminate() { keep_running_ = false; }

private:
    bool keep_running_ = true;
    bool test_mode_ = true;

    int term_sx_ = -1;
    int term_sy_ = -1;

    WINDOW *body_info_win_;
    int bi_win_sx_;
    int bi_win_sy_;
    int bi_origin_x_;
    int bi_origin_y_;

    WINDOW *system_info_win_;
    int si_win_sx_;
    int si_win_sy_;
    int si_origin_x_;
    int si_origin_y_;

    WINDOW *scout_cmd_win_;

    ScoutBase scout_base_;
    ScoutState scout_state_;

    const int linear_axis_length_ = 5;
    const int angular_axis_length_ = 5;

    const int vehicle_fp_offset_x_ = 9;
    const int vehicle_fp_offset_y_ = 9;

    bool resizing_detected_;

    void UpdateAll();
    void ClearAll();

    void CalcDimensions();
    void HandleResizing();

    void SetTestStateData();
    void ShowVehicleState(int y, int x);
    void ShowStatusItemName(int y, int x, std::string name);
    void ShowFault(int y, int x, bool no_fault);
    void ShowMotorInfo(int y, int x, double cur, int rpm, int temp, bool is_right);

    void UpdateScoutBodyInfo();
    void UpdateScoutSystemInfo();
    void UpdateScoutCmdWindow();
};
} // namespace westonrobot

#endif /* SCOUT_MONITOR_HPP */
