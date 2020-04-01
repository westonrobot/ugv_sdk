/* 
 * scout_base.hpp
 * 
 * Created on: Jun 04, 2019 01:22
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_BASE_HPP
#define SCOUT_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <functional>

#include "async_io/async_can.hpp"
#include "async_io/async_serial.hpp"

#include "scout_protocol/scout_protocol.h"
#include "scout_protocol/scout_can_parser.h"
#include "scout_protocol/scout_uart_parser.h"

#include "scout_base/scout_types.hpp"

namespace wescore
{
class ScoutBase
{
public:
    ScoutBase() = default;
    ~ScoutBase();

    // do not allow copy
    ScoutBase(const ScoutBase &scout) = delete;
    ScoutBase &operator=(const ScoutBase &scout) = delete;

public:
    // connect to roboot from CAN or serial
    void Connect(std::string dev_name, int32_t baud_rate = 0);

    // disconnect from roboot, only valid for serial port
    void Disconnect();

    // cmd thread runs at 100Hz (10ms) by default
    void SetCmdThreadPeriodMs(int32_t period_ms) { cmd_thread_period_ms_ = period_ms; };

    // motion control
    void SetMotionCommand(double linear_vel, double angular_vel,
                          ScoutMotionCmd::FaultClearFlag fault_clr_flag = ScoutMotionCmd::FaultClearFlag::NO_FAULT);

    // light control
    void SetLightCommand(ScoutLightCmd cmd);
    void DisableLightCmdControl();

    // get robot state
    ScoutState GetScoutState();

private:
    // hardware communication interface
    std::shared_ptr<ASyncCAN> can_if_;
    std::shared_ptr<ASyncSerial> serial_if_;

    // CAN priority higher than serial if both connected
    bool can_connected_ = false;
    bool serial_connected_ = false;

    // serial port related variables
    uint8_t tx_cmd_len_;
    uint8_t tx_buffer_[SCOUT_CMD_BUF_LEN];

    // cmd/status update related variables
    std::thread cmd_thread_;
    std::mutex scout_state_mutex_;
    std::mutex motion_cmd_mutex_;
    std::mutex light_cmd_mutex_;

    ScoutState scout_state_;
    ScoutMotionCmd current_motion_cmd_;
    ScoutLightCmd current_light_cmd_;

    int32_t cmd_thread_period_ms_ = 10;
    bool cmd_thread_started_ = false;

    bool light_ctrl_enabled_ = false;
    bool light_ctrl_requested_ = false;

    // internal functions
    void ConfigureCANBus(const std::string &can_if_name = "can1");
    void ConfigureSerial(const std::string uart_name = "/dev/ttyUSB0", int32_t baud_rate = 115200);

    void StartCmdThread();
    void ControlLoop(int32_t period_ms);

    void SendMotionCmd(uint8_t count);
    void SendLightCmd(uint8_t count);

    void ParseCANFrame(can_frame *rx_frame);
    void ParseUARTBuffer(uint8_t *buf, const size_t bufsize, size_t bytes_received);

    void NewStatusMsgReceivedCallback(const ScoutMessage &msg);

public:
    static void UpdateScoutState(const ScoutMessage &status_msg, ScoutState &state);
};
} // namespace wescore

#endif /* SCOUT_BASE_HPP */
