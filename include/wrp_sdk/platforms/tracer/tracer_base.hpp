/* 
 * tracer_base.hpp
 * 
 * Created on: Apr 14, 2020 10:21
 * Description: 
 * 
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */ 

#ifndef TRACER_BASE_HPP
#define TRACER_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <functional>

#include "async_io/async_can.hpp"
#include "async_io/async_serial.hpp"

#include "tracer_protocol/tracer_protocol.h"
#include "tracer_protocol/tracer_can_parser.h"

#include "tracer_base/tracer_types.hpp"

namespace westonrobot
{
class TracerBase
{
public:
    TracerBase() = default;
    ~TracerBase();

    // do not allow copy
    TracerBase(const TracerBase &tracer) = delete;
    TracerBase &operator=(const TracerBase &tracer) = delete;

public:
    // connect to roboot from CAN
    void Connect(std::string dev_name);

    // disconnect from roboot, only valid for serial port
    void Disconnect();

    // cmd thread runs at 100Hz (10ms) by default
    void SetCmdThreadPeriodMs(int32_t period_ms) { cmd_thread_period_ms_ = period_ms; };

    // motion control
    void SetMotionCommand(double linear_vel, double angular_vel,
                          TracerMotionCmd::FaultClearFlag fault_clr_flag = TracerMotionCmd::FaultClearFlag::NO_FAULT);

    // light control
    void SetLightCommand(TracerLightCmd cmd);
    void DisableLightCmdControl();

    // get robot state
    TracerState GetTracerState();

private:
    // hardware communication interface
    std::shared_ptr<AsyncCAN> can_if_;
    std::shared_ptr<AsyncSerial> serial_if_;

    // CAN priority higher than serial if both connected
    bool can_connected_ = false;
    bool serial_connected_ = false;

    // serial port related variables
    uint8_t tx_cmd_len_;
    uint8_t tx_buffer_[TRACER_CMD_BUF_LEN];

    // cmd/status update related variables
    std::thread cmd_thread_;
    std::mutex tracer_state_mutex_;
    std::mutex motion_cmd_mutex_;
    std::mutex light_cmd_mutex_;

    TracerState tracer_state_;
    TracerMotionCmd current_motion_cmd_;
    TracerLightCmd current_light_cmd_;

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

    void NewStatusMsgReceivedCallback(const TracerMessage &msg);

public:
    static void UpdateTracerState(const TracerMessage &status_msg, TracerState &state);
};
} // namespace westonrobot

#endif /* TRACER_BASE_HPP */
