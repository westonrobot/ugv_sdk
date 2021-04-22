/*
 * agilex_base.hpp
 *
 * Created on: Dec 22, 2020 17:14
 * Description:
 *
 * Each robot class derived from this base class should provide implementation
 * for the following two functions:
 *
 * - virtual void Connect(std::string dev_name) = 0;
 * - virtual void ParseCANFrame(can_frame *rx_frame) = 0;
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef AGILEX_BASE_HPP
#define AGILEX_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <atomic>

#include "async_port/async_can.hpp"
#include "ugv_sdk/agilex_message.h"

namespace westonrobot {
class AgilexBase {
 public:
  AgilexBase() = default;
  virtual ~AgilexBase();

  // do not allow copy or assignment
  AgilexBase(const AgilexBase &hunter) = delete;
  AgilexBase &operator=(const AgilexBase &hunter) = delete;

  // any derived robot must implement this method with proper call back defined
  virtual void Connect(std::string dev_name) = 0;

  // cmd thread runs at 50Hz (20ms) by default
  void SetCmdThreadPeriodMs(int32_t period_ms) {
    cmd_thread_period_ms_ = period_ms;
  };

  // timeout: robot stops if user does not call SetMotionCommand() periodically
  void EnableCmdTimeout(uint32_t timeout_ms = 100);
  void DisableTimeout() { enable_timeout_ = false; }

  // switch to commanded mode
  void EnableCommandedMode();

  // enforce 50Hz command loop for all AgileX robots internally
  void SetMotionCommand(double linear_vel, double angular_vel,
                        double lateral_velocity, double steering_angle);

  // one-shot light command
  void SendLightCommand(LightMode front_mode, uint8_t front_custom_value,
                        LightMode rear_mode, uint8_t rear_custom_value);
  void DisableLightControl();

  // motion mode change
  void SetMotionMode(uint8_t mode);

  // reset fault states
  void ResetRobotState();

 protected:
  std::mutex state_mutex_;
  std::mutex motion_cmd_mutex_;
  MotionCommandMessage current_motion_cmd_;

  // communication interface
  bool can_connected_ = false;
  std::shared_ptr<AsyncCAN> can_;

  // timeout to be implemented in each vehicle
  bool enable_timeout_ = true;
  uint32_t timeout_ms_ = 500;
  uint32_t watchdog_counter_ = 0;
  void FeedCmdTimeoutWatchdog() { watchdog_counter_ = 0; };

  // command thread
  std::thread cmd_thread_;
  int32_t cmd_thread_period_ms_ = 20;
  bool cmd_thread_started_ = false;
  std::atomic<bool> keep_running_;

  // internal functions
  void StartCmdThread();
  void ControlLoop(int32_t period_ms);

  // connect to roboot from CAN or serial
  using CANFrameRxCallback = AsyncCAN::ReceiveCallback;
  void Connect(std::string dev_name, CANFrameRxCallback cb);
  void Disconnect();

  // ask background thread to shutdown properly
  void Terminate();

  void SendRobotCmd();
  virtual void ParseCANFrame(can_frame *rx_frame) = 0;
};
}  // namespace westonrobot

#endif /* AGILEX_BASE_HPP */
