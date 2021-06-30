
#ifndef AGILEX_BASE_SERIALPORT_HPP
#define AGILEX_BASE_SERIALPORT_HPP

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "async_port/async_serial.hpp"
#include "ugv_sdk/agilex_message.h"
#include "ugv_sdk/details/agilex_msg_parser.h"

namespace westonrobot {

/**
 * the base class that use serial port
 */
class AgilexBaseSerialPort {
 public:
  AgilexBaseSerialPort() = default;
  virtual ~AgilexBaseSerialPort();

  // do not allow copy or assignment
  AgilexBaseSerialPort(const AgilexBaseSerialPort &) = delete;
  AgilexBaseSerialPort &operator=(const AgilexBaseSerialPort &) = delete;

  // any derived robot must implement this method with proper call back defined
  // usually: ttyUSB0, ttyTHS1
  virtual void Connect(std::string dev_name, uint32_t bouadrate) = 0;

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

  void SetBaudRate(unsigned baudrate) {
    if (serial_) {
      serial_->SetBaudRate(baudrate);
    }
  }
  // send the can format data
  void SendCanFrame(const can_frame &frame);

 protected:
  std::mutex state_mutex_;
  std::mutex motion_cmd_mutex_;
  MotionCommandMessage current_motion_cmd_;

  // communication interface
  bool serial_connected_ = false;
  std::shared_ptr<AsyncSerial> serial_{nullptr};

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

  // connect to roboot from can or serial
  using SerialFrameRxCallback = AsyncSerial::ReceiveCallback;
  void Connect(std::string dev_name, SerialFrameRxCallback cb, uint32_t bouadrate);
  void Disconnect();

  // ask background thread to shutdown properly
  void Terminate();

  void SendRobotCmd();
  //   virtual void ParseSerialFrame(can_frame *rx_frame) = 0;
};
}  // namespace westonrobot

#endif /* AGILEX_BASE_SERIALPORT_HPP */
