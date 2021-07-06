/*
 * async_serial.hpp
 *
 * Created on: Sep 10, 2020 12:59
 * Description:
 *
 * Copyright (c) 2020 Weston Robot Pte. Ltd.
 */

#ifndef ASYNC_SERIAL_HPP
#define ASYNC_SERIAL_HPP

#include <cstdint>
#include <memory>
#include <array>
#include <mutex>

#include "ugv_sdk/details/async_port/async_port_base.hpp"
#include "ugv_sdk/details/async_port/ring_buffer.hpp"

namespace westonrobot {
class AsyncSerial : public AsyncPortBase,
                    public std::enable_shared_from_this<AsyncSerial> {
 public:
  using ReceiveCallback =
      std::function<void(uint8_t *data, const size_t bufsize, size_t len)>;

 public:
  AsyncSerial(std::string port_name, uint32_t baud_rate = 115200);

  void StopService() override;

  bool IsOpened() const override { return serial_port_.is_open(); }

  void SetReceiveCallback(ReceiveCallback cb) { rcv_cb_ = cb; }
  void SendBytes(const uint8_t *bytes, size_t length);

  void SetBaudRate(unsigned baudrate);
  void SetHardwareFlowControl(bool enabled) { hwflow_ = enabled; }

 private:
  asio::serial_port serial_port_;
  uint32_t baud_rate_ = 115200;
  bool hwflow_ = false;
  ReceiveCallback rcv_cb_ = nullptr;


  // tx/rx buffering
  static constexpr uint32_t rxtx_buffer_size = 1024 * 8;
  // rx buffer
  std::array<uint8_t, rxtx_buffer_size> rx_buf_;
  // tx buffer
  uint8_t tx_buf_[rxtx_buffer_size];
  RingBuffer<uint8_t, rxtx_buffer_size> tx_rbuf_;
  std::recursive_mutex tx_mutex_;
  bool tx_in_progress_ = false;

  bool SetupPort();
  void DefaultReceiveCallback(uint8_t *data, const size_t bufsize, size_t len);
  void ReadFromPort();
  void WriteToPort(bool check_if_busy);
};
}  // namespace westonrobot

#endif /* ASYNC_SERIAL_HPP */
