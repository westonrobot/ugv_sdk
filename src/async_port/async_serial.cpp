/*
 * async_serial.cpp
 *
 * Created on: Sep 10, 2020 13:00
 * Description:
 *
 * Copyright (c) 2020 Weston Robot Pte. Ltd.
 */

#include "ugv_sdk/details/async_port/async_serial.hpp"

#if defined(__linux__)
#include <linux/serial.h>
#endif

#include <cstring>
#include <iostream>

namespace westonrobot {

AsyncSerial::AsyncSerial(std::string port_name, uint32_t baud_rate)
    : AsyncPortBase(port_name),
      baud_rate_(baud_rate),
      serial_port_(io_context_) {}

void AsyncSerial::SetBaudRate(unsigned baudrate) {
  serial_port_.set_option(asio::serial_port_base::baud_rate(baudrate));
}

bool AsyncSerial::SetupPort() {
  using SPB = asio::serial_port_base;

  try {
    serial_port_.open(port_);

    // Set baudrate and 8N1 mode
    serial_port_.set_option(SPB::baud_rate(baud_rate_));
    serial_port_.set_option(SPB::character_size(8));
    serial_port_.set_option(SPB::parity(SPB::parity::none));
    serial_port_.set_option(SPB::stop_bits(SPB::stop_bits::one));
    serial_port_.set_option(SPB::flow_control(
        (hwflow_) ? SPB::flow_control::hardware : SPB::flow_control::none));

#if defined(__linux__)
    // Enable low latency mode on Linux
    {
      int fd = serial_port_.native_handle();
      struct serial_struct ser_info;
      ioctl(fd, TIOCGSERIAL, &ser_info);
      ser_info.flags |= ASYNC_LOW_LATENCY;
      ioctl(fd, TIOCSSERIAL, &ser_info);
    }
#endif

    port_opened_ = true;
    std::cout << "Start listening to port: " << port_ << "@" << baud_rate_
              << std::endl;
  } catch (std::system_error &e) {
    std::cout << e.what() << std::endl;
    return false;
  }

  // give some work to io_service to start async io chain
#if ASIO_VERSION < 101200L
  io_context_.post(std::bind(&AsyncSerial::ReadFromPort, this));
#else
  asio::post(io_context_, std::bind(&AsyncSerial::ReadFromPort, this));
#endif

  return true;
}

void AsyncSerial::StopService() {
  // stop io thread
  io_context_.stop();
  if (io_thread_.joinable()) io_thread_.join();
  io_context_.reset();

  if (IsOpened()) {
    serial_port_.cancel();
    serial_port_.close();
  }

  port_opened_ = false;
}

void AsyncSerial::DefaultReceiveCallback(uint8_t *data, const size_t bufsize,
                                         size_t len) {}

void AsyncSerial::ReadFromPort() {
  auto sthis = shared_from_this();
  serial_port_.async_read_some(
      asio::buffer(rx_buf_),
      [sthis](asio::error_code error, size_t bytes_transferred) {
        if (error) {
          sthis->StopService();
          return;
        }

        if (sthis->rcv_cb_ != nullptr) {
          sthis->rcv_cb_(sthis->rx_buf_.data(), sthis->rx_buf_.size(),
                         bytes_transferred);
        } else {
          sthis->DefaultReceiveCallback(
              sthis->rx_buf_.data(), sthis->rx_buf_.size(), bytes_transferred);
        }
        sthis->ReadFromPort();
      });
}

void AsyncSerial::WriteToPort(bool check_if_busy) {
  // do nothing if an async tx has already been initiated
  if (check_if_busy && tx_in_progress_) return;

  std::lock_guard<std::recursive_mutex> lock(tx_mutex_);
  if (tx_rbuf_.IsEmpty()) return;

  auto sthis = shared_from_this();
  tx_in_progress_ = true;
  auto len = tx_rbuf_.Read(tx_buf_, tx_rbuf_.GetOccupiedSize());
  serial_port_.async_write_some(
      asio::buffer(tx_buf_, len),
      [sthis](asio::error_code error, size_t bytes_transferred) {
        if (error) {
          sthis->StopService();
          return;
        }
        std::lock_guard<std::recursive_mutex> lock(sthis->tx_mutex_);
        if (sthis->tx_rbuf_.IsEmpty()) {
          sthis->tx_in_progress_ = false;
          return;
        } else {
          sthis->WriteToPort(false);
        }
      });
}

void AsyncSerial::SendBytes(const uint8_t *bytes, size_t length) {
  if (!IsOpened()) {
    std::cerr << "Failed to send, port closed" << std::endl;
    return;
  }
  assert(length < rxtx_buffer_size);
  std::lock_guard<std::recursive_mutex> lock(tx_mutex_);
  if (tx_rbuf_.GetFreeSize() < length) {
    throw std::length_error(
        "AsyncSerial::SendBytes: tx buffer overflow, try to slow down sending "
        "data");
  }
  tx_rbuf_.Write(bytes, length);
  io_context_.post(
      std::bind(&AsyncSerial::WriteToPort, shared_from_this(), true));
}
}  // namespace westonrobot