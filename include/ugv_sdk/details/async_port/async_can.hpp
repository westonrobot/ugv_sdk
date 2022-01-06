/*
 * async_can.hpp
 *
 * Created on: Sep 10, 2020 13:22
 * Description:
 *
 * Note: CAN TX is not buffered and only the latest frame will be transmitted.
 *  Buffered transmission will need to be added if a message has to be divided
 *  into multiple frames and in applications where no frame should be dropped.
 *
 * Copyright (c) 2020 Weston Robot Pte. Ltd.
 */

#ifndef ASYNC_CAN_HPP
#define ASYNC_CAN_HPP

#include <linux/can.h>

#include <atomic>
#include <memory>
#include <thread>
#include <functional>

#include "asio.hpp"
#include "asio/posix/basic_stream_descriptor.hpp"

namespace westonrobot {
class AsyncCAN : public std::enable_shared_from_this<AsyncCAN> {
 public:
  using ReceiveCallback = std::function<void(can_frame *rx_frame)>;

 public:
  AsyncCAN(std::string can_port = "can0");
  ~AsyncCAN();

  // do not allow copy
  AsyncCAN(const AsyncCAN &) = delete;
  AsyncCAN &operator=(const AsyncCAN &) = delete;

  // Public API
  bool Open();
  void Close();
  bool IsOpened() const;

  void SetReceiveCallback(ReceiveCallback cb) { rcv_cb_ = cb; }
  void SendFrame(const struct can_frame &frame);

 private:
  std::string port_;
  std::atomic<bool> port_opened_{false};

#if ASIO_VERSION < 101200L
  asio::io_service io_context_;
#else
  asio::io_context io_context_;
#endif

  std::thread io_thread_;

  int can_fd_;
  asio::posix::basic_stream_descriptor<> socketcan_stream_;

  struct can_frame rcv_frame_;
  ReceiveCallback rcv_cb_ = nullptr;

  void DefaultReceiveCallback(can_frame *rx_frame);
  void ReadFromPort(struct can_frame &rec_frame,
                    asio::posix::basic_stream_descriptor<> &stream);
};
}  // namespace westonrobot

#endif /* ASYNC_CAN_HPP */
