/*
 * async_listener.hpp
 *
 * Created on: Sep 10, 2020 11:50
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef ASYNC_LISTENER_HPP
#define ASYNC_LISTENER_HPP

#include <thread>
#include <mutex>
#include <functional>
#include <iostream>

#include "asio.hpp"

namespace westonrobot {
class AsyncListener {
 public:
  AsyncListener(std::string port) : port_(port) {}
  virtual ~AsyncListener(){};

  // do not allow copy
  AsyncListener() = delete;
  AsyncListener(const AsyncListener& other) = delete;

  virtual bool IsOpened() const { return port_opened_; }

  bool StartListening() {
    if (SetupPort()) {
      io_thread_ = std::thread([this]() { io_context_.run(); });
      return true;
    }
    std::cerr << "Failed to setup port, please check if specified port exits "
                 "or if you have proper permissions to access it"
              << std::endl;
    return false;
  };

  virtual void StopService() {}

 protected:
  std::string port_;
  bool port_opened_ = false;

  asio::io_context io_context_;
  std::thread io_thread_;

  virtual bool SetupPort() = 0;
};
}  // namespace westonrobot

#endif /* ASYNC_LISTENER_HPP */
