/*
 * async_can.cpp
 *
 * Created on: Sep 10, 2020 13:23
 * Description:
 *
 * Copyright (c) 2020 Weston Robot Pte. Ltd.
 */

#include "ugv_sdk/details/async_port/async_can.hpp"

#include <net/if.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/can.h>

#include <iostream>

namespace westonrobot {
AsyncCAN::AsyncCAN(std::string can_port)
    : AsyncPortBase(can_port), socketcan_stream_(io_context_) {}

bool AsyncCAN::SetupPort() {
  try {
    const size_t iface_name_size = strlen(port_.c_str()) + 1;
    if (iface_name_size > IFNAMSIZ) return false;

    can_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (can_fd_ < 0) return false;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    memcpy(ifr.ifr_name, port_.c_str(), iface_name_size);

    const int ioctl_result = ioctl(can_fd_, SIOCGIFINDEX, &ifr);
    if (ioctl_result < 0) {
      StopService();
      return false;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    const int bind_result =
        bind(can_fd_, (struct sockaddr *)&addr, sizeof(addr));
    if (bind_result < 0) {
      StopService();
      return false;
    }

    port_opened_ = true;
    std::cout << "Start listening to port: " << port_ << std::endl;
  } catch (std::system_error &e) {
    std::cout << e.what() << std::endl;
    return false;
  }

  // give some work to io_service to start async io chain
  socketcan_stream_.assign(can_fd_);

#if ASIO_VERSION < 101200L
  io_context_.post(std::bind(&AsyncCAN::ReadFromPort, this,
                             std::ref(rcv_frame_),
                             std::ref(socketcan_stream_)));
#else
  asio::post(io_context_,
             std::bind(&AsyncCAN::ReadFromPort, this, std::ref(rcv_frame_),
                       std::ref(socketcan_stream_)));
#endif

  return true;
}

void AsyncCAN::StopService() {
  // stop io thread
  io_context_.stop();
  if (io_thread_.joinable()) io_thread_.join();
  io_context_.reset();

  // release port fd
  const int close_result = ::close(can_fd_);
  can_fd_ = -1;

  port_opened_ = false;
}

void AsyncCAN::DefaultReceiveCallback(can_frame *rx_frame) {
  std::cout << std::hex << rx_frame->can_id << "  ";
  for (int i = 0; i < rx_frame->can_dlc; i++)
    std::cout << std::hex << int(rx_frame->data[i]) << " ";
  std::cout << std::dec << std::endl;
}

void AsyncCAN::ReadFromPort(struct can_frame &rec_frame,
                            asio::posix::basic_stream_descriptor<> &stream) {
  auto sthis = shared_from_this();
  stream.async_read_some(
      asio::buffer(&rec_frame, sizeof(rec_frame)),
      [sthis](asio::error_code error, size_t bytes_transferred) {
        if (error) {
          sthis->StopService();
          return;
        }

        if (sthis->rcv_cb_ != nullptr)
          sthis->rcv_cb_(&sthis->rcv_frame_);
        else
          sthis->DefaultReceiveCallback(&sthis->rcv_frame_);

        sthis->ReadFromPort(std::ref(sthis->rcv_frame_),
                            std::ref(sthis->socketcan_stream_));
      });
}

void AsyncCAN::SendFrame(const struct can_frame &frame) {
  socketcan_stream_.async_write_some(
      asio::buffer(&frame, sizeof(frame)),
      [](asio::error_code error, size_t bytes_transferred) {
        if (error) {
          std::cerr << "Failed to send CAN frame" << std::endl;
        }
        // std::cout << "frame sent" << std::endl;
      });
}

}  // namespace westonrobot