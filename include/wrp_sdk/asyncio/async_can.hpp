/* 
 * async_can.hpp
 * 
 * Created on: Jun 10, 2019 02:16
 * Description: code based on uavcan and libmavconn
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

/*
 * Copyright (c) 2016 UAVCAN Team
 *
 * Distributed under the MIT License, available in the file LICENSE.
 *
 */

/*
 * libmavconn
 * Copyright 2013,2014,2015,2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#ifndef ASYNC_CAN_HPP
#define ASYNC_CAN_HPP

#include <linux/can.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <deque>
#include <functional>
#include <iostream>

#include "asio.hpp"
#include "asio/posix/basic_stream_descriptor.hpp"

// #include "async_io/device_error.hpp"
// #include "async_io/msg_buffer.hpp"

namespace westonrobot
{
using steady_clock = std::chrono::steady_clock;
using lock_guard = std::lock_guard<std::recursive_mutex>;

class ASyncCAN : public std::enable_shared_from_this<ASyncCAN>
{
public:
    static constexpr auto DEFAULT_DEVICE = "can1";
    static constexpr std::size_t MAX_TXQ_SIZE = 1000;

    using ReceiveCallback = std::function<void(can_frame *rx_frame)>;
    using ClosedCallback = std::function<void(void)>;

    using Ptr = std::shared_ptr<ASyncCAN>;
    using ConstPtr = std::shared_ptr<ASyncCAN const>;
    using WeakPtr = std::weak_ptr<ASyncCAN>;

    struct IOStat
    {
        std::size_t tx_total_frames; // total bytes transferred
        std::size_t rx_total_frames; // total bytes received
        float tx_speed;              // current transfer speed [Frames/s]
        float rx_speed;              // current receive speed [Frames/s]
    };

public:
    ASyncCAN(std::string device = DEFAULT_DEVICE);
    ~ASyncCAN();

    // do not allow copy
    ASyncCAN(const ASyncCAN &other) = delete;
    ASyncCAN &operator=(const ASyncCAN &other) = delete;

    std::size_t conn_id;
    int can_fd_;

    static Ptr open_url(std::string url);
    void open(std::string device);
    void close();

    void set_receive_callback(ReceiveCallback cb) { receive_cb = cb; }
    void set_closed_callback(ClosedCallback cb) { port_closed_cb = cb; }

    inline bool is_open() { return true; }
    IOStat get_iostat();

    void send_frame(const can_frame &tx_frame);

private:
    // monotonic counter (increment only)
    bool can_interface_opened_ = false;
    static std::atomic<std::size_t> conn_id_counter;

    struct can_frame rcv_frame;

    // port statistics
    std::atomic<std::size_t> tx_total_frames;
    std::atomic<std::size_t> rx_total_frames;
    std::size_t last_tx_total_frames;
    std::size_t last_rx_total_frames;
    std::chrono::time_point<steady_clock> last_iostat;
    std::recursive_mutex iostat_mutex;

    // io service
    asio::io_service io_service;
    asio::posix::basic_stream_descriptor<> stream;
    std::thread io_thread;

    // callback objects
    ClosedCallback port_closed_cb;
    ReceiveCallback receive_cb;

    // internal processing
    void do_read(struct can_frame &rec_frame, asio::posix::basic_stream_descriptor<> &stream);
    void do_write(bool check_tx_state);

    void call_receive_callback(can_frame *rx_frame);
    void default_receive_callback(can_frame *rx_frame);

    void iostat_tx_add(std::size_t bytes);
    void iostat_rx_add(std::size_t bytes);
};
} // namespace westonrobot

#endif /* ASYNC_CAN_HPP */
