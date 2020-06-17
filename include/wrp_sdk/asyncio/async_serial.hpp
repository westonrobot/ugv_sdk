/* 
 * async_serial.hpp
 * 
 * Created on: Nov 23, 2018 22:18
 * Description: asynchronous serial communication using asio
 *              adapted from code in libmavconn
 * 
 * Main changes: 1. Removed dependency on Boost (asio standalone
 *                  and C++ STL only)
 *               2. Removed dependency on console-bridge
 *               3. Removed mavlink related code
 *               4. Removed UDP/TCP related code
 * 
 * Author: Vladimir Ermakov <vooon341@gmail.com>
 *         Ruixiang Du <ruixiang.du@gmail.com>
 * 
 * Additioanl reference:
 *  [1] http://www.webalice.it/fede.tft/serial_port/serial_port.html
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

#ifndef ASYNC_SERIAL_HPP
#define ASYNC_SERIAL_HPP

#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <deque>
#include <functional>

#include "asio.hpp"

#include "wrp_sdk/asyncio/device_error.hpp"
#include "wrp_sdk/asyncio/msg_buffer.hpp"

namespace westonrobot
{
using steady_clock = std::chrono::steady_clock;
using lock_guard = std::lock_guard<std::recursive_mutex>;

/// Note: instance of ASyncSerial MUST be managed by a std::shared_ptr<ASyncSerial>
class ASyncSerial : public std::enable_shared_from_this<ASyncSerial>
{
public:
    static constexpr auto DEFAULT_DEVICE = "/dev/ttyUSB0";
    static constexpr auto DEFAULT_BAUDRATE = 115200;
    static constexpr std::size_t MAX_TXQ_SIZE = 1000;

    /**
     * @param buf: pointer to a buffer
     * @param bufsize: size of the buffer (the value should be constant in most cases)
     * @param bytes_received: number of bytes received inside the buffer
     */
    using ReceiveCallback = std::function<void(uint8_t *buf, const size_t bufsize, size_t bytes_received)>;
    using ClosedCallback = std::function<void(void)>;

    using Ptr = std::shared_ptr<ASyncSerial>;
    using ConstPtr = std::shared_ptr<ASyncSerial const>;
    using WeakPtr = std::weak_ptr<ASyncSerial>;

    struct IOStat
    {
        std::size_t tx_total_bytes; //!< total bytes transferred
        std::size_t rx_total_bytes; //!< total bytes received
        float tx_speed;             //!< current transfer speed [B/s]
        float rx_speed;             //!< current receive speed [B/s]
    };

public:
    ASyncSerial(std::string device = DEFAULT_DEVICE, unsigned baudrate = DEFAULT_BAUDRATE, bool hwflow = false);
    ~ASyncSerial();

    // do not allow copy
    ASyncSerial(const ASyncSerial &other) = delete;
    ASyncSerial &operator=(const ASyncSerial &other) = delete;

    std::size_t conn_id;

    static Ptr open_url(std::string url);
    void open(std::string device = "", unsigned baudrate = 0, bool hwflow = false);
    void close();

    void set_baud(unsigned baudrate) { serial_dev_.set_option(asio::serial_port_base::baud_rate(baudrate)); }

    void send_bytes(const uint8_t *bytes, size_t length);
    void set_receive_callback(ReceiveCallback cb) { receive_cb = cb; }
    void set_closed_callback(ClosedCallback cb) { port_closed_cb = cb; }

    inline bool is_open() { return serial_dev_.is_open(); }
    IOStat get_iostat();

private:
    // monotonic counter (increment only)
    static std::atomic<std::size_t> conn_id_counter;

    // port properties
    std::string device_ = DEFAULT_DEVICE;
    unsigned baudrate_ = DEFAULT_BAUDRATE;
    bool hwflow_ = false;

    // port statistics
    std::atomic<std::size_t> tx_total_bytes;
    std::atomic<std::size_t> rx_total_bytes;
    std::size_t last_tx_total_bytes;
    std::size_t last_rx_total_bytes;
    std::chrono::time_point<steady_clock> last_iostat;
    std::recursive_mutex iostat_mutex;

    // io service
    asio::io_service io_service;
    std::thread io_thread;
    asio::serial_port serial_dev_;

    std::atomic<bool> tx_in_progress;
    std::deque<MsgBuffer> tx_q;
    std::array<uint8_t, MsgBuffer::MAX_SIZE> rx_buf;
    std::recursive_mutex mutex;

    // callback objects
    ClosedCallback port_closed_cb;
    ReceiveCallback receive_cb;

    // internal processing
    void do_read();
    void do_write(bool check_tx_state);

    void call_receive_callback(uint8_t *buf, const std::size_t bufsize, std::size_t bytes_received);
    void default_receive_callback(uint8_t *buf, const std::size_t bufsize, std::size_t bytes_received);

    void iostat_tx_add(std::size_t bytes);
    void iostat_rx_add(std::size_t bytes);
};
} // namespace westonrobot

#endif /* ASYNC_SERIAL_HPP */
