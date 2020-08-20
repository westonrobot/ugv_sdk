/* 
 * async_serial.cpp
 * 
 * Created on: Nov 23, 2018 22:24
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
 */
/*
 * libmavconn
 * Copyright 2013,2014,2015,2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <cassert>
#include <iostream>

#include "wrp_sdk/asyncio/async_serial.hpp"
#include "asyncio_utils.hpp"

#if defined(__linux__)
#include <linux/serial.h>
#endif

using namespace westonrobot;

using asio::buffer;
using asio::io_service;
using std::error_code;

std::atomic<size_t> ASyncSerial::conn_id_counter{0};

ASyncSerial::ASyncSerial(std::string device, unsigned baudrate, bool hwflow) : device_(device),
                                                                               baudrate_(baudrate),
                                                                               hwflow_(hwflow), tx_total_bytes(0),
                                                                               rx_total_bytes(0),
                                                                               last_tx_total_bytes(0),
                                                                               last_rx_total_bytes(0),
                                                                               last_iostat(steady_clock::now()),
                                                                               tx_in_progress(false),
                                                                               tx_q{},
                                                                               rx_buf{},
                                                                               io_service(),
                                                                               serial_dev_(io_service)
{
    conn_id = conn_id_counter.fetch_add(1);
}

ASyncSerial::~ASyncSerial()
{
    close();
}

void ASyncSerial::open(std::string device, unsigned baudrate, bool hwflow)
{
    using SPB = asio::serial_port_base;

    if (device != "")
    {
        device_ = device;
        baudrate_ = baudrate;
        hwflow_ = hwflow;
    }

    std::cout << "connection: " << conn_id << " , device: " << device_ << " @ " << baudrate_ << "bps" << std::endl;

    try
    {
        serial_dev_.open(device_);

        // Set baudrate and 8N1 mode
        serial_dev_.set_option(SPB::baud_rate(baudrate_));
        serial_dev_.set_option(SPB::character_size(8));
        serial_dev_.set_option(SPB::parity(SPB::parity::none));
        serial_dev_.set_option(SPB::stop_bits(SPB::stop_bits::one));
        serial_dev_.set_option(SPB::flow_control((hwflow_) ? SPB::flow_control::hardware : SPB::flow_control::none));

#if defined(__linux__)
        // Enable low latency mode on Linux
        {
            int fd = serial_dev_.native_handle();

            struct serial_struct ser_info;
            ioctl(fd, TIOCGSERIAL, &ser_info);

            ser_info.flags |= ASYNC_LOW_LATENCY;

            ioctl(fd, TIOCSSERIAL, &ser_info);
        }
#endif
    }
    catch (std::system_error &err)
    {
        throw DeviceError("serial", err);
    }

    // NOTE: shared_from_this() should not be used in constructors

    // TODO is the following step necessary?
    // give some work to io_service before start
    io_service.post(std::bind(&ASyncSerial::do_read, this));

    // run io_service for async io
    io_thread = std::thread([this]() {
        set_this_thread_name("mserial%zu", conn_id);
        io_service.run();
    });
}

void ASyncSerial::close()
{
    lock_guard lock(mutex);
    if (!is_open())
        return;

    serial_dev_.cancel();
    serial_dev_.close();

    io_service.stop();

    if (io_thread.joinable())
        io_thread.join();

    io_service.reset();

    if (port_closed_cb)
        port_closed_cb();
}

ASyncSerial::IOStat ASyncSerial::get_iostat()
{
    std::lock_guard<std::recursive_mutex> lock(iostat_mutex);
    IOStat stat;

    stat.tx_total_bytes = tx_total_bytes;
    stat.rx_total_bytes = rx_total_bytes;

    auto d_tx = stat.tx_total_bytes - last_tx_total_bytes;
    auto d_rx = stat.rx_total_bytes - last_rx_total_bytes;
    last_tx_total_bytes = stat.tx_total_bytes;
    last_rx_total_bytes = stat.rx_total_bytes;

    auto now = steady_clock::now();
    auto dt = now - last_iostat;
    last_iostat = now;

    float dt_s = std::chrono::duration_cast<std::chrono::seconds>(dt).count();

    stat.tx_speed = d_tx / dt_s;
    stat.rx_speed = d_rx / dt_s;

    return stat;
}

void ASyncSerial::iostat_tx_add(size_t bytes)
{
    tx_total_bytes += bytes;
}

void ASyncSerial::iostat_rx_add(size_t bytes)
{
    rx_total_bytes += bytes;
}

void ASyncSerial::send_bytes(const uint8_t *bytes, size_t length)
{
    if (!is_open())
    {
        std::cerr << "send: channel closed! connection id: " << conn_id << std::endl;
        return;
    }

    lock_guard lock(mutex);
    if (tx_q.size() >= MAX_TXQ_SIZE)
        throw std::length_error("ASyncSerial::send_bytes: TX queue overflow");
    tx_q.emplace_back(bytes, length);
    io_service.post(std::bind(&ASyncSerial::do_write, shared_from_this(), true));
}

void ASyncSerial::call_receive_callback(uint8_t *buf, const std::size_t bufsize, std::size_t bytes_received)
{
    assert(bufsize >= bytes_received);

    // keep track of statistics
    iostat_rx_add(bytes_received);

    // call the actual parser
    if (receive_cb)
        receive_cb(buf, bufsize, bytes_received);
    else
        default_receive_callback(buf, bufsize, bytes_received);
}

void ASyncSerial::default_receive_callback(uint8_t *buf, const size_t bufsize, size_t bytes_received)
{
    // do nothing
    std::cerr << "no callback function set" << std::endl;
}

void ASyncSerial::do_read(void)
{
    auto sthis = shared_from_this();
    serial_dev_.async_read_some(
        buffer(rx_buf),
        [sthis](error_code error, size_t bytes_transferred) {
            if (error)
            {
                std::cerr << "read error in connection " << sthis->conn_id << " : "
                          << error.message().c_str() << std::endl;
                sthis->close();
                return;
            }

            sthis->call_receive_callback(sthis->rx_buf.data(), sthis->rx_buf.size(), bytes_transferred);
            sthis->do_read();
        });

    // std::cout << rx_buf.data() << std::endl;
}

void ASyncSerial::do_write(bool check_tx_state)
{
    if (check_tx_state && tx_in_progress)
        return;

    lock_guard lock(mutex);
    if (tx_q.empty())
        return;

    tx_in_progress = true;
    auto sthis = shared_from_this();
    auto &buf_ref = tx_q.front();
    serial_dev_.async_write_some(
        buffer(buf_ref.dpos(), buf_ref.nbytes()),
        [sthis, &buf_ref](error_code error, size_t bytes_transferred) {
            assert(bytes_transferred <= buf_ref.len);

            if (error)
            {
                std::cerr << "write error in connection " << sthis->conn_id << " : "
                          << error.message().c_str() << std::endl;
                sthis->close();
                return;
            }

            sthis->iostat_tx_add(bytes_transferred);
            lock_guard lock(sthis->mutex);

            if (sthis->tx_q.empty())
            {
                sthis->tx_in_progress = false;
                return;
            }

            buf_ref.pos += bytes_transferred;
            if (buf_ref.nbytes() == 0)
                sthis->tx_q.pop_front();

            if (!sthis->tx_q.empty())
                sthis->do_write(false);
            else
                sthis->tx_in_progress = false;
        });
}

//---------------------------------------------------------------------------------------//

namespace
{
ASyncSerial::Ptr url_parse_serial(
    std::string path, std::string query, bool hwflow)
{
    std::string file_path;
    int baudrate;

    url_parse_host(path, file_path, baudrate, ASyncSerial::DEFAULT_DEVICE, ASyncSerial::DEFAULT_BAUDRATE);
    url_parse_query(query);

    return std::make_shared<ASyncSerial>(file_path, baudrate, hwflow);
}
} // namespace

ASyncSerial::Ptr ASyncSerial::open_url(std::string url)
{
    /* Based on code found here:
	 * http://stackoverflow.com/questions/2616011/easy-way-to-parse-a-url-in-c-cross-platform
	 */

    const std::string proto_end("://");
    std::string proto;
    std::string host;
    std::string path;
    std::string query;

    auto proto_it = std::search(
        url.begin(), url.end(),
        proto_end.begin(), proto_end.end());
    if (proto_it == url.end())
    {
        // looks like file path
        std::cout << "URL: " << url.c_str() << " looks like file path" << std::endl;
        return url_parse_serial(url, "", false);
    }

    // copy protocol
    proto.reserve(std::distance(url.begin(), proto_it));
    std::transform(url.begin(), proto_it,
                   std::back_inserter(proto),
                   std::ref(tolower));

    // copy host
    std::advance(proto_it, proto_end.length());
    auto path_it = std::find(proto_it, url.end(), '/');
    std::transform(proto_it, path_it,
                   std::back_inserter(host),
                   std::ref(tolower));

    // copy path, and query if exists
    auto query_it = std::find(path_it, url.end(), '?');
    path.assign(path_it, query_it);
    if (query_it != url.end())
        ++query_it;
    query.assign(query_it, url.end());

    if (proto == "serial")
        return url_parse_serial(path, query, false);
    else if (proto == "serial-hwfc")
        return url_parse_serial(path, query, true);
    else
        throw DeviceError("url", "Unknown URL type");
}
