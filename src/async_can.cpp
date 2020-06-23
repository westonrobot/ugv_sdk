/* 
 * async_can.cpp
 * 
 * Created on: Jun 10, 2019 02:25
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 * Copyright (c) 2016 UAVCAN Team
 */

// This is needed to enable necessary declarations in sys/
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "wrp_sdk/asyncio/async_can.hpp"

#include <net/if.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <cassert>
#include <iostream>

#include "asyncio_utils.hpp"

using namespace westonrobot;

using asio::buffer;
using asio::io_service;
using std::error_code;

std::atomic<size_t> ASyncCAN::conn_id_counter{0};

ASyncCAN::ASyncCAN(std::string device) : tx_total_frames(0),
                                         rx_total_frames(0),
                                         last_tx_total_frames(0),
                                         last_rx_total_frames(0),
                                         last_iostat(steady_clock::now()),
                                         io_service(),
                                         stream(io_service)
{
    conn_id = conn_id_counter.fetch_add(1);
    open(device);
}

ASyncCAN::~ASyncCAN()
{
    close();
}

void ASyncCAN::open(std::string device)
{
    const size_t iface_name_size = strlen(device.c_str()) + 1;
    if (iface_name_size > IFNAMSIZ)
        return;

    can_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (can_fd_ < 0)
        return;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    memcpy(ifr.ifr_name, device.c_str(), iface_name_size);

    const int ioctl_result = ioctl(can_fd_, SIOCGIFINDEX, &ifr);
    if (ioctl_result < 0)
        close();

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    const int bind_result = bind(can_fd_, (struct sockaddr *)&addr, sizeof(addr));
    if (bind_result < 0)
        close();
    ;

    can_interface_opened_ = true;

    // NOTE: shared_from_this() should not be used in constructors

    // give some work to io_service before start
    stream.assign(can_fd_);
    io_service.post(std::bind(&ASyncCAN::do_read, this, std::ref(rcv_frame), std::ref(stream)));

    // run io_service for async io
    io_thread = std::thread([this]() {
        set_this_thread_name("mcan%zu", conn_id);
        io_service.run();
    });
}

void ASyncCAN::close()
{
    // lock_guard lock(mutex);
    // if (!is_open())
    //     return;

    const int close_result = ::close(can_fd_);
    can_fd_ = -1;

    io_service.stop();

    if (io_thread.joinable())
        io_thread.join();

    io_service.reset();

    can_interface_opened_ = false;

    if (port_closed_cb)
        port_closed_cb();
}

ASyncCAN::IOStat ASyncCAN::get_iostat()
{
    std::lock_guard<std::recursive_mutex> lock(iostat_mutex);
    IOStat stat;

    stat.tx_total_frames = tx_total_frames;
    stat.rx_total_frames = rx_total_frames;

    auto d_tx = stat.tx_total_frames - last_tx_total_frames;
    auto d_rx = stat.rx_total_frames - last_rx_total_frames;
    last_tx_total_frames = stat.tx_total_frames;
    last_rx_total_frames = stat.rx_total_frames;

    auto now = steady_clock::now();
    auto dt = now - last_iostat;
    last_iostat = now;

    float dt_s = std::chrono::duration_cast<std::chrono::seconds>(dt).count();

    stat.tx_speed = d_tx / dt_s;
    stat.rx_speed = d_rx / dt_s;

    return stat;
}

void ASyncCAN::iostat_tx_add(size_t frame)
{
    tx_total_frames += frame;
}

void ASyncCAN::iostat_rx_add(size_t frame)
{
    rx_total_frames += frame;
}

void ASyncCAN::send_frame(const can_frame &tx_frame)
{
    iostat_tx_add(1);
    stream.async_write_some(asio::buffer(&tx_frame, sizeof(tx_frame)),
                            [](error_code error, size_t bytes_transferred) {
                                // std::cout << "frame sent" << std::endl;
                            });
}

void ASyncCAN::call_receive_callback(can_frame *rx_frame)
{
    // keep track of statistics
    iostat_rx_add(1);

    // call the actual parser
    if (receive_cb)
        receive_cb(rx_frame);
    else
        default_receive_callback(rx_frame);
}

void ASyncCAN::default_receive_callback(can_frame *rx_frame)
{
    // do nothing
    // std::cerr << "no callback function set" << std::endl;
    std::cout << std::hex << rx_frame->can_id << "  ";
    for (int i = 0; i < rx_frame->can_dlc; i++)
        std::cout << std::hex << int(rx_frame->data[i]) << " ";
    std::cout << std::dec << std::endl;
}

void ASyncCAN::do_read(struct can_frame &rec_frame, asio::posix::basic_stream_descriptor<> &stream)
{
    auto sthis = shared_from_this();
    stream.async_read_some(
        asio::buffer(&rcv_frame, sizeof(rcv_frame)),
        [sthis](error_code error, size_t bytes_transferred) {
            if (error)
            {
                std::cerr << "read error in connection " << sthis->conn_id << " : "
                          << error.message().c_str() << std::endl;
                sthis->close();
                return;
            }

            sthis->call_receive_callback(&sthis->rcv_frame);
            sthis->do_read(std::ref(sthis->rcv_frame), std::ref(sthis->stream));
        });
}
