// source: https://stackoverflow.com/questions/10467178/boostasio-over-socketcan

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <iostream>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#define ASIO_ENABLE_OLD_SERVICES
#define ASIO_HAS_POSIX_STREAM_DESCRIPTOR

#include "asio.hpp"
#include <functional>

#include "asio/posix/basic_stream_descriptor.hpp"

void data_send(void)
{
    std::cout << "omg sent" << std::endl;
}

void data_rec(struct can_frame &rec_frame,
              asio::posix::basic_stream_descriptor<> &stream)
{
    std::cout << std::hex << rec_frame.can_id << "  ";
    for (int i = 0; i < rec_frame.can_dlc; i++)
    {
        std::cout << std::hex << int(rec_frame.data[i]) << " ";
    }
    std::cout << std::dec << std::endl;
    stream.async_read_some(
        asio::buffer(&rec_frame, sizeof(rec_frame)),
        std::bind(data_rec, std::ref(rec_frame), std::ref(stream)));
}

int main(void)
{
    struct sockaddr_can addr;
    struct can_frame frame;
    struct can_frame rec_frame;
    struct ifreq ifr;

    int natsock = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    strcpy(ifr.ifr_name, "can1");
    ioctl(natsock, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(natsock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Error in socket bind");
        return -2;
    }

    frame.can_id = 0x123;
    frame.can_dlc = 2;
    frame.data[0] = 0x11;
    frame.data[1] = 0x23;

    asio::io_service ios;
    asio::posix::basic_stream_descriptor<> stream(ios);
    stream.assign(natsock);

    stream.async_write_some(asio::buffer(&frame, sizeof(frame)),
                            std::bind(data_send));
    stream.async_read_some(
        asio::buffer(&rec_frame, sizeof(rec_frame)),
        std::bind(data_rec, std::ref(rec_frame), std::ref(stream)));
    ios.run();
}