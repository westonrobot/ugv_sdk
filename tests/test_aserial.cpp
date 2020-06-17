/*
 * test_interface.cpp
 *
 *  Created on: Dec 25, 2016
 *      Author: rdu
 */

#include <iostream>

#include "wrp_sdk/asyncio/async_serial.hpp"

using namespace westonrobot;

void parse_buffer(uint8_t *buf, const size_t bufsize, size_t bytes_received)
{
    std::cout << "parser called" << std::endl;

    for (; bytes_received > 0; bytes_received--)
    {
        auto c = *buf++;

    }
}

int main(int argc, char *argv[])
{
    // ASyncSerial::Ptr serial = ASyncSerial::open_url("/dev/ttyUSB0:115200");
    std::shared_ptr<ASyncSerial> serial = std::make_shared<ASyncSerial>("/dev/ttyO5", 115200);

    serial->set_receive_callback(parse_buffer);

    if (serial->is_open())
        std::cout << "serial port opened" << std::endl;

    uint8_t data[8] = {'a','b','c'};

    while (1)
    {
        // serial->send_bytes(data, 3);
        sleep(1);
    }
}
