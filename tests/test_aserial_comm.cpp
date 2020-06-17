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
    // std::cout << "parser called" << std::endl;

    // for (int i = 0; i < bytes_received; ++i)
    // {
    //     // auto c = *buf++;
    //     std::cout << std::hex << static_cast<int>(buf[i]) << std::dec << " ";
    // }

    if (bytes_received > 2)
    {
        for (int i = 0; i < bytes_received - 1; ++i)
        {
            uint8_t first = buf[i];
            uint8_t second = buf[i + 1];

            if (first == 0xB5 && second == 0x62)
                std::cout << "- start bytes found" << std::endl;
            // std::cout << std::hex << static_cast<int>(buf[i]) << std::dec << " ";
        }
    }
}

int main(int argc, char *argv[])
{
    std::string device_name;
    int baud;

    if (argc == 3)
    {
        device_name = {argv[1]};
        baud = std::stoi(argv[2]);
        std::cout << "Specified device: " << device_name << ", baud: " << baud << std::endl;
    }
    else
    {
        std::cout << "Usage: test_aserial_comm <interface> <baud>" << std::endl
                  << "Example: ./test_aserial_comm /dev/ttyUSB0 115200" << std::endl;
        return -1;
    }

    std::shared_ptr<ASyncSerial> serial = std::make_shared<ASyncSerial>(device_name, baud);

    serial->set_receive_callback(parse_buffer);

    if (serial->is_open())
        std::cout << "serial port opened" << std::endl;

    uint8_t data[8] = {'a', 'b', 'c'};

    int count = 0;
    uint8_t idx = 0;
    const unsigned baudrates[] = {9600, 57600, 115200};
    while (1)
    {
        // serial->send_bytes(data, 3);
        if (++count == 15)
        {
            count = 0;
            std::cout << "----------------------------------------" << std::endl;
            std::cout << "change baud rate to " << baudrates[idx % 3] << std::endl;
            std::cout << "----------------------------------------" << std::endl;
            serial->set_baud(baudrates[idx % 3]);
            idx++;
        }
        sleep(1);
    }
}
