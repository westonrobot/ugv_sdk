#include <iostream>

#include "scout_base/details/scout_serial_parser.hpp"

using namespace westonrobot;

void print_msg(uint8_t data[8])
{
    for (int i = 0; i < 8; ++i)
        std::cout << std::hex << static_cast<int>(data[i]) << " ";
    std::cout << std::dec << std::endl;
}

uint8_t calc_checksum(uint8_t *buf, uint8_t len)
{
    uint8_t checksum = 0;
    for (int i = 0; i < len; ++i)
        checksum ^= buf[i];
    return checksum;
}

int main()
{
    uint8_t frame_data[16];

    // SOF
    frame_data[0] = 0x5a;
    frame_data[1] = 0xa5;

    // Frame len, type, ID
    frame_data[2] = 0x0a;
    frame_data[3] = 0xaa;
    frame_data[4] = 0x01;

    // Frame payload
    frame_data[5] = 0;
    frame_data[6] = 1;
    frame_data[7] = 2;
    frame_data[8] = 3;
    frame_data[9] = 4;
    frame_data[10] = 5;

    // Frame count, checksum
    frame_data[11] = 1;
    frame_data[12] = calc_checksum(frame_data, 12);

    ScoutSerialParser parser;
    parser.ParseBuffer(frame_data, 7 + 6);

    return 0;
}