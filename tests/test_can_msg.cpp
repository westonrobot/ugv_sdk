#include <iostream>

#include "scout_base/details/scout_can_parser.hpp"

using namespace westonrobot;

void print_msg(uint8_t data[8])
{
    for (int i = 0; i < 8; ++i)
        std::cout << std::hex << static_cast<int>(data[i]) << " ";
    std::cout << std::dec << std::endl;
}

int main()
{
    MotionControlMessage msg;
    msg.msg.cmd.control_mode = CTRL_MODE_CMD_CAN;
    msg.msg.cmd.fault_clear_flag = FAULT_CLR_NONE;
    msg.msg.cmd.linear_velocity_cmd = 10;
    msg.msg.cmd.angular_velocity_cmd = 0;
    msg.msg.cmd.reserved0 = 0;
    msg.msg.cmd.reserved1 = 0;
    msg.msg.cmd.count = 0;
    msg.msg.cmd.checksum = ScoutCANParser::Agilex_CANMsgChecksum(ScoutCANParser::CAN_MSG_MOTION_CONTROL_CMD_ID, msg.msg.raw, msg.len);
    print_msg(msg.msg.raw);

    LightControlMessage msg2;
    msg2.msg.cmd.light_ctrl_enable = LIGHT_DISABLE_CTRL;
    msg2.msg.cmd.front_light_mode = LIGHT_MODE_CONST_ON;
    msg2.msg.cmd.front_light_custom = 0;
    msg2.msg.cmd.rear_light_mode = LIGHT_MODE_CONST_ON;
    msg2.msg.cmd.rear_light_custom = 0;
    msg2.msg.cmd.reserved0 = 0;
    msg2.msg.cmd.count = 0;
    msg2.msg.cmd.checksum = ScoutCANParser::Agilex_CANMsgChecksum(ScoutCANParser::CAN_MSG_LIGHT_CONTROL_CMD_ID, msg2.msg.raw, msg2.len);
    print_msg(msg2.msg.raw);

    return 0;
}