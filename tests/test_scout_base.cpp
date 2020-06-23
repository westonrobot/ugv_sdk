#include <unistd.h>

#include <thread>
#include <mutex>
#include <functional>
#include <string>
#include <iostream>
#include <chrono>
#include <cmath>

#include "wrp_sdk/platforms/scout/scout_base.hpp"

#define TEST_WITHOUT_SERIAL_HARDWARE

using namespace westonrobot;

int main(int argc, char **argv)
{
    std::string device_name;
    int32_t baud_rate = 0;

    if (argc == 2)
    {
        device_name = {argv[1]};
        std::cout << "Specified CAN: " << device_name << std::endl;
    }
    else if (argc == 3)
    {
        device_name = {argv[1]};
        baud_rate = std::stol(argv[2]);
        std::cout << "Specified serial: " << device_name << "@" << baud_rate << std::endl;
    }
    else
    {
        std::cout << "Usage: app_scout_monitor <interface>" << std::endl
                  << "Example 1: ./app_scout_monitor can1"
                  << "Example 2: ./app_scout_monitor /dev/ttyUSB0 115200" << std::endl;
        return -1;
    }

    ScoutBase scout;
    scout.Connect(device_name, baud_rate);

    scout.SetLightCommand({ScoutLightCmd::LightMode::CONST_ON, 0, ScoutLightCmd::LightMode::CONST_ON, 0});

    int count = 0;
    while (true)
    {
        scout.SetMotionCommand(0.5, 0.2);

        if(count == 10)
        {
            // scout.SetLightCommand({ScoutLightCmd::LightMode::LIGHT_MODE_CONST_OFF, 0, ScoutLightCmd::LightMode::LIGHT_MODE_CONST_OFF, 0});
            scout.DisableLightCmdControl();
        }

        auto state = scout.GetScoutState();
        std::cout << "-------------------------------" << std::endl;
        std::cout << "control mode: " << static_cast<int>(state.control_mode) << " , base state: " << static_cast<int>(state.base_state) << std::endl;
        std::cout << "battery voltage: " << state.battery_voltage << std::endl;
        std::cout << "velocity (linear, angular): " << state.linear_velocity << ", " << state.angular_velocity << std::endl;
        std::cout << "-------------------------------" << std::endl;

        sleep(1);
        ++count;
    }

    return 0;
}