/* 
 * demo_scout_can.cpp
 * 
 * Created on: Jun 12, 2019 05:03
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "ugv_sdk/scout_base.hpp"

using namespace westonrobot;

int main(int argc, char **argv)
{
    std::string device_name;

    if (argc == 2)
    {
        device_name = {argv[1]};
        std::cout << "Specified CAN: " << device_name << std::endl;
    }
    else
    {
        std::cout << "Usage: app_scout_monitor <interface>" << std::endl
                  << "Example 1: ./app_scout_demo can0" << std::endl
                  << "Example 2: ./app_scout_demo /dev/ttyUSB0 115200" << std::endl;
        return -1;
    }

    ScoutBase scout;
    scout.Connect(device_name);

    scout.EnableCommandedMode();

    // light control
    std::cout << "Light: const on" << std::endl;
    scout.SetLightCommand({CONST_ON, 0, CONST_ON, 0});

    int count = 0;
    while (true)
    {
        auto state = scout.GetScoutState();

        if (state.system_state.battery_voltage >= 22.5)
        {
            scout.SetMotionCommand(1.35, 0);

            std::cout << "-------------------------------" << std::endl;
            std::cout << "------->  discharging <--------" << std::endl;
            std::cout << "elapsed time: " << count / 60 << " minutes " << count % 60 << " seconds" << std::endl;
            std::cout << "control mode: " << static_cast<int>(state.system_state.control_mode) << " , vehicle state: " << static_cast<int>(state.system_state.vehicle_state) << std::endl;
            std::cout << "battery voltage: " << state.system_state.battery_voltage << std::endl;
            std::cout << "velocity (linear, angular): " << state.motion_state.linear_velocity << ", " << state.motion_state.angular_velocity << std::endl;
            std::cout << "-------------------------------" << std::endl;
        }
        else
        {
            scout.SetMotionCommand(0, 0);
            std::cout << "-------------------------------" << std::endl;
            std::cout << "discharge stopped at: " << state.system_state.battery_voltage << " V" << std::endl;
            std::cout << "-------------------------------" << std::endl;
        }
        sleep(1);
        ++count;
    }

    return 0;
}