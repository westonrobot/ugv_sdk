/* 
 * demo_scout_can.cpp
 * 
 * Created on: Jun 12, 2019 05:03
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "wrp_sdk/platforms/scout/scout_base.hpp"

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
        std::cout << "Usage: app_scout_demo <interface>" << std::endl
                  << "Example 1: ./app_scout_demo can0" << std::endl
                  << "Example 2: ./app_scout_demo /dev/ttyUSB0 115200" << std::endl;
        return -1;
    }

    ScoutBase scout;
    scout.Connect(device_name, baud_rate);

    // light control
    std::cout << "Light: const off" << std::endl;
    scout.SetLightCommand({ScoutLightCmd::LightMode::CONST_OFF, 0, ScoutLightCmd::LightMode::CONST_OFF, 0});
    sleep(3);
    std::cout << "Light: const on" << std::endl;
    scout.SetLightCommand({ScoutLightCmd::LightMode::CONST_ON, 0, ScoutLightCmd::LightMode::CONST_ON, 0});
    sleep(3);
    std::cout << "Light: breath" << std::endl;
    scout.SetLightCommand({ScoutLightCmd::LightMode::BREATH, 0, ScoutLightCmd::LightMode::BREATH, 0});
    sleep(3);
    std::cout << "Light: custom 90-80" << std::endl;
    scout.SetLightCommand({ScoutLightCmd::LightMode::CUSTOM, 90, ScoutLightCmd::LightMode::CUSTOM, 80});
    sleep(3);
    std::cout << "Light: diabled cmd control" << std::endl;
    scout.DisableLightCmdControl();

    int count = 0;
    while (true)
    {
        // motion control
        if (count < 5)
        {
            std::cout << "Motor: 0.2, 0" << std::endl;
            scout.SetMotionCommand(0.2, 0.0);
        }
        else if (count < 10)
        {
        std::cout << "Motor: 0.8, 0" << std::endl;
        scout.SetMotionCommand(0.8, 0.0);
        }
        else if (count < 15)
        {
            std::cout << "Motor: 1.5, 0" << std::endl;
            scout.SetMotionCommand(1.5, 0.0);
        }
        else if (count < 20)
        {
            std::cout << "Motor: 1.0, 0.5" << std::endl;
            scout.SetMotionCommand(1.0, 0.5);
        }
        else if (count < 25)
        {
            std::cout << "Motor: 0.0, 0" << std::endl;
            scout.SetMotionCommand(0.0, 0.0);
        }
        else if (count < 30)
        {
            std::cout << "Motor: -0.5, 0" << std::endl;
            scout.SetMotionCommand(-0.5, 0.0);
        }
        else if (count < 35)
        {
            std::cout << "Motor: -1.0, -0.5" << std::endl;
            scout.SetMotionCommand(-1.0, -0.5);
        }
        else if (count < 40)
        {
            std::cout << "Motor: 0.0, 0, Light: breath" << std::endl;
            scout.SetMotionCommand(0.0, 0.0);
            scout.SetLightCommand({ScoutLightCmd::LightMode::BREATH, 0, ScoutLightCmd::LightMode::BREATH, 0});
        }

        auto state = scout.GetScoutState();
        std::cout << "-------------------------------" << std::endl;
        std::cout << "count: " << count << std::endl;
        std::cout << "control mode: " << static_cast<int>(state.control_mode) << " , base state: " << static_cast<int>(state.base_state) << std::endl;
        std::cout << "battery voltage: " << state.battery_voltage << std::endl;
        std::cout << "velocity (linear, angular): " << state.linear_velocity << ", " << state.angular_velocity << std::endl;
        std::cout << "-------------------------------" << std::endl;

        sleep(1);
        ++count;
    }

    return 0;
}