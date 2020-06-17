#include <iostream>

#include "scout_base/details/scout_serial_parser.hpp"

using namespace westonrobot;

#include "scout_base/scout_base.hpp"

#define TEST_WITHOUT_SERIAL_HARDWARE

using namespace westonrobot;

int main(int argc, char **argv)
{
    ScoutBase scout;
    // scout.ConfigureCANBus("can1");
    scout.Connect("/dev/ttyUSB0", 115200);
    // scout.StartCmdThread(10);

    // scout.SetLightCommand({ScoutLightCmd::LightMode::CONST_ON, 0, ScoutLightCmd::LightMode::CONST_ON, 0});

    int count = 0;
    while (true)
    {
        // scout.SetMotionCommand(0.5, 0.2);

        // if(count == 10)
        // {
        //     // scout.SetLightCommand({ScoutLightCmd::LightMode::LIGHT_MODE_CONST_OFF, 0, ScoutLightCmd::LightMode::LIGHT_MODE_CONST_OFF, 0});
        //     scout.DisableLightCmdControl();
        // }

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