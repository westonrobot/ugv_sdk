/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-04-19  19:41:52
 * @FileName  : ranger_demo.cpp
 * @Mail      : wangzheqie@qq.com
 * Copyright  : AgileX Robotics
 **/

#include "ugv_sdk/mobile_robot/ranger_robot.hpp"

using namespace westonrobot;

int main(int argc, char *argv[]) {
  std::string device_name;

  if (argc == 2) {
    device_name = {argv[1]};
    std::cout << "Specified CAN: " << device_name << std::endl;
  } else {
    std::cout << "Usage: app_ranger_demo <interface>" << std::endl
              << "Example 1: ./app_ranger_demo can0" << std::endl;
    return -1;
  }

  RangerRobot ranger;
  ranger.Connect(device_name);

  ranger.EnableCommandedMode();

  int count = 0;
  while (true) {
    if (count < 100) {
      std::cout << "Motor: 0.2, 0" << std::endl;
      ranger.SetMotionCommand(0.2, 0.0);
    }

    auto state = ranger.GetRobotState();
    std::cout << "-------------------------------" << std::endl;
    std::cout << "count: " << count << std::endl;
    std::cout << "control mode: "
              << static_cast<int>(state.system_state.control_mode)
              << " , vehicle state: "
              << static_cast<int>(state.system_state.vehicle_state)
              << std::endl;
    std::cout << "battery voltage: " << state.system_state.battery_voltage
              << std::endl;
    std::cout << "velocity (linear, angular): "
              << state.motion_state.linear_velocity << ", "
              << state.motion_state.angular_velocity << std::endl;
    std::cout << "-------------------------------" << std::endl;

    usleep(20000);
    ++count;
  }
  return 0;
}
