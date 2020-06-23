#include <unistd.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "wrp_sdk/platforms/hunter/hunter_base.hpp"

#define TEST_WITHOUT_SERIAL_HARDWARE

using namespace westonrobot;

int main(int argc, char **argv) {
  std::string device_name;
  int32_t baud_rate = 0;

  if (argc == 2) {
    device_name = {argv[1]};
    std::cout << "Specified CAN: " << device_name << std::endl;
  } else {
    std::cout << "Usage: app_scout_demo <interface>" << std::endl
              << "Example 1: ./app_scout_demo can0" << std::endl;
    return -1;
  }

  HunterBase scout;
  scout.Connect(device_name);

  int count = 0;
  while (true) {
    std::cout << "Motor: 0.0, 0.0," << std::endl;
    scout.SetMotionCommand(0.0, 0.0);

    auto state = scout.GetHunterState();
    std::cout << "-------------------------------" << std::endl;
    std::cout << "count: " << count << std::endl;
    std::cout << "control mode: " << static_cast<int>(state.control_mode)
              << " , base state: " << static_cast<int>(state.base_state)
              << std::endl;
    std::cout << "battery voltage: " << state.battery_voltage << std::endl;
    std::cout << "velocity (linear, angular): " << state.linear_velocity << ", "
              << state.steering_angle << std::endl;
    std::cout << "-------------------------------" << std::endl;

    sleep(1);
    ++count;
  }

  return 0;
}