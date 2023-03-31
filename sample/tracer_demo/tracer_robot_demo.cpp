/*
 * scout_robot_demo.cpp
 *
 * Created on: Jul 13, 2021 22:22
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include <unistd.h>

#include <memory>
#include <iostream>

#include "ugv_sdk/mobile_robot/tracer_robot.hpp"

using namespace westonrobot;

int main(int argc, char **argv) {
  std::string device_name;

  if (argc == 2) {
    device_name = {argv[1]};
    std::cout << "Using interface "
              << device_name << std::endl;
  } else {
    std::cout << "Usage: app_tracer_demo <interface>"
              << std::endl
              << "Example 1: ./app_tracer_demo can0" << std::endl;
    return -1;
  }

  std::unique_ptr<TracerRobot> tracer;
  tracer = std::unique_ptr<TracerRobot>(new TracerRobot());
  if (tracer == nullptr)
    std::cout << "Failed to create robot object" << std::endl;

  tracer->Connect(device_name);
  tracer->EnableCommandedMode();

  // light control
  std::cout << "Light: const off" << std::endl;
  tracer->SetLightCommand(CONST_OFF, 0);
  sleep(3);
  std::cout << "Light: const on" << std::endl;
  tracer->SetLightCommand(CONST_ON, 0);
  sleep(3);
  std::cout << "Light: breath" << std::endl;
  tracer->SetLightCommand(BREATH, 0);
  sleep(3);
  std::cout << "Light: custom 30-80" << std::endl;
  tracer->SetLightCommand(CUSTOM, 30);
  sleep(3);
  //   std::cout << "Light: diabled cmd control" << std::endl;
  //   scout->DisableLightControl();
  tracer->SetLightCommand(CONST_OFF, 0);

  int count = 0;
  while (true) {
    // motion control
    std::cout << "Motor: 1.0, 0" << std::endl;
    tracer->SetMotionCommand(1.0, 0.0);

    // get robot state
    auto state = tracer->GetRobotState();
    std::cout << "-------------------------------" << std::endl;
    std::cout << "count: " << count << std::endl;
    std::cout << "control mode: "
              << static_cast<int>(state.system_state.control_mode)
              << " , vehicle state: "
              << static_cast<int>(state.system_state.vehicle_state)
              << " , error code: " << std::hex << state.system_state.error_code
              << ", battery voltage: " << state.system_state.battery_voltage
              << std::endl;
    std::cout << "velocity (linear, angular): "
              << state.motion_state.linear_velocity << ", "
              << state.motion_state.angular_velocity << std::endl;

    for (int i = 0; i < 2; ++i) {
    }
    std::cout << "-------------------------------" << std::endl;

    usleep(20000);
    ++count;
  }

  return 0;
}