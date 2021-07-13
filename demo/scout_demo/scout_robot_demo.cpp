/*
 * scout_robot_demo.cpp
 *
 * Created on: Jul 08, 2021 11:12
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include <unistd.h>

#include <memory>
#include <iostream>

#include "ugv_sdk/mobile_robot/scout_robot.hpp"

using namespace westonrobot;

int main(int argc, char **argv) {
  std::string protocol_version;
  std::string device_name;

  if (argc == 3) {
    protocol_version = {argv[1]};
    device_name = {argv[2]};
    std::cout << "Use protocol " << protocol_version << " on interface "
              << device_name << std::endl;
  } else {
    std::cout << "Usage: app_scout_demo <protocol-version> <interface>"
              << std::endl
              << "Example 1: ./app_scout_demo v1 can0" << std::endl;
    return -1;
  }

  std::unique_ptr<ScoutRobot> scout;
  if (protocol_version == "v1") {
    scout =
        std::unique_ptr<ScoutRobot>(new ScoutRobot(ProtocolVersion::AGX_V1));
  } else if (protocol_version == "v2") {
    scout =
        std::unique_ptr<ScoutRobot>(new ScoutRobot(ProtocolVersion::AGX_V2));
  } else {
    std::cout << "Error: invalid protocol version string" << std::endl;
    return -1;
  }

  if (scout == nullptr)
    std::cout << "Failed to create robot object" << std::endl;

  scout->Connect(device_name);

  if (scout->GetProtocolVersion() == ProtocolVersion::AGX_V2) {
    scout->EnableCommandedMode();
    std::cout << "Protocol version 2" << std::endl;
  } else {
    std::cout << "Protocol version 1" << std::endl;
  }

  // light control
  std::cout << "Light: const off" << std::endl;
  scout->SetLightCommand(CONST_OFF, 0, CONST_OFF, 0);
  sleep(3);
  std::cout << "Light: const on" << std::endl;
  scout->SetLightCommand(CONST_ON, 0, CONST_ON, 0);
  sleep(3);
  std::cout << "Light: breath" << std::endl;
  scout->SetLightCommand(BREATH, 0, BREATH, 0);
  sleep(3);
  std::cout << "Light: custom 90-80" << std::endl;
  scout->SetLightCommand(CUSTOM, 90, CUSTOM, 80);
  sleep(3);
  std::cout << "Light: diabled cmd control" << std::endl;
  scout->DisableLightControl();

  int count = 0;
  while (true) {
    // motion control
    // if (count < 100) {
    //   std::cout << "Motor: 0.2, 0" << std::endl;
    //   scout->SetMotionCommand(0.2, 0.0);
    // }

    // get robot state
    auto state = scout->GetRobotState();
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

    if (scout->GetProtocolVersion() == ProtocolVersion::AGX_V1) {
      for (int i = 0; i < 4; ++i) {
        printf("motor %d: current %f, rpm %d, driver temp %f, motor temp %f\n",
               state.actuator_state[i].motor_id,
               state.actuator_state[i].current, state.actuator_state[i].rpm,
               state.actuator_state[i].driver_temp,
               state.actuator_state[i].motor_temp);
      }
    } else {
    }
    std::cout << "-------------------------------" << std::endl;

    usleep(20000);
    ++count;
  }

  return 0;
}