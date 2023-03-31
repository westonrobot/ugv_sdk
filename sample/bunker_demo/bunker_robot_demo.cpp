/*
 * bunker_robot_demo.cpp
 *
 * Created on: Jul 08, 2021 11:12
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include <unistd.h>

#include <memory>
#include <iostream>

#include "ugv_sdk/mobile_robot/bunker_robot.hpp"

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
    std::cout << "Usage: app_bunker_demo <protocol-version> <interface>"
              << std::endl
              << "Example 1: ./app_bunker_demo v1 can0" << std::endl;
    return -1;
  }

  std::unique_ptr<BunkerRobot> bunker;
  if (protocol_version == "v1") {
    bunker =
        std::unique_ptr<BunkerRobot>(new BunkerRobot(ProtocolVersion::AGX_V1));
  } else if (protocol_version == "v2") {
    bunker =
        std::unique_ptr<BunkerRobot>(new BunkerRobot(ProtocolVersion::AGX_V2));
  } else {
    std::cout << "Error: invalid protocol version string" << std::endl;
    return -1;
  }

  if (bunker == nullptr)
    std::cout << "Failed to create robot object" << std::endl;

  bunker->Connect(device_name);

  if (bunker->GetParserProtocolVersion() == ProtocolVersion::AGX_V2) {
    bunker->EnableCommandedMode();
    std::cout << "Protocol version 2" << std::endl;
  } else {
    std::cout << "Protocol version 1" << std::endl;
  }

  int count = 0;
  while (true) {
    // motion control
    std::cout << "Motor: 1.0, 0" << std::endl;
    bunker->SetMotionCommand(1.0, 0.0);

    // get robot state
    auto state = bunker->GetRobotState();
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
    std::cout << "-------------------------------" << std::endl;

    usleep(20000);
    ++count;
  }

  return 0;
}