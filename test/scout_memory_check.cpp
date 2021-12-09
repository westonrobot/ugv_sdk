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
#include "ugv_sdk/utilities/protocol_detector.hpp"

using namespace westonrobot;

int main(int argc, char **argv) {
  std::string device_name;

  if (argc == 2) {
    device_name = {argv[1]};
    std::cout << "Selected interface " << device_name << std::endl;
  } else {
    std::cout << "Usage: demo_scout_mini_omni_robot <interface>" << std::endl
              << "Example 1: ./demo_scout_mini_omni_robot can0" << std::endl;
    return -1;
  }

  std::unique_ptr<ScoutMiniOmniRobot> scout;

  scout = std::unique_ptr<ScoutMiniOmniRobot>(
      new ScoutMiniOmniRobot(ProtocolVersion::AGX_V2));

  if (scout == nullptr)
    std::cout << "Failed to create robot object" << std::endl;

  scout->Connect(device_name);
  scout->SetMotionCommand(0.0, 0.0, 0.8);

  // get robot state
  auto state = scout->GetRobotState();
  auto actuator = scout->GetActuatorState();

  return 0;
}