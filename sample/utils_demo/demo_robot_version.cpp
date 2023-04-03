/**
 * @file demo_robot_version.cpp
 * @date 03-04-2023
 * @brief
 *
 * @copyright Copyright (c) 2023 Ruixiang Du (rdu)
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
    std::cout << "Selected interface " << device_name << ", robot type: scout"
              << std::endl;
  } else {
    std::cout << "Usage: " << argv[0] << " <interface>" << std::endl
              << "Example 1: ./demo_robot_version can0" << std::endl;
    return -1;
  }

  std::unique_ptr<ScoutRobot> scout;

  ProtocolDetector detector;
  if (detector.Connect(device_name)) {
    auto proto = detector.DetectProtocolVersion(5);
    if (proto == ProtocolVersion::AGX_V1) {
      std::cout << "Detected protocol: AGX_V1" << std::endl;
      scout = std::unique_ptr<ScoutRobot>(
          new ScoutRobot(ProtocolVersion::AGX_V1, false));
    } else if (proto == ProtocolVersion::AGX_V2) {
      std::cout << "Detected protocol: AGX_V2" << std::endl;
      scout = std::unique_ptr<ScoutRobot>(
          new ScoutRobot(ProtocolVersion::AGX_V2, false));
    } else {
      std::cout << "Detected protocol: UNKONWN" << std::endl;
      return -1;
    }
  } else {
    return -1;
  }

  if (scout == nullptr)
    std::cout << "Failed to create robot object" << std::endl;

  scout->Connect(device_name);

  if (scout->GetParserProtocolVersion() == ProtocolVersion::AGX_V2) {
    std::cout << "Version string: " << scout->RequestVersion() << std::endl;
  } else {
    std::cout << "AGX_V1 does not support version check" << std::endl;
  }

  return 0;
}