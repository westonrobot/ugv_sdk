/*
 * hunter_robot_demo.cpp
 *
 * Created on: Jul 08, 2021 11:12
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include <unistd.h>

#include <memory>
#include <iostream>

#include "ugv_sdk/mobile_robot/hunter_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

using namespace westonrobot;

int main(int argc, char **argv) {
  std::string device_name;

  if (argc == 2) {
    device_name = {argv[1]};
    std::cout << "Selected interface " << device_name << std::endl;
  } else {
    std::cout << "Usage: app_hunter_demo <interface>" << std::endl
              << "Example 1: ./app_hunter_demo can0" << std::endl;
    return -1;
  }

  std::unique_ptr<HunterRobot> hunter;

  ProtocolDectctor detector;
  detector.Connect(device_name);
  auto proto = detector.DetectProtocolVersion(5);

  if (proto == ProtocolVersion::AGX_V1) {
    hunter =
        std::unique_ptr<HunterRobot>(new HunterRobot(ProtocolVersion::AGX_V1));
  } else if (proto == ProtocolVersion::AGX_V2) {
    hunter =
        std::unique_ptr<HunterRobot>(new HunterRobot(ProtocolVersion::AGX_V2));
  } else {
    std::cout << "Detected protocol: UNKONWN" << std::endl;
    return -1;
  }

  if (hunter == nullptr)
    std::cout << "Failed to create robot object" << std::endl;

  hunter->Connect(device_name);

  if (hunter->GetParserProtocolVersion() == ProtocolVersion::AGX_V2) {
    hunter->EnableCommandedMode();
    std::cout << "Protocol version 2" << std::endl;
  } else {
    std::cout << "Protocol version 1" << std::endl;
  }

  int count = 0;
  while (true) {
    // motion control
    std::cout << "Motor: 1.0, 0" << std::endl;
    // hunter->SetMotionCommand(1.0, 0.0);

    // get robot state
    auto state = hunter->GetRobotState();
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
              << state.motion_state.steering_angle << std::endl;

    auto actuator = hunter->GetActuatorState();
    if (hunter->GetParserProtocolVersion() == ProtocolVersion::AGX_V1) {
      for (int i = 0; i < 3; ++i) {
        printf("motor %d: current %f, rpm %d, driver temp %f, motor temp %f\n",
               actuator.actuator_state[i].motor_id,
               actuator.actuator_state[i].current,
               actuator.actuator_state[i].rpm,
               actuator.actuator_state[i].driver_temp,
               actuator.actuator_state[i].motor_temp);
      }
    } else {
    }
    std::cout << "-------------------------------" << std::endl;

    usleep(20000);
    ++count;
  }

  return 0;
}