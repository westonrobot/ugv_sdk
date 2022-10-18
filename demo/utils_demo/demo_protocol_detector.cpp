/*
 * demo_protocol_detect.cpp
 *
 * Created on: Jul 15, 2021 14:10
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include <memory>
#include <iostream>

#include "ugv_sdk/utilities/protocol_detector.hpp"

using namespace westonrobot;

int main(int argc, char **argv) {
  ProtocolDetector detector;
  if (detector.Connect("can0")) {
    auto proto = detector.DetectProtocolVersion(5);

    if (proto == ProtocolVersion::AGX_V1) {
      std::cout << "Detected protocol: AGX_V1" << std::endl;
    } else if (proto == ProtocolVersion::AGX_V2) {
      std::cout << "Detected protocol: AGX_V2" << std::endl;
    } else {
      std::cout << "Detected protocol: UNKONWN" << std::endl;
    }
  } else {
    std::cout << "Failed to open port" << std::endl;
    return -1;
  }

  return 0;
}