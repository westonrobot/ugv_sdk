/**
 * @file ugv_sdk_py.cpp
 * @brief 
 * @date 28-05-2024
 * 
 * python bindings for ugv_sdk
 * 
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */
#include <pybind11/pybind11.h>

#include "ugv_sdk/details/interface/agilex_message.h"

#include "ugv_sdk_py/agilex_message.hpp"
#include "ugv_sdk_py/scout_robot.hpp"
#include "ugv_sdk_py/hunter_robot.hpp"
#include "ugv_sdk_py/ranger_robot.hpp"

namespace py = pybind11;

using namespace westonrobot;

PYBIND11_MODULE(ugv_sdk_py, m) {
    m.doc() = "Python bindings for ugv_sdk";

    BindProtocolVersion(m);
    BindSystemStateMessage(m);
    BindMotionStateMessage(m);
    BindLightStateMessage(m);
    BindRcStateMessage(m);
    BindActuatorHSStateMessage(m);
    BindActuatorLSStateMessage(m);
    BindBmsBasicMessage(m);

    BindScoutRobot(m);
    BindHunterRobot(m);
    BindRangerRobot(m);
}