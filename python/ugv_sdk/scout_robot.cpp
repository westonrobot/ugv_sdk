/* 
 * scout_robot.cpp
 *
 * Created on 5/8/24 2:57 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <pybind11/pybind11.h>

#include "ugv_sdk/mobile_robot/scout_robot.hpp"

namespace py = pybind11;

using namespace westonrobot;

// clang-format off
PYBIND11_MODULE(scout_robot, m) {
    m.doc() = "Python bindings for handling a Scout robot";

    py::enum_<ProtocolVersion>(m, "ProtocolVersion")
        .value("AGX_V1", ProtocolVersion::AGX_V1)
        .value("AGX_V2", ProtocolVersion::AGX_V2)
        .export_values();

    py::class_<ScoutRobot>(m, "ScoutRobot")
        .def(py::init<ProtocolVersion, bool>(),
            py::arg("protocol") = ProtocolVersion::AGX_V2,
            py::arg("is_mini_model") = false,
            "Constructor for ScoutRobot with optional protocol and model type")
        .def("Connect", &ScoutRobot::Connect,
            py::arg("can_name"),
            "Connects the robot to the specified CAN interface.")
        .def("EnableCommandedMode", &ScoutRobot::EnableCommandedMode,
             "To enable robot control")
        .def("RequestVersion", &ScoutRobot::RequestVersion,
            py::arg("timeout_sec") = 3,
            "Request the robot version")
        .def("SetMotionCommand", &ScoutRobot::SetMotionCommand,
            py::arg("linear_vel"), py::arg("angular_vel"),
            "Set the motion command for the robot");
}
// clang-format on
