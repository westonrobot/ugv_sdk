/* 
 * scout_robot.cpp
 *
 * Created on 5/8/24 2:57 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <pybind11/pybind11.h>

#include "agilex_message.hpp"
#include "ugv_sdk/mobile_robot/scout_robot.hpp"

namespace py = pybind11;

using namespace westonrobot;

// clang-format off
PYBIND11_MODULE(scout_robot, m) {
    m.doc() = "Python bindings for handling a Scout robot";

    BindSystemStateMessage(m);
    BindMotionStateMessage(m);
    BindLightStateMessage(m);
    BindRcStateMessage(m);

    py::enum_<ProtocolVersion>(m, "ProtocolVersion")
        .value("AGX_V1", ProtocolVersion::AGX_V1)
        .value("AGX_V2", ProtocolVersion::AGX_V2)
        .export_values();

    py::class_<ScoutCoreState>(m, "ScoutCoreState")
        .def(py::init<>())
        .def_property("time_stamp",
        [](const ScoutCoreState &s) {
                        // Convert time_point to duration since epoch in milliseconds
                        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                                s.time_stamp.time_since_epoch());
                        return duration.count();
                        },
                        [](ScoutCoreState &s, long long ms) {
                        // Convert milliseconds back to time_point
                        s.time_stamp = std::chrono::steady_clock::time_point(
                                std::chrono::milliseconds(ms));
                        })
        .def_readwrite("system_state", &ScoutCoreState::system_state)
        .def_readwrite("motion_state", &ScoutCoreState::motion_state)
        .def_readwrite("light_state", &ScoutCoreState::light_state)
        .def_readwrite("rc_state", &ScoutCoreState::rc_state);

    // ScoutRobot class
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
            "Set the motion command for the robot")
        // get robot state
        .def("GetRobotState", &ScoutRobot::GetRobotState, "Get the robot state");
}
// clang-format on
