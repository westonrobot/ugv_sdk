/*
 * hunter_robot.cpp
 *
 * Created on 15/5/2024 10.03 AM
 * Description:
 *
 * Copyright (c) 2024 Weston Robot Pte. Ltd.
 */
#include "ugv_sdk_py/hunter_robot.hpp"

#include "ugv_sdk/mobile_robot/hunter_robot.hpp"

namespace py = pybind11;

namespace westonrobot {
// clang-format off
void BindHunterRobot(pybind11::module &m) {
    py::module_ m_hunter_robot = m.def_submodule("hunter_robot");
    m_hunter_robot.doc() = "Python bindings for handling a Hunter robot";

    py::class_<HunterCoreState>(m_hunter_robot, "HunterCoreState")
        .def(py::init<>())
        .def_property("time_stamp",
        [](const HunterCoreState &s) {
            // Convert time_point to duration since epoch in milliseconds
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                    s.time_stamp.time_since_epoch());
            return duration.count();
            },
            [](HunterCoreState &s, long long ms) {
            // Convert milliseconds back to time_point
            s.time_stamp = std::chrono::steady_clock::time_point(
                    std::chrono::milliseconds(ms));
            })
        .def_readwrite("system_state", &HunterCoreState::system_state)
        .def_readwrite("motion_state", &HunterCoreState::motion_state)
        .def_readwrite("rc_state", &HunterCoreState::rc_state);

    py::class_<HunterActuatorState>(m_hunter_robot, "HunterActuatorState")
        .def(py::init<>())
        .def_property("time_stamp",
            [](const HunterActuatorState &s) {
                // Convert time_point to duration since epoch in milliseconds
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                        s.time_stamp.time_since_epoch());
                return duration.count();
            },
            [](HunterActuatorState &s, long long ms) {
                // Convert milliseconds back to time_point
                s.time_stamp = std::chrono::steady_clock::time_point(
                        std::chrono::milliseconds(ms));
            })
        .def_property("actuator_hs_state",
            [](const HunterActuatorState &s) {
                // Convert C-style array to Python list
                return py::list(py::make_iterator(std::begin(s.actuator_hs_state), std::end(s.actuator_hs_state)));
            },
            [](HunterActuatorState &s, const py::list l) {
                // Convert Python list to C-style array
                if (l.size() != 3) throw std::runtime_error("List size must be 3");
                    for (size_t i = 0; i < 3; i++) {
                        s.actuator_hs_state[i] = l[i].cast<ActuatorHSStateMessage>();
                    }
            })
        .def_property("actuator_ls_state",
            [](const HunterActuatorState &s) {
                // Convert C-style array to Python list
                return py::list(py::make_iterator(std::begin(s.actuator_ls_state), std::end(s.actuator_ls_state)));
            },
            [](HunterActuatorState &s, const py::list l) {
                // Convert Python list to C-style array
                if (l.size() != 3) throw std::runtime_error("List size must be 3");
                    for (size_t i = 0; i < 3; i++) {
                        s.actuator_ls_state[i] = l[i].cast<ActuatorLSStateMessage>();
                    }
            }) 
        .def_property("actuator_state",
            [](const HunterActuatorState &s) {
                // Convert C-style array to Python list
                return py::list(py::make_iterator(std::begin(s.actuator_state), std::end(s.actuator_state)));
            },
            [](HunterActuatorState &s, const py::list l) {
                // Convert Python list to C-style array
                if (l.size() != 3) throw std::runtime_error("List size must be 3");
                    for (size_t i = 0; i < 3; i++) {
                        s.actuator_state[i] = l[i].cast<ActuatorStateMessageV1>();
                    }
            });

    py::class_<HunterCommonSensorState>(m_hunter_robot, "HunterCommonSensorState")
        .def(py::init<>())
        .def_property("time_stamp",
            [](const HunterCommonSensorState &s) {
                // Convert time_point to duration since epoch in milliseconds
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                        s.time_stamp.time_since_epoch());
                return duration.count();
            },
            [](HunterCommonSensorState &s, long long ms) {
                // Convert milliseconds back to time_point
                s.time_stamp = std::chrono::steady_clock::time_point(
                        std::chrono::milliseconds(ms));
            })
        .def_readwrite("bms_basic_state", &HunterCommonSensorState::bms_basic_state)
        .def_readwrite("bms_extend_state", &HunterCommonSensorState::bms_extend_state);

    // HunterRobot class
    py::class_<HunterRobot>(m_hunter_robot, "HunterRobot")
        .def(py::init<ProtocolVersion>(),
            py::arg("protocol") = ProtocolVersion::AGX_V2,
            "Constructor for HunterRobot with optional protocol version")
        .def("connect", &HunterRobot::Connect,
            py::arg("can_name"),
            "Connect to the robot using the specified CAN interface")
        .def("request_version", &HunterRobot::RequestVersion,
            py::arg("timeout_sec") = 3,
            "Request the robot's version information")
        .def("enable_commanded_mode", &HunterRobot::EnableCommandedMode,
            "To enable robot control")
        .def("activate_brake", &HunterRobot::ActivateBrake,
            "Activate the brake")
        .def("release_brake", &HunterRobot::ReleaseBrake,
            "Release the brake")
        .def("set_motion_command", &HunterRobot::SetMotionCommand,
            py::arg("linear_vel"), py::arg("angular_vel"),
            "Set the motion command for the robot")
        .def("reset_robot_state", &HunterRobot::ResetRobotState,
            "Reset the robot's state")
        .def("get_parser_protocol_version", &HunterRobot::GetParserProtocolVersion,
            "Get the parser protocol version")
        // get robot state
        .def("get_robot_state", &HunterRobot::GetRobotState,
            "Get the robot state")
        .def("get_actuator_state", &HunterRobot::GetActuatorState,
            "Get the actuator state")
        .def("get_common_sensor_state", &HunterRobot::GetCommonSensorState,
            "Get the common sensor state");
}
// clang-format on
}  // namespace westonrobot