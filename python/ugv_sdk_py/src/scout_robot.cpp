/*
 * scout_robot.cpp
 *
 * Created on 5/8/24 2:57 PM
 * Description:
 *
 * Copyright (c) 2024 Weston Robot Pte. Ltd.
 */
#include "ugv_sdk_py/scout_robot.hpp"

#include "ugv_sdk/mobile_robot/scout_robot.hpp"

namespace py = pybind11;

namespace westonrobot {

// clang-format off
void BindScoutRobot(pybind11::module &m) {
    py::module_ m_scout_robot = m.def_submodule("scout_robot");
    m_scout_robot.doc() = "Python bindings for handling a Scout robot";

    py::class_<ScoutCoreState>(m_scout_robot, "ScoutCoreState")
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

    py::class_<ScoutActuatorState>(m_scout_robot, "ScoutActuatorState")
        .def(py::init<>())
        .def_property("time_stamp",
        [](const ScoutActuatorState &s) {
                // Convert time_point to duration since epoch in milliseconds
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                        s.time_stamp.time_since_epoch());
                return duration.count();
            },
            [](ScoutActuatorState &s, long long ms) {
                // Convert milliseconds back to time_point
                s.time_stamp = std::chrono::steady_clock::time_point(
                        std::chrono::milliseconds(ms));
            })
        .def_property("actuator_hs_state",
            [](const ScoutActuatorState &s) {
                // Convert C-style array to Python list
                return py::list(py::make_iterator(std::begin(s.actuator_hs_state), std::end(s.actuator_hs_state)));
            },
            [](ScoutActuatorState &s, py::list l) {
                // Convert Python list back to C-style array
                if (l.size() != 4) throw std::runtime_error("List size must be 4");
                    for (size_t i = 0; i < 4; ++i) {
                        s.actuator_hs_state[i] = l[i].cast<ActuatorHSStateMessage>();
                    }
            })
        .def_property("actuator_ls_state",
            [](const ScoutActuatorState &s) {
                // Convert C-style array to Python list
                return py::list(py::make_iterator(std::begin(s.actuator_ls_state), std::end(s.actuator_ls_state)));
            },
            [](ScoutActuatorState &s, py::list l) {
                // Convert Python list back to C-style array
                if (l.size() != 4) throw std::runtime_error("List size must be 4");
                for (size_t i = 0; i < 4; ++i) {
                s.actuator_ls_state[i] = l[i].cast<ActuatorLSStateMessage>();
            }
        })
        .def_property("actuator_state",
            [](const ScoutActuatorState &s) {
                // Convert C-style array to Python list
                return py::list(py::make_iterator(std::begin(s.actuator_state), std::end(s.actuator_state)));
            },
            [](ScoutActuatorState &s, py::list l) {
                // Convert Python list back to C-style array
                if (l.size() != 4) throw std::runtime_error("List size must be 4");
                for (size_t i = 0; i < 4; ++i) {
                s.actuator_state[i] = l[i].cast<ActuatorStateMessageV1>();
            }
        });

    py::class_<ScoutCommonSensorState>(m_scout_robot, "ScoutCommonSensorState")
        .def(py::init<>())
        .def_property("time_stamp",
        [](const ScoutCommonSensorState &s) {
                // Convert time_point to duration since epoch in milliseconds
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                        s.time_stamp.time_since_epoch());
                return duration.count();
            },
            [](ScoutCommonSensorState &s, long long ms) {
                // Convert milliseconds back to time_point
                s.time_stamp = std::chrono::steady_clock::time_point(
                        std::chrono::milliseconds(ms));
            })
        .def_readwrite("bms_basic_state", &ScoutCommonSensorState::bms_basic_state);

    // ScoutRobot class
    py::class_<ScoutRobot>(m_scout_robot, "ScoutRobot")
        .def(py::init<ProtocolVersion, bool>(),
            py::arg("protocol") = ProtocolVersion::AGX_V2,
            py::arg("is_mini_model") = false,
            "Constructor for ScoutRobot with optional protocol and model type")
        .def("connect", &ScoutRobot::Connect,
            py::arg("can_name"),
            "Connects the robot to the specified CAN interface.")
        .def("enable_commanded_mode", &ScoutRobot::EnableCommandedMode,
             "To enable robot control")
        .def("request_version", &ScoutRobot::RequestVersion,
            py::arg("timeout_sec") = 3,
            "Request the robot version")
        .def("set_motion_command", &ScoutRobot::SetMotionCommand,
            py::arg("linear_vel"), py::arg("angular_vel"),
            "Set the motion command for the robot")
        .def("set_light_command", &ScoutRobot::SetLightCommand,
            py::arg("f_mode"), py::arg("f_value"),
            py::arg("r_mode"), py::arg("r_value"),
            "Set the light command for the robot")
        .def("disable_light_control", &ScoutRobot::DisableLightControl,
            "Disable light control")
        .def("reset_robot_state", &ScoutRobot::ResetRobotState,
            "Reset the robot state")
        .def("get_parser_protocol_version", &ScoutRobot::GetParserProtocolVersion,
            "Get the parser protocol version")
        // get robot state
        .def("get_robot_state", &ScoutRobot::GetRobotState, "Get the robot state")
        .def("get_actuator_state", &ScoutRobot::GetActuatorState, "Get the actuator state")
        .def("get_common_sensor_state", &ScoutRobot::GetCommonSensorState, "Get the common sensor state");

        // ScoutMiniOmniRobot class
    py::class_<ScoutMiniOmniRobot, ScoutRobot>(m_scout_robot, "ScoutMiniOmniRobot")
        .def(py::init<ProtocolVersion>(),
            py::arg("protocol") = ProtocolVersion::AGX_V2,
            "Constructor for ScoutMiniOmniRobot with optional protocol")
        .def("set_motion_command", &ScoutOmniInterface::SetMotionCommand,
        py::arg("linear_vel"), py::arg("angular_vel"), py::arg("lateral_velocity"),
        "Set the motion command for the robot");
}
// clang-format on
}  // namespace westonrobot
