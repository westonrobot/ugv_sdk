/* 
 * agilex_message.cpp
 *
 * Created on 5/8/24 7:34 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <pybind11/pybind11.h>

#include "agilex_message.hpp"
#include "ugv_sdk/details/interface/agilex_message.h"

namespace py = pybind11;
namespace westonrobot {
    // clang-format off
    void BindSystemStateMessage(py::module &m) {
        py::enum_<AgxVehicleState>(m, "AgxVehicleState")
                .value("VEHICLE_STATE_NORMAL", AgxVehicleState::VEHICLE_STATE_NORMAL)
                .value("VEHICLE_STATE_ESTOP", AgxVehicleState::VEHICLE_STATE_ESTOP)
                .value("VEHICLE_STATE_EXCEPTION", AgxVehicleState::VEHICLE_STATE_EXCEPTION)
                .export_values();

        py::enum_<AgxControlMode>(m, "AgxControlMode")
                .value("CONTROL_MODE_STANDBY", AgxControlMode::CONTROL_MODE_STANDBY)
                .value("CONTROL_MODE_CAN", AgxControlMode::CONTROL_MODE_CAN)
                .value("CONTROL_MODE_UART", AgxControlMode::CONTROL_MODE_UART)
                .value("CONTROL_MODE_RC", AgxControlMode::CONTROL_MODE_RC)
                .export_values();

        py::class_<SystemStateMessage>(m, "SystemStateMessage")
                .def(py::init<>())
                .def_readwrite("vehicle_state", &SystemStateMessage::vehicle_state)
                .def_readwrite("control_mode", &SystemStateMessage::control_mode)
                .def_readwrite("battery_voltage", &SystemStateMessage::battery_voltage)
                .def_readwrite("error_code", &SystemStateMessage::error_code);
    }

    void BindMotionStateMessage(pybind11::module &m) {
        py::class_<MotionStateMessage>(m, "MotionStateMessage")
                .def(py::init<>())
                .def_readwrite("linear_velocity", &MotionStateMessage::linear_velocity)
                .def_readwrite("angular_velocity", &MotionStateMessage::angular_velocity)
                .def_readwrite("lateral_velocity", &MotionStateMessage::lateral_velocity)
                .def_readwrite("steering_angle", &MotionStateMessage::steering_angle);
    }

    void BindLightStateMessage(pybind11::module &m) {
        py::enum_<AgxLightMode>(m, "AgxLightMode")
                .value("CONST_OFF", AgxLightMode::CONST_OFF)
                .value("CONST_ON", AgxLightMode::CONST_ON)
                .value("BREATH", AgxLightMode::BREATH)
                .value("CUSTOM", AgxLightMode::CUSTOM)
                .export_values();

        py::class_<AgxLightOperation>(m, "AgxLightOperation")
                .def(py::init<>())
                .def_readwrite("mode", &AgxLightOperation::mode)
                .def_readwrite("custom_value", &AgxLightOperation::custom_value);

        py::class_<LightStateMessage>(m, "LightStateMessage")
                .def(py::init<>())
                .def_readwrite("enable_cmd_ctrl", &LightStateMessage::enable_cmd_ctrl)
                .def_readwrite("front_light", &LightStateMessage::front_light)
                .def_readwrite("rear_light", &LightStateMessage::rear_light);
    }

    void BindRcStateMessage(pybind11::module &m) {
        py::enum_<AgxRcSwitchState>(m, "AgxRcSwitchState")
                .value("RC_SWITCH_UP", AgxRcSwitchState::RC_SWITCH_UP)
                .value("RC_SWITCH_MIDDLE", AgxRcSwitchState::RC_SWITCH_MIDDLE)
                .value("RC_SWITCH_DOWN", AgxRcSwitchState::RC_SWITCH_DOWN)
                .export_values();

        py::class_<RcStateMessage>(m, "RcStateMessage")
                .def(py::init<>())
                .def_readwrite("swa", &RcStateMessage::swa)
                .def_readwrite("swb", &RcStateMessage::swb)
                .def_readwrite("swc", &RcStateMessage::swc)
                .def_readwrite("swd", &RcStateMessage::swd)
                .def_readwrite("stick_right_v", &RcStateMessage::stick_right_v)
                .def_readwrite("stick_right_h", &RcStateMessage::stick_right_h)
                .def_readwrite("stick_left_v", &RcStateMessage::stick_left_v)
                .def_readwrite("stick_left_h", &RcStateMessage::stick_left_h)
                .def_readwrite("var_a", &RcStateMessage::var_a);
    }
    // clang-format on
}
