/* 
 * agilex_message.cpp
 *
 * Created on 5/8/24 7:34 PM
 * Description:
 *
 * Copyright (c) 2024 Weston Robot Pte. Ltd.
 */

#include <pybind11/pybind11.h>

#include "ugv_sdk_py/agilex_message.hpp"
#include "ugv_sdk/details/interface/agilex_message.h"
#include "ugv_sdk/details/parser_base.hpp"

namespace py = pybind11;
namespace westonrobot {
    // clang-format off
    void BindProtocolVersion(pybind11::module &m) {
        py::enum_<ProtocolVersion>(m, "ProtocolVersion")
                .value("AGX_V1", ProtocolVersion::AGX_V1)
                .value("AGX_V2", ProtocolVersion::AGX_V2)
                .export_values();
    }

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

    void BindActuatorHSStateMessage(pybind11::module &m) {
        py::class_<ActuatorHSStateMessage>(m, "ActuatorHSStateMessage")
                .def(py::init<>())
                .def_readwrite("motor_id", &ActuatorHSStateMessage::motor_id)
                .def_readwrite("rpm", &ActuatorHSStateMessage::rpm)
                .def_readwrite("current", &ActuatorHSStateMessage::current)
                .def_readwrite("pulse_count", &ActuatorHSStateMessage::pulse_count);
    }

    void BindActuatorLSStateMessage(pybind11::module &m) {
        py::class_<ActuatorLSStateMessage>(m, "ActuatorLSStateMessage")
                .def(py::init<>())
                .def_readwrite("motor_id", &ActuatorLSStateMessage::motor_id)
                .def_readwrite("driver_voltage", &ActuatorLSStateMessage::driver_voltage)
                .def_readwrite("driver_temp", &ActuatorLSStateMessage::driver_temp)
                .def_readwrite("motor_temp", &ActuatorLSStateMessage::motor_temp)
                .def_readwrite("driver_state", &ActuatorLSStateMessage::driver_state);
    }

    void BindActuatorStateMessageV1(pybind11::module &m) {
        py::class_<ActuatorStateMessageV1>(m, "ActuatorStateMessageV1")
                .def(py::init<>())
                .def_readwrite("motor_id", &ActuatorStateMessageV1::motor_id)
                .def_readwrite("current", &ActuatorStateMessageV1::current)
                .def_readwrite("rpm", &ActuatorStateMessageV1::rpm)
                .def_readwrite("driver_temp", &ActuatorStateMessageV1::driver_temp)
                .def_readwrite("motor_temp", &ActuatorStateMessageV1::motor_temp);
    }

    void BindBmsBasicMessage(pybind11::module &m) {
        py::class_<BmsBasicMessage>(m, "BmsBasicMessage")
                .def(py::init<>())
                .def_readwrite("battery_soc", &BmsBasicMessage::battery_soc)
                .def_readwrite("battery_soh", &BmsBasicMessage::battery_soh)
                .def_readwrite("voltage", &BmsBasicMessage::voltage)
                .def_readwrite("current", &BmsBasicMessage::current)
                .def_readwrite("temperature", &BmsBasicMessage::temperature);
    }
    // clang-format on
}
