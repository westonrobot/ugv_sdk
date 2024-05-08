/* 
 * agilex_message.hpp
 *
 * Created on 5/8/24 7:36 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef UGV_SDK_AGILEX_MESSAGE_HPP
#define UGV_SDK_AGILEX_MESSAGE_HPP

#include <pybind11/pybind11.h>

namespace westonrobot {
    void BindSystemStateMessage(pybind11::module &m);
    void BindMotionStateMessage(pybind11::module &m);
    void BindLightStateMessage(pybind11::module &m);
    void BindRcStateMessage(pybind11::module &m);
    void BindActuatorHSStateMessage(pybind11::module &m);
    void BindActuatorLSStateMessage(pybind11::module &m);
    void BindMotionModeStateMessage(pybind11::module &m);
    void BindActuatorStateMessageV1(pybind11::module &m);
}

#endif //UGV_SDK_AGILEX_MESSAGE_HPP
