/**
 * @file ranger_robot.hpp
 * @brief 
 * @date 28-05-2024
 * 
 * ranger_robot submodule for ugv_sdk_py
 * 
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */
#ifndef UGV_SDK_PY_RANGER_ROBOT_HPP
#define UGV_SDK_PY_RANGER_ROBOT_HPP
#include <pybind11/pybind11.h>

namespace westonrobot {
    void BindRangerRobot(pybind11::module &m);
}


#endif /* UGV_SDK_PY_RANGER_ROBOT_HPP */
