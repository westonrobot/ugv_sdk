/**
 * @brief MAVConn device error class
 * @file device_error.hpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * libmavconn
 * Copyright 2013,2014,2015,2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#ifndef DEVICE_ERROR_HPP
#define DEVICE_ERROR_HPP

#include <string>
#include <cstring>
#include <stdexcept>

namespace wescore
{
class DeviceError : public std::runtime_error
{
  public:
    template <typename T>
    DeviceError(const char *module, T msg) : std::runtime_error(make_message(module, msg))
    {
    }

    template <typename T>
    static std::string make_message(const char *module, T msg)
    {
        std::ostringstream ss;
        ss << "DeviceError:" << module << ":" << msg_to_string(msg);
        return ss.str();
    }

    static std::string msg_to_string(const char *description)
    {
        return description;
    }

    static std::string msg_to_string(int errnum)
    {
        return std::strerror(errnum);
    }

    static std::string msg_to_string(std::system_error &err)
    {
        return err.what();
    }
};
} // namespace mavconn

#endif /* DEVICE_ERROR_HPP */
