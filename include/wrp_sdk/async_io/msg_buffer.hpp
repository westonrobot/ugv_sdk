/**
 * @brief MAVConn message buffer class (internal)
 * @file msgbuffer.hpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * libmavconn
 * Copyright 2014,2015,2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <sys/types.h>
#include <cstdint>
#include <cstring>
#include <cassert>

namespace wescore
{

/**
 * @brief Message buffer for internal use in libmavconn
 */
struct MsgBuffer
{
    //! Maximum buffer size with padding for CRC bytes (280 + padding)
    static constexpr ssize_t MAX_PACKET_LEN = 255;
    static constexpr ssize_t MAX_SIZE = MAX_PACKET_LEN + 16;
    uint8_t data[MAX_SIZE];
    ssize_t len;
    ssize_t pos;

    MsgBuffer() : pos(0),
                  len(0)
    {
    }
    
    /**
	 * @brief Buffer constructor for send_bytes()
	 * @param[in] nbytes should be less than MAX_SIZE
	 */
    MsgBuffer(const uint8_t *bytes, ssize_t nbytes) : pos(0),
                                                      len(nbytes)
    {
        assert(0 < nbytes && nbytes < MAX_SIZE);
        std::memcpy(data, bytes, nbytes);
    }

    virtual ~MsgBuffer()
    {
        pos = 0;
        len = 0;
    }

    uint8_t *dpos()
    {
        return data + pos;
    }

    ssize_t nbytes()
    {
        return len - pos;
    }
};
} // namespace wescore
