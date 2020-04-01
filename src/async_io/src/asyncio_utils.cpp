/* 
 * asyncio_utils.cpp
 * 
 * Created on: Jul 24, 2019 01:46
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "asyncio_utils.hpp"

namespace wescore
{
void url_parse_host(std::string host,
                    std::string &host_out, int &port_out,
                    const std::string def_host, const int def_port)
{
    std::string port;

    auto sep_it = std::find(host.begin(), host.end(), ':');
    if (sep_it == host.end())
    {
        // host
        if (!host.empty())
        {
            host_out = host;
            port_out = def_port;
        }
        else
        {
            host_out = def_host;
            port_out = def_port;
        }
        return;
    }

    if (sep_it == host.begin())
    {
        // :port
        host_out = def_host;
    }
    else
    {
        // host:port
        host_out.assign(host.begin(), sep_it);
    }

    port.assign(sep_it + 1, host.end());
    port_out = std::stoi(port);
}

/**
 * Parse ?ids=sid,cid
 */
void url_parse_query(std::string query)
{
    const std::string ids_end("ids=");
    std::string sys, comp;

    if (query.empty())
        return;

    auto ids_it = std::search(query.begin(), query.end(),
                              ids_end.begin(), ids_end.end());
    if (ids_it == query.end())
    {
        std::cerr << "URL: unknown query arguments" << std::endl;
        return;
    }

    std::advance(ids_it, ids_end.length());
    auto comma_it = std::find(ids_it, query.end(), ',');
    if (comma_it == query.end())
    {
        std::cerr << "URL: no comma in ids= query" << std::endl;
        return;
    }

    sys.assign(ids_it, comma_it);
    comp.assign(comma_it + 1, query.end());
}
} // namespace wescore