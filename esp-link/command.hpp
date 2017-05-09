//
//  Copyright (C) 2017 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef ESP_LINK_COMMAND_HPP_
#define ESP_LINK_COMMAND_HPP_
#include "command_codes.hpp"
#include <stdint.h>

namespace esp_link {

template< uint16_t Command, typename... Arguments>
struct command
{
};

template< typename T>
struct return_type
{
    using type = T;
};

// tag types for return values or parameters
struct ack {};    /// return bool to indicate whether an ack package arrived
struct string {}; /// accept any string type as argument
struct callback {};

template<>
struct return_type<ack>
{
    using type = bool;
};

using sync_t     = command< commands::CMD_SYNC,     ack      ()>;
using get_time_t = command< commands::CMD_GET_TIME, uint32_t ()>;
using mqtt_subscribe_t = command< commands::CMD_MQTT_SUBSCRIBE, void ( string, uint8_t)>;
using mqtt_setup_t = command< commands::CMD_MQTT_SETUP, void ( callback, callback, callback, callback)>;

namespace {
    constexpr sync_t     sync = {};
    constexpr get_time_t get_time = {};
}
}



#endif /* ESP_LINK_COMMAND_HPP_ */
