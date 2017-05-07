//
//  Copyright (C) 2017 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
#include "client.hpp"
#include <stdlib.h>

namespace esp_link
{

const esp_link::packet* client::receive(uint32_t timeout)
{
    static char buffer[10];
    uint8_t index = 0;
    //            uint8_t last_received = 0;
    //            while ((last_received = receive_byte_w()) != SLIP_END)
    //            {
    //                send("skipped:");
    //                send( itoa( last_received, buffer, 10));
    //                send("\n");
    //            }
    uint8_t retries = 3;
    while (index == 0 && retries--)
    {
        uint8_t last_received = 0;
        if (!receive_byte( last_received, timeout)) return nullptr;

        while (index < buffer_size && last_received != SLIP_END)
        {
            m_buffer[index++] = last_received;
            if (!receive_byte( last_received, timeout)) return nullptr;
        }
    }
    if (index == buffer_size || index < 8)
    {
        send( "index: ");
        send( itoa( index, buffer, 10));
        send( "\n");
        return nullptr;
    }
    return decode_packet( m_buffer, index);
}

void client::send(const char* str)
{
    while (*str)
        send_byte( static_cast<uint8_t>( *str++));
}

bool client::sync()
{
    send_direct( SLIP_END);
    const packet* p = nullptr;
    request( CMD_SYNC, 0x0142);
    p = receive();
    return p != nullptr;
}

const esp_link::packet* client::decode_packet(const uint8_t* buffer,
        uint8_t) const
{
    return reinterpret_cast<const packet*>( buffer);
}

void client::send_bytes(const uint8_t* buffer, uint8_t size)
{
    while (size)
    {
        crc16_add( *buffer);
        send_byte( *buffer);
        --size;
        ++buffer;
    }
}

void client::clear_input()
{
    while (m_uart.data_available())
        m_uart.get();
}

bool client::receive_byte(uint8_t& value, uint32_t timeout) ///< timeout in units of approx. 1.25 us
{
    while (--timeout && !m_uart.data_available())
        /* spin lock */
        ;

    if (!m_uart.data_available()) return false;

    uint8_t newValue = m_uart.get();
    // if we previously received an escape...
    if (value == SLIP_ESC)
    {
        if (newValue == SLIP_ESC_END) newValue = SLIP_END;

        if (newValue == SLIP_ESC_ESC) newValue = SLIP_ESC;
    }
    value = newValue;
    return newValue != SLIP_ESC;
}

}



