//
//  Copyright (C) 2017 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
#include "client.hpp"
#include <stdlib.h>
#include "command_codes.hpp"

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
    uint8_t last_received = 0;
    while (( index < 8) && retries--)
    {
        if (!receive_byte( last_received, timeout)) return nullptr;

        while (index < buffer_size && last_received != SLIP_END)
        {
            m_buffer[index++] = last_received;
            if (!receive_byte( last_received, timeout)) return nullptr;
        }
    }

    for (uint8_t count = 0; count < index; ++count)
    {
        send_hex( m_buffer[count]);
    }
    send_hex( last_received);
    send( "\n");

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
    execute( esp_link::sync);
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
    while (m_uart->data_available())
        m_uart->get();
}

bool client::receive_byte(uint8_t& value, uint32_t timeout) ///< timeout in units of approx. 1.25 us
{
    while (--timeout && !m_uart->data_available()) /* spinlock */;
    if (timeout == 0) return false;

    value = m_uart->get();

    if (value == SLIP_ESC)
    {
        while (--timeout && !m_uart->data_available()) /* spinlock */;
        if (timeout == 0) return false;

        value = m_uart->get();
        if (value == SLIP_ESC_END) value = SLIP_END;
        if (value == SLIP_ESC_ESC) value = SLIP_ESC;
    }

    return true;
}

uint8_t client::receive_byte_w()
{
    uint8_t result = 0;
    while (!receive_byte( result))
        /* wait */
        ;

    return result;
}

void client::send_hex( uint8_t value)
{
    constexpr char digits[] = {
            '0', '1', '2', '3',
            '4', '5', '6', '7',
            '8', '9', 'a', 'b',
            'c', 'd', 'e', 'f',
    };
    m_uart->send( (uint8_t)digits[value / 16]);
    m_uart->send( (uint8_t)digits[value % 16]);
    m_uart->send( (uint8_t)' ');
}

void client::send_direct(uint8_t value)
{
    //send_hex( value);
    m_uart->send( value);
}

void client::send_byte(uint8_t value)
{
    switch (value)
    {
    case SLIP_END:
        send_direct( SLIP_ESC);
        send_direct( SLIP_ESC_END);
        break;
    case SLIP_ESC:
        send_direct( SLIP_ESC);
        send_direct( SLIP_ESC_ESC);
        break;
    default:
        send_direct( value);
    }
}

void client::crc16_add(uint8_t value)
{
    m_runningCrc ^= value;
    m_runningCrc = (m_runningCrc >> 8) | (m_runningCrc << 8);
    m_runningCrc ^= (m_runningCrc & 0xff00) << 4;
    m_runningCrc ^= (m_runningCrc >> 8) >> 4;
    m_runningCrc ^= (m_runningCrc & 0xff00) >> 5;
}

void client::send_request_header(uint16_t command, uint32_t value,
        uint16_t argcount)
{
    //clear_input();
    send_direct( SLIP_END);
    m_runningCrc = 0;
    send_binary( command);
    send_binary( argcount);
    send_binary( value);
}

void client::finalize_request()
{
    auto crc = m_runningCrc;
    send_binary( crc);
    send_direct( SLIP_END);
}

}



