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

namespace
{
    constexpr uint8_t SLIP_END     = 0xC0;    /**< End of packet */
    constexpr uint8_t SLIP_ESC     = 0xDB;    /**< Escape */
    constexpr uint8_t SLIP_ESC_END = 0xDC;    /**< Escaped END */
    constexpr uint8_t SLIP_ESC_ESC = 0xDD;    /**< Escaped escape*/
}

namespace esp_link
{

/**
 * Wait a limited time for a packet to arrive and return
 * either nullptr if no packet arrived or a pointer to a
 * successfully received packet
 *
 * timeout is specified in an undefined unit (loop count)
 * that will normally be in the 100Khz range.
 *
 */
const esp_link::packet* client::receive(uint32_t timeout)
{
    while (timeout--)
    {
        auto p = try_receive();
        if (p) return p;
    }

    return nullptr;
}

/**
 * Listen for incoming packets and return immediately if no
 * packet is arriving.
 *
 */
const packet* client::try_receive()
{
    while (m_uart->data_available())
    {
        uint8_t lastByte = m_uart->read();
        if (lastByte == SLIP_ESC)
        {
            m_last_was_esc = true;
            continue;
        }

        if (lastByte == SLIP_END)
        {
            auto packet = decode_packet( m_buffer, m_buffer_index);
            m_buffer_index = 0;
            m_last_was_esc = false;
            return packet;
        }

        if (m_last_was_esc)
        {
            m_last_was_esc = false;
            if (lastByte == SLIP_ESC_ESC)
            {
                lastByte = SLIP_ESC;
            }
            else if (lastByte == SLIP_ESC_END)
            {
                lastByte = SLIP_END;
            }
        }

        if (m_buffer_index <= buffer_size)
        {
            m_buffer[m_buffer_index++] = lastByte;
        }
    }
    return nullptr;
}

/**
 * Send a null-terminated character string.
 */
void client::send(const char* str)
{
    while (*str)
        send_byte( static_cast<uint8_t>( *str++));
}

/**
 * Synchronize with the esp-link.
 *
 * Returns false if no response to the sync packet
 * was recieved within a sane timeout.
 */
bool client::sync()
{
    const packet* p = nullptr;

    // never recurse
    if (!m_syncing)
    {
        send( "sync\n");
        m_syncing = true;
        clear_input();
        send_direct( SLIP_END);
        clear_input();
        execute( esp_link::sync);
        while ((p = receive()))
        {
            if (p->cmd ==  commands::CMD_RESP_V)
            {
                m_syncing = false;
                return true;
            }
        }

        m_syncing = false;
    }
    return false;
}

/**
 * Decode a packet from a sequence of bytes received.
 */
const esp_link::packet* client::decode_packet(
        const uint8_t*  buffer,
        uint8_t         size)
{
    auto p = check_packet( buffer, size);
    if (p)
    {
        if ( p->cmd == commands::CMD_SYNC)
        {
            sync();
            return nullptr;
        }
        else if( p->cmd == commands::CMD_RESP_CB)
        {
            if (p->value < callbacks_size && m_callbacks[p->value])
            {
                m_callbacks[p->value]( p);
            }
            return nullptr;
        }
    }
    return p;
}

/**
 * Send a textual representation of a received packet to the serial port.
 */
void client::log_packet(const esp_link::packet *p)
{
    char buffer[10];
    if (!p)
    {
        send( "Null\n");
    }
    else
    {
        send( "command: " );
        send( itoa( p->cmd, buffer, 10));
        send( " value: ");
        send( itoa( p->value, buffer, 10));
        send( "\n");
    }
}

/**
 * Check a sequence of bytes for a correct checksum.
 */
const esp_link::packet* client::check_packet(
        const uint8_t*  buffer,
        uint8_t         size)
{

    if (size < 8) return nullptr;

    uint16_t crc = 0;
    const uint8_t *data = buffer;

    while (size-- > 2)
    {
        crc16_add( *data++, crc);
    }
    if (*reinterpret_cast<const uint16_t*>( data) != crc)
    {
        send("check failed\n");
        return nullptr;
    }
    else
    {
        send("got packet\n");
        return reinterpret_cast<const packet*>( buffer);
    }
}

/**
 * Send a sequence of bytes indicated by a pointer to the start of
 * the sequence and the sequence size.
 */
void client::send_bytes(const uint8_t* buffer, uint8_t size)
{
    while (size)
    {
        crc16_add( *buffer, m_runningCrc);
        send_byte( *buffer);
        --size;
        ++buffer;
    }
}

/**
 * Clear the input buffer of the uart.
 */
void client::clear_input()
{
    while (m_uart->data_available())
        m_uart->get();
}

/**
 * Wait a limited time for an incoming byte on the serial port.
 */
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

/**
 * Wait, potentially forever, until a byte arrives at the serial port
 * and return that bytes value.
 */
uint8_t client::receive_byte_w()
{
    uint8_t result = 0;
    while (!receive_byte( result))
        /* wait */
        ;

    return result;
}

/**
 * Send a byte value as a hex string.
 * Useful for debugging.
 */
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

/**
 * Send a byte directly to the uart, without SLIP
 * ESCAPEing.
 */
void client::send_direct(uint8_t value)
{
    //send_hex( value);
    m_uart->send( value);
}

/**
 * Send a byte to the uart, but perform proper
 * escaping of SLIP_END characters in the data.
 */
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

/**
 * Calculate the next crc16 value given an accumulator and a new value.
 */
void client::crc16_add(uint8_t value, uint16_t &accumulator)
{
    accumulator ^= value;
    accumulator  = (accumulator >> 8) | (accumulator << 8);
    accumulator ^= (accumulator & 0xff00) << 4;
    accumulator ^= (accumulator >> 8) >> 4;
    accumulator ^= (accumulator & 0xff00) >> 5;
}

/**
 * Send the header for a request (command) to the esp-link.
 *
 * A request consists of a request header, parameters and a two
 * byte crc code. This function sends the header.
 */
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

/**
 * Send the last bytes of a request.
 *
 * This means sending the crc and a SLIP_END
 * character.
 */
void client::finalize_request()
{
    // make a copy of the running crc because
    // send_binary() will change it.
    auto crc = m_runningCrc;
    send_binary( crc);
    send_direct( SLIP_END);
}

/**
 * Send a parameter to the esp-link.
 * A parameter is always sent as a 2-byte size value, followed by the bytes
 * of the actual parameter.
 *
 * The size is, as all binary numbers, transmitted as little endian.
 */
void client::add_parameter_bytes(const uint8_t* data, uint16_t length)
{
    send_binary( length);
    send_bytes( data, length);
    uint8_t pad = (4 - (length & 3)) & 3;
    while (pad--)
    {
        crc16_add( 0, m_runningCrc);
        send_direct( 0);
    }
}

/**
 * Send a callback parameter to the esp-link.
 * Callbacks are represented by 32-bit integers.
 */
void client::add_parameter(tag<callback>, client::callback_type func)
{
    add_parameter( register_callback( func));
}

/**
 * Send a string parameter to the esp-link.
 * This overload accepts a const char * for the string.
 */
void client::add_parameter(tag<string>, const char* string)
{
    add_parameter_bytes( reinterpret_cast<const uint8_t*>( string),
            strlen( string));
}

/**
 * This implements a special case where some strings are sent normally
 * (i.e. a 16-bit size followed by the bytes of the string), but with an added
 * parameter that again holds the size of the string.
 *
 * This is notably the case for MQTT::publish commands, where the second string
 * must be sent as follows:
 * (normal string)
 * <size> <char> <char> <char>... (<size> chars, padded to a multiple of 4)
 * 02 00 <size> (the size again, as a 2-byte parameter)
 */
void client::add_parameter(tag<string_with_extra_len>, const char* string)
{
    uint16_t len = strlen( string);
    add_parameter_bytes( reinterpret_cast<const uint8_t*>( string), len);
    add_parameter( len);
}

uint32_t client::register_callback(callback_type f)
{
    if (!f) return callbacks_size;

    for (uint8_t count = 0; count < callbacks_size; ++count)
    {
        if (!m_callbacks[count])
        {
            m_callbacks[count] = f;
            return count;
        }
    }

    return callbacks_size;
}

}
