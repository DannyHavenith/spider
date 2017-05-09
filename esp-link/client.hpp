//
//  Copyright (C) 2017 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef ESP_LINK_CLIENT_HPP_
#define ESP_LINK_CLIENT_HPP_
#include "command.hpp"

#include <stdint.h>
#include <avr_utilities/devices/uart.h>

namespace esp_link
{
    struct packet
    {
        uint16_t cmd;            /**< Command to execute */
        uint16_t argc;           /**< Number of arguments */
        uint32_t value;          /**< Callback to invoke, NULL if none; or response value */
        uint8_t  args[0];        /**< Arguments */
    };

    class client
    {
    public:
        client( serial::uart<> &uart)
        : m_uart{&uart}
        {
        }

        template< uint16_t Cmd, typename ReturnType, typename... Parameters, typename... Arguments>
        void execute( command<Cmd, ReturnType( Parameters...)> /*ignore*/, const Arguments &... args)
        {
            static_assert( sizeof...(Parameters) <= sizeof...(Arguments), "Too many arguments provided for this command");
            static_assert( sizeof...(Parameters) >= sizeof...(Arguments), "Too few arguments provided for this command");

            constexpr uint16_t argc = sizeof...( Parameters);
            send_request_header( Cmd, 0x142, argc);
            (void)((int[]){(add_parameter<Parameters>(args),0)...});
            finalize_request();
        }

        template< typename Par, typename Arg>
        void add_parameter( const Arg &)
        {
        }

        const packet* receive(uint32_t timeout = 5000000L);
        void send(const char* str);
        bool sync();

    private:

        template< typename T>
        void send_binary( const T &value)
        {
            send_bytes( reinterpret_cast< const uint8_t *>( &value), sizeof value);
        }

        void send_hex( uint8_t value);
        void send_request_header(uint16_t command, uint32_t value, uint16_t argcount);
        void finalize_request();
        const packet* decode_packet(const uint8_t* buffer, uint8_t) const;
        void send_bytes(const uint8_t* buffer, uint8_t size);
        void clear_input();
        bool receive_byte(uint8_t& value, uint32_t timeout = 100000L);
        uint8_t receive_byte_w();
        void send_direct(uint8_t value);
        void send_byte(uint8_t value);
        void crc16_add(uint8_t value);

        uint16_t        m_runningCrc = 0;
        serial::uart<>  *m_uart;

        static constexpr uint8_t buffer_size = 20;
        uint8_t m_buffer[buffer_size];

        static constexpr uint8_t SLIP_END     = 0xC0;    /**< End of packet */
        static constexpr uint8_t SLIP_ESC     = 0xDB;    /**< Escape */
        static constexpr uint8_t SLIP_ESC_END = 0xDC;    /**< Escaped END */
        static constexpr uint8_t SLIP_ESC_ESC = 0xDE;    /**< Escaped escape*/
    };

}

#endif /* ESP_LINK_CLIENT_HPP_ */
