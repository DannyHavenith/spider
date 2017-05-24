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
#include <string.h>
#include "function/function.hpp"

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

        using callback_type = function::function<void (const packet *)>;
        client( serial::uart<> &uart)
        : m_uart{&uart}
        {
        }

        /**
         * Execute a command.
         *
         * The first argument must be an instantiation of the command<> template. This instantiation should have
         * a command code as its first argument (Cmd) and then a function prototype as its second argument. The parameters
         * of this function prototype (Parameters...) determine what other arguments are expected of the caller of execute():
         * each type in Parameters... is matched with a function argument in args... Then, the type of Parameters determines how
         * the argument will be translated into data in the packet that will be sent to the esp-link serial port.
         *
         */
        template< uint16_t Cmd, typename ReturnType, typename... Parameters, typename... Arguments>
        void execute( command<Cmd, ReturnType( Parameters...)> /*ignore*/, const Arguments &... args)
        {
            static_assert( sizeof...(Parameters) <= sizeof...(Arguments), "Too few arguments provided for this command");
            static_assert( sizeof...(Parameters) >= sizeof...(Arguments), "Too many arguments provided for this command");

            constexpr uint16_t argc = send_parameter_count( tag<Parameters>{}...);
            send_request_header( Cmd, 0x142, argc);
            (void)((int[]){0, (add_parameter(tag<Parameters>{}, args),0)...});
            finalize_request();
        }

        const packet* receive(uint32_t timeout = 50000L);
        const packet* try_receive();

        void log_packet(const esp_link::packet *p);


        void send(const char* str);
        bool sync();

    private:

        template <typename T>
        struct tag {};

        // constexpr functions to determine how many parameters to send to the
        // esp-link, given the list of function parameters.
        // This is not simply the count of the function parameters, because parameters
        // of type 'string_with_extra_len' are represented as a single argument, but will
        // result in two parameters being sent to the esp-link.
        template< typename T>
        static constexpr uint16_t send_parameter_count( tag<T>)
        {
            return 1;
        }

        static constexpr uint16_t send_parameter_count( tag<string_with_extra_len>)
        {
            return 2;
        }

        template< typename Head, typename... Tail>
        static constexpr uint16_t send_parameter_count( tag<Head> head, Tail... tail)
        {
            return send_parameter_count( head) + send_parameter_count( tail...);
        }

        static constexpr uint16_t send_parameter_count()
        {
            return 0;
        }

        // metafunction to exclude arguments from parameter type deduction
        template<typename T>
        struct no_type_deduction
        {
            using type = T;
        };

        template<typename T>
        using no_type_deduction_t = typename no_type_deduction<T>::type;


        // sending parameters...
        void add_parameter_bytes(const uint8_t* data, uint16_t length);
        void add_parameter(tag<callback>, callback_type f);
        void add_parameter(tag<string>,     const char* string);
        void add_parameter(tag<string_with_extra_len>, const char* string);
        // send a parameter of any type, represented by a value of that type.
        template< typename T>
        void add_parameter( tag<T>,  no_type_deduction_t<T> value)
        {
            add_parameter( value);
        }


        template< typename T>
        void add_parameter( T value)
        {
            add_parameter_bytes(
                    reinterpret_cast<const uint8_t *>( &value),
                    sizeof( value));
        }


        uint32_t register_callback(callback_type f);

        void send_direct(uint8_t value);
        void send_byte(uint8_t value);
        void send_bytes(const uint8_t* buffer, uint8_t size);
        /// sent a value as a sequence of bytes to the esp. This will send
        /// the memory bytes of this value
        template< typename T>
        void send_binary( const T &value)
        {
            send_bytes( reinterpret_cast< const uint8_t *>( &value), sizeof value);
        }
        void send_hex( uint8_t value);

        void send_request_header(uint16_t command, uint32_t value, uint16_t argcount);
        void finalize_request();

        void clear_input();

        bool receive_byte(uint8_t& value, uint32_t timeout = 100000L);
        uint8_t receive_byte_w();

        static void crc16_add(uint8_t value, uint16_t &accumulator);

        const packet* decode_packet(const uint8_t* buffer, uint8_t size);
        const packet* check_packet(const uint8_t* buffer, uint8_t size);

        uint16_t        m_runningCrc = 0;
        serial::uart<>  *m_uart;
        static constexpr uint8_t buffer_size = 128;
        uint8_t m_buffer[buffer_size];
        uint8_t m_buffer_index = 0;
        bool    m_last_was_esc = false;
        bool    m_syncing = false;

        static constexpr uint8_t callbacks_size = 8;
        callback_type m_callbacks[callbacks_size];
    };

}

#endif /* ESP_LINK_CLIENT_HPP_ */
