//
//  Copyright (C) 2017 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef ESP_LINK_CLIENT_HPP_
#define ESP_LINK_CLIENT_HPP_
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
        enum {
          CMD_NULL = 0,     /**< null, mainly to prevent 0 from doing something bad */
          CMD_SYNC,         /**< Synchronize, starts the protocol */
          CMD_RESP_V,       /**< Response with a value */
          CMD_RESP_CB,      /**< Response with a callback */
          CMD_WIFI_STATUS,  /**< Get the wifi status */
          CMD_CB_ADD,       /**< Add a custom callback */
          CMD_CB_EVENTS,    /**< ??? */
          CMD_GET_TIME,     /**< Get current time in seconds since the unix epoch */
          //CMD_GET_INFO,

          CMD_MQTT_SETUP = 10, /**< Register callback functions */
          CMD_MQTT_PUBLISH,    /**< Publish MQTT topic */
          CMD_MQTT_SUBSCRIBE,  /**< Subscribe to MQTT topic */
          CMD_MQTT_LWT,        /**< Define MQTT last will */

          CMD_REST_SETUP = 20, /**< Setup REST connection */
          CMD_REST_REQUEST,    /**< Make request to REST server */
          CMD_REST_SETHEADER,  /**< Define HTML header */

          CMD_WEB_SETUP = 30,  /**< web-server setup */
          CMD_WEB_DATA,        /**< used for publishing web-server data */

          CMD_SOCKET_SETUP = 40,  /**< Setup socket connection */
          CMD_SOCKET_SEND,        /**< Send socket packet */
        }; /**< Enumeration of commands supported by esp-link, this needs to match the definition in esp-link! */

        client( serial::uart<> &uart)
        : m_uart{uart}
        {
        }

        template< typename... Arguments>
        void request( uint16_t cmd, uint32_t value, const Arguments &... args)
        {
            constexpr uint16_t argc = sizeof...( Arguments);
            clear_input();
            send_direct( SLIP_END);
            m_runningCrc = 0;
            send_binary( cmd);
            send_binary( argc);
            send_binary( value);
            auto crc = m_runningCrc;
            send_binary(crc);
            send_direct( SLIP_END);
            m_uart.send((uint8_t)'\n');
        }

    const packet* receive(uint32_t timeout = 5000000L);

    void send(const char* str);

    bool sync();

    private:

    const packet* decode_packet(const uint8_t* buffer, uint8_t) const;

        template< typename T>
        void send_binary( const T &value)
        {
            send_bytes( reinterpret_cast< const uint8_t *>( &value), sizeof value);
        }

    void send_bytes(const uint8_t* buffer, uint8_t size);

    void clear_input();

    bool receive_byte(uint8_t& value, uint32_t timeout = 100000L);

        uint8_t receive_byte_w()
        {
            uint8_t result = 0;
            while (!receive_byte( result)) /* wait */;
            return result;
        }

        void send_direct( uint8_t value)
        {
//            constexpr char digits[] = {
//                    '0', '1', '2', '3',
//                    '4', '5', '6', '7',
//                    '8', '9', 'a', 'b',
//                    'c', 'd', 'e', 'f',
//            };
//            m_uart.send( (uint8_t)digits[value / 16]);
//            m_uart.send( (uint8_t)digits[value % 16]);
//            m_uart.send( (uint8_t)' ');

            m_uart.send( value);
        }

        void send_byte( uint8_t value)
        {
            switch (value) {
            case SLIP_END:
              send_direct(SLIP_ESC);
              send_direct(SLIP_ESC_END);
              break;
            case SLIP_ESC:
              send_direct(SLIP_ESC);
              send_direct(SLIP_ESC_ESC);
              break;
            default:
              send_direct(value);
            }
        }

        void crc16_add( uint8_t value)
        {
            m_runningCrc ^= value;
            m_runningCrc = (m_runningCrc >> 8) | (m_runningCrc << 8);
            m_runningCrc ^= (m_runningCrc & 0xff00) << 4;
            m_runningCrc ^= (m_runningCrc >> 8) >> 4;
            m_runningCrc ^= (m_runningCrc & 0xff00) >> 5;
        }

        uint16_t        m_runningCrc = 0;
        serial::uart<> &m_uart;

        static constexpr uint8_t buffer_size = 20;
        uint8_t m_buffer[buffer_size];

        static constexpr uint8_t SLIP_END     = 0xC0;    /**< End of packet */
        static constexpr uint8_t SLIP_ESC     = 0xDB;    /**< Escape */
        static constexpr uint8_t SLIP_ESC_END = 0xDC;    /**< Escaped END */
        static constexpr uint8_t SLIP_ESC_ESC = 0xDE;    /**< Escaped escape*/
    };


}



#endif /* ESP_LINK_CLIENT_HPP_ */
