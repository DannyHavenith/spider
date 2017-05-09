//
//  Copyright (C) 2017 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//

#include "esp-link/client.hpp"
#include "avr_utilities/pin_definitions.hpp"
#include "avr_utilities/devices/uart.h"

#include <util/delay.h>
#include <stdlib.h>

PIN_TYPE( B, 6) led;
PIN_TYPE( D, 3) transmit;
PIN_TYPE( B, 3) pir;


serial::uart<> uart(19200);
IMPLEMENT_UART_INTERRUPT( uart);

esp_link::client esp( uart);

void log_packet(const esp_link::packet *p)
{
    char buffer[10];
    if (!p)
    {
        esp.send( "Null\n");
    }
    else
    {
        esp.send( "command: " );
        esp.send( itoa( p->cmd, buffer, 10));
        esp.send( " value: ");
        esp.send( itoa( p->value, buffer, 10));
        esp.send( "\n");
    }
}

char *digits( uint8_t value)
{
    static char buffer[3] = {};
    buffer[1] = '0' + value % 10;
    buffer[0] = '0' + (value/10) %10;
    return buffer;
}

void log_time( const esp_link::packet *p)
{
    if (!p)
    {
        esp.send( "Null\n");
    }
    else
    {
        uint32_t seconds = p->value;
        uint8_t s = seconds % 60;
        seconds /= 60;
        uint8_t m = seconds % 60;
        seconds /= 60;
        uint8_t h = seconds % 24;
        esp.send( digits( h));
        esp.send( ":");
        esp.send( digits( m));
        esp.send( ":");
        esp.send( digits( s));
        esp.send( "\n");
    }
}
void clear_uart()
{
    while (uart.data_available()) uart.get();
}

int main(void)
{
    make_output( led);
    set( led);

    // get startup logging of the uart out of the way.
    _delay_ms( 100); // wait for an eternity.
    clear_uart();    // then clear everything received on uart.

    uart.send("sending sync2\n");
    while (not esp.sync()) toggle( led);
    uart.send( "ready syncing\n");

    for(;;)
    {
        const esp_link::packet *p = 0;
        esp.execute( esp_link::get_time);
        p = esp.receive();
        log_time( p);
        _delay_ms( 1000);
        toggle( led);
    }
}
