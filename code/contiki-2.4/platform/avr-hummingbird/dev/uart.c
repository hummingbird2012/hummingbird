/*
 *  Copyright (c) 2008  Swedish Institute of Computer Science
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \file
 *
 * \brief
 *      Handles the control of the USART for communication with the ATmega1284p
 *      for sending commands.
 *
 * \author
 *      Mike Vidales mavida404@gmail.com
 *
 */

#include <stdint.h>
#include "uart.h"
#include "dev/rs232.h"

/* Macros & Defines */
#define BUFSIZE 80

/** \brief Circular buffer structure */
typedef struct {
    volatile uint8_t head;  /**< Index to last available character in buffer. */
    volatile uint8_t tail;  /**< Index to first available character in buffer. */
    uint8_t buf[BUFSIZE];   /**< The actual buffer used for storing characters. */
} tcirc_buf;

/** \brief The RX circular buffer, for storing characters from serial port. */
tcirc_buf rxbuf;

/*---------------------------------------------------------------------------*/

/**
 *   \brief This will intialize the circular buffer head and tail of tcirc_buf struct.
 *
 *   \param cbuf Pointer to buffer to initialize.
*/
void
uart_init_circ_buf(tcirc_buf *cbuf)
{
    cbuf->head = cbuf->tail = 0;
}

/*---------------------------------------------------------------------------*/

/**
 *   \brief This will add a new character to the circular buffer.
 *
 *   \param cbuf Pointer to buffer where character will be stored.
 *   \param ch Character to store into buffer.
*/
void
uart_add_to_circ_buf(tcirc_buf *cbuf, uint8_t ch)
{
    /* Add char to buffer */
    uint8_t newhead = cbuf->head;
    newhead++;
    if (newhead >= BUFSIZE){
        newhead = 0;
    }
    if (newhead == cbuf->tail){
        /* Buffer full, quit it */
        return;
    }

    cbuf->buf[cbuf->head] = ch;
    cbuf->head = newhead;
}

/*---------------------------------------------------------------------------*/

/**
 *   \brief This will get a character from the buffer requested.
 *
 *   \param cbuf Pointer to buffer to get character from.
 *
 *   \return retval Return character from buffer.
*/
uint8_t
uart_get_from_circ_buf(tcirc_buf *cbuf)
{
    /* Get char from buffer. */
    /* Be sure to check first that there is a char in buffer. */
    uint8_t newtail = cbuf->tail;
    uint8_t retval = cbuf->buf[newtail];

    newtail++;
    if (newtail >= BUFSIZE){
        /* Rollover */
        newtail = 0;
    }
    cbuf->tail = newtail;

    return retval;
}

/*---------------------------------------------------------------------------*/

/**
 *   \brief This will clear the RX buffer.
*/
void
uart_clear_rx_buf(void)
{
    rxbuf.tail = rxbuf.head = 0;
}

/**
 *   \brief This will check for a character in the requested buffer.
 *
 *   \param cbuf Pointer to buffer to check for any characters.
 *
 *   \return True if buffer empty.
*/
uint8_t
uart_circ_buf_has_char(tcirc_buf *cbuf)
{
    /* Return true if buffer empty */
    return (cbuf->head != cbuf->tail);
}

/*---------------------------------------------------------------------------*/
uint8_t is_uart_data_ready(void)
{
    return (rxbuf.head != rxbuf.tail);
}
/**
 *   \brief This will wait for a new character from the ATmega1284p and timeout
 *   if no new character is received.
 *
 *   \retval TIMEOUT Returns if timeout has occured.
 *   \return retval Character returned upon seeing rx_char_ready()
*/
uint8_t
uart_read_char(void)
{
    /* Gets a serial char, and waits for timeout */
    uint32_t timex = 10000;
    uint8_t retval;

    while (!is_uart_data_ready()){
        if (!timex--){
            /* Timeout, return timeout */
            return TIMEOUT;
        }
    }

    retval = uart_get_from_circ_buf(&rxbuf);
    return retval;
}

int uart_isr(unsigned char c)
{
    uart_add_to_circ_buf(&rxbuf, c);

    return 0;
}

/*---------------------------------------------------------------------------*/

/**
 *   \brief Initialize UART to 38400 Baud Rate and only
 *   enable UART for transmission.
*/
void
uart_init(void)
{
     /* rs232 port1 for at command */
    rs232_init(RS232_PORT_1, USART_BAUD_4800,USART_PARITY_NONE | USART_STOP_BITS_1 | USART_DATA_BITS_8); 
    /* set input handler */
    rs232_set_input(RS232_PORT_1, uart_isr);

    uart_clear_rx_buf();
    uart_init_circ_buf(&rxbuf);
}

/*---------------------------------------------------------------------------*/

/**
 *   \brief Turn off UART for sleep mode.
*/
void
uart_deinit(void)
{
}

/*---------------------------------------------------------------------------*/

/**
 *   \brief This function builds and sends a binary command frame to the
 *   ATmega1284p.
 *
 *   \param cmd Command to send.
 *   \param payload_length Length of data to be sent with command.
 *   \param payload Pointer to data to send.
*/
void uart_send_char(uint8_t ch)
{
    rs232_send(RS232_PORT_1, ch);
}

void uart_send_frame(uint8_t *payload, uint8_t payload_length)
{
    /* Send a frame to 1284p */
    int8_t i;

    for (i=0; i<payload_length; i++)
    {
   	uart_send_char(payload[i]);
    }
}
