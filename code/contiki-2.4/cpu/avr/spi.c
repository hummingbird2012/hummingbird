/*
 * Copyright (c) 2007, Swedish Institute of Computer Science
 * All rights reserved. 
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met: 
 * 1. Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 * 3. Neither the name of the Institute nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND 
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
 * SUCH DAMAGE. 
 *
 * @(#)$Id: spi.c,v 1.1 2007/01/25 18:22:55 bg- Exp $
 */

#include <avr/io.h>

#include "contiki-conf.h"

/*
 * On the Tmote sky access to I2C/SPI/UART0 must always be
 * exclusive. Set spi_busy so that interrupt handlers can check if
 * they are allowed to use the bus or not. Only the CC2420 radio needs
 * this in practice.
 * 
 */
unsigned char spi_busy = 0;

/*
 * Initialize SPI bus.
 */
void
spi_init(void)
{
  static unsigned char spi_inited = 0;

  if (spi_inited)
    return;

  /* Initalize ports for communication with SPI units. */
  /* CSN=SS and must be output when master! */  
  #if defined (__AVR_ATmega644P__)
  /*SPI Specific Initialization.*/
  /* Set SS, CLK and MOSI as output. */
  /* 
    * The DDxn bit in the DDRx Register selects the direction of this pin. If DDxn is written logic one,
    * Pxn is configured as an output pin. If DDxn is written logic zero, Pxn is configured as an input
    * pin.
  */
  printf("\n spi_init start \n");
  // 1 = Output, 0 = Input
  //CE(Chip Enable Rx or Tx), Output--PD6
  //CSN(SPI Chip Select), Output--PD4
  //SCK(SPI Clock), Output--PB7/SCL
  //MISO(SPI Data Input), Input--PB6/MISO
  //MOSI(SPI Data Output), Output--PB5/MOSI
  //IRQ, Output--None
  DDR(SSPORT) |= BV(SSPIN) | BV(CEPIN);  	//DDR(SSPORT) |= BV(CEPIN);  //chip enable, need remove this chip enable pin in the HW
  DDR(SPIPORT) |= BV(SCKPIN) | BV(MOSIPIN);
  
  /* If PORTxn is written logic one when the pin is configured as an output pin, the port pin is driven
    * high (one). If PORTxn is written logic zero when the pin is configured as an output pin, the port
    * pin is driven low (zero).
 */
  PORT(SPIPORT) |= BV(SCKPIN) | BV(MOSIPIN);		//driven to high
  PORT(SSPORT) |= BV(SSPIN) | BV(CEPIN); 		//driven to high
  
  //Enable pull-up resistors (page 74), if it is input, set PORTx means pull-up
  PORT(SSPORT) |= BV(MISOPIN); //Pulling up a pin that is grounded will cause 90uA current leak
  printf("\n PORTB=0x%x, DDRB=0x%x, PORTD:0x%x, DDRD=0x%x \n", PORT(SPIPORT), DDR(SPIPORT), PORT(SSPORT), DDR(SSPORT) );
#else
#error "\n No CPU select, spi_init() failed in spi.c \n"
#endif

  /* Enable SPI module and master operation. */
  SPCR = BV(SPE) | BV(MSTR);

  /* Enable doubled SPI speed in master mode. */
  SPSR = BV(SPI2X);  
  printf("\n spi end, SPCR=0x%x, SPSR=0x%x \n", SPCR, SPSR);
}
