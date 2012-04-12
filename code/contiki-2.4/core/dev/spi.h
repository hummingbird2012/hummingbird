/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
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
 * $Id: spi.h,v 1.8 2010/06/23 10:15:28 joxe Exp $
 */

/**
 * \file
 *         Basic SPI macros
 * \author
 *         Joakim Eriksson <joakime@sics.se>
 *         Niclas Finne <nfi@sics.se>
 */

#ifndef __SPI_H__
#define __SPI_H__

#include "contiki-conf.h"

extern unsigned char spi_busy;

void spi_init(void);

/* Write one character to SPI */
#define SPI_WRITE(data)                         \
  do {                                          \
     SPI_TRANSFER(data);	\
  } while(0)

/* Write one character to SPI - will not wait for end
   useful for multiple writes with wait after final */
#define SPI_WRITE_FAST(data)                    \
  do {                                          \
    SPI_TRANSFER_WRITE(data);		\
  } while(0)
  
/* Read one character from SPI */
#define SPI_READ(data)   \
  do {                   \
     data = SPI_TRANSFER(0);	\
  } while(0)

/* Read register by SPI */
#define SPI_READ_REG(addr, data)   \
  do {                   \
    SPI_ENABLE();		\
    SPI_WRITE(addr);	\
    data = (uint8_t)SPI_WRITE(0);	\
    SPI_DISABLE();		\
  } while(0)

/* Write register by SPI */
#define SPI_WRITE_REG(addr, data)   \
  do {                   \
    SPI_ENABLE();		\
    SPI_WRITE(addr);	\
    SPI_WRITE(data);	\
    SPI_DISABLE();		\
  } while(0) 

/* Flush the SPI read register */
#define SPI_FLUSH() \
  do {              \
     SPI_TRANSFER_READ();	\
  } while(0)

/* Write to RAM  */
#define SPI_WRITE_BUF(addr,buffer,count)                 \
  do {                                                       \
    uint8_t i;                                               \
    SPI_ENABLE();                                     \
    SPI_WRITE_FAST(addr);                   \
    for(i = 0; i < (count); i++) {                           \
      SPI_WRITE_FAST(((uint8_t*)(buffer))[i]);               \
    }                                                        \
    SPI_TRANSFER_WAIT();                                   \
    SPI_DISABLE();                                    \
  } while(0)

/* Read from RAM */
#define SPI_READ_BUF(addr,buffer,count)                    \
  do {                                                       \
    uint8_t i;                                               \
    SPI_ENABLE();                                     \
    SPI_WRITE(addr);                        \
    for(i = 0; i < (count); i++) {                           \
      SPI_READ(((uint8_t*)(buffer))[i]);                     \
    }                                                        \
    SPI_DISABLE();                                    \
  } while(0)

#endif /* __SPI_H__ */
