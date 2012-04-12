/*
 * Copyright (c) 2006, Technical University of Munich
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
 * This file is part of the Contiki operating system.
 *
 * @(#)$$
 */
#define DEBUG 1

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#include <stdio.h>
#include <string.h>

#include "loader/symbols-def.h"
#include "loader/symtab.h"

#include "contiki.h"
#include "rs232_atmega644.h"
#include "dev_config.h"
#include "dev/radio/nrf24l01.h"
#include "dev/rs232.h"
#include "net/rime.h"

//#include "mac/frame802154.h"
#include "sicslowpan.h"
//#include "net/uip-netif.h"
#include "sicslowmac.h"

FILE *g_dbg_file = NULL;

PROCINIT(&etimer_process);

/*-------------------------Low level initialization------------------------*/
/*------Done in a subroutine to keep main routine stack usage small--------*/
void initialize(void)
{
  /* rs232 port0 for debugging */
  rs232_init(RS232_PORT_0, USART_BAUD_4800,USART_PARITY_NONE | USART_STOP_BITS_1 | USART_DATA_BITS_8);
  /* Redirect stdout to second port */
  rs232_redirect_stdout(RS232_PORT_0);

  clock_init();
  
  PRINTF("\n*******Booting %s*******\n",CONTIKI_VERSION_STRING);

 /* Initialize process subsystem */
  process_init();
 
 /* Start radio and radio receive process */
 #ifdef USE_TX_MODE
  nrf24l01_init(RF_TX_MODE);
 #else
  nrf24l01_init(RF_RX_MODE);
 #endif
  
  rime_init(sicslowmac_init(&nrf24l01_driver));

/* Set addresses BEFORE starting tcpip process */
  rimeaddr_t addr;
  memset(&addr, 0, sizeof(rimeaddr_t));
  
  /*ENTER_CRITICAL_REGION();
  eeprom_read_block ((void *)&addr.u8,  &mac_address, 8);
  LEAVE_CRITICAL_REGION();*/
  /*addr.u8[0] = random_rand() % 256;
  addr.u8[1] = addr.u8[0] + 1;
  rimeaddr_set_node_addr(&addr); 
  PRINTF("MAC address %x:%x\n",addr.u8[0],addr.u8[1]);*/

  /* Register initial processes */
  procinit_init(); 

  /* Autostart processes */
  autostart_start(autostart_processes);
}

/*-------------------------------------------------------------------------*/
/*------------------------- Main Scheduler loop----------------------------*/
/*-------------------------------------------------------------------------*/
int
main(void)
{
  initialize();
  while(1) {   
    process_run();
  }
  return 0;
}
