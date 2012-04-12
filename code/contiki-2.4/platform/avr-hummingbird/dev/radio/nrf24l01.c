/*
 * Copyright (c) 2011, M2M SIG
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
 *
 * @(#)$Id: nrf24l01.c,v 1.1 2011/08/20 16:17:00 nick Exp $   */
/**
 *  \brief This module contains radio driver code for the nRF24L01. It is modified to use the contiki core MAC layer.
 *
 *  \author 
 *          Nicolas Chang <zhbsh.zhbsh@gmail.com>
 *
*/
/**
 *  \file
 *  This file contains "barebones" radio driver code for use with the
 *  contiki core MAC layer.
 *
 */


#include <stdio.h>
#include <string.h>

#include "contiki.h"

#if defined(__AVR_ATmega644P__)
#include <avr/io.h>
#else
#error "NRF24L01, wrong CPU configuration"
#endif
#include <avr/pgmspace.h>

//this delay is implemented by loop, not interrupt, so this can be used in the ISR
#include "delay.h"
#include "spi.h"
#include "nrf24l01.h"
#include "radio.h"

#include "net/rime/packetbuf.h"
#include "net/rime/rimestats.h"

#include "sys/timetable.h"

#include "ringbuf.h"

#define RADIOSTATS 0
#define TRUE		1
#define FALSE		0

#define SOFT_IRQ_SIZE    16
static struct ringbuf softIrqList;
static uint8_t softIrqBuf[SOFT_IRQ_SIZE];

/* See clock.c and httpd-cgi.c for RADIOSTATS code */
static uint8_t rf_radio_on = 0;

#if RADIOSTATS
uint8_t RF230_rsigsi;
uint16_t RF230_sendpackets,RF230_receivepackets,RF230_sendfail,RF230_receivefail;
#endif

#if RF230_CONF_TIMESTAMPS
#include "net/rime/timesynch.h"
#define TIMESTAMP_LEN 3
#else /* RF230_CONF_TIMESTAMPS */
#define TIMESTAMP_LEN 0
#endif /* RF230_CONF_TIMESTAMPS */

#ifndef RF230_CONF_CHECKSUM
#define RF230_CONF_CHECKSUM 0
#endif /* RF230_CONF_CHECKSUM */

#if RF230_CONF_CHECKSUM
#include "lib/crc16.h"
#define CHECKSUM_LEN 2
#else
#define CHECKSUM_LEN 0
#endif /* RF230_CONF_CHECKSUM */

#define FRAME_LEN   1

#define AUX_LEN (CHECKSUM_LEN + TIMESTAMP_LEN + FRAME_LEN)

struct timestamp {
  uint16_t time;
  uint8_t authority_level;
};

#define DEBUG 1
#if DEBUG
#define PRINTF(FORMAT,args...) printf_P(PSTR(FORMAT),##args)
#else
#define PRINTF(...) do {} while (0)
#endif

#if RF230_CONF_TIMESTAMPS
int rf230_authority_level_of_sender;

/* XXX hack: these will be made as Chameleon packet attributes */
rtimer_clock_t rf230_time_of_arrival, rf230_time_of_departure;

static rtimer_clock_t setup_time_for_transmission;
static unsigned long total_time_for_transmission, total_transmission_len;
static int num_transmissions;
#endif /* RF230_CONF_TIMESTAMPS */

#ifdef USE_TX_MODE
#define RADIO_VECT	 INT0_vect	
#define IRQPIN  (0x2)
#define IRQ_PORT D
#else
#define RADIO_VECT	 PCINT0_vect	
#define IRQPIN  (0x4)
#define IRQ_PORT A
#endif

#define MASK_MAX_RT 4
#define MASK_TX_DS  5
#define MASK_RX_DR  6    

//good macro
#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)

#define HAL_MIN_FRAME_LENGTH   ( 0x03 ) /**< A frame should be at least 3 bytes. */
#define HAL_MAX_FRAME_LENGTH   ( 0x20 ) /**< A frame should no more than 32 bytes. */
#define HAL_CALCULATED_CRC_OK   (0) /**<  CRC calculated over the frame including the CRC field should be 0. */

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))
	
/*============================ TYPDEFS =======================================*/
/** \struct hal_rx_frame_t
 *  \brief  This struct defines the rx data container.
 *
 *  \see hal_frame_read
 */
typedef struct{
    uint8_t length;                       /**< Length of frame. */
    uint8_t data[ HAL_MAX_FRAME_LENGTH ]; /**< Actual frame data. */
    uint8_t lqi;                          /**< LQI value for received frame. */
    bool crc;                             /**< Flag - did CRC pass for received frame? */
} hal_rx_frame_t;

#define ISR_TX_DS (0x1)
#define ISR_MAX_RT (0x2)
#define ISR_RX_DR (0x3)

typedef enum{
     ISR_RX_MODE = 0x0,
     ISR_TX_MODE = 0x1,
}isr_mode_t;

volatile uint8_t write_busy = TRUE;

/* The frame is buffered to rxframe in the interrupt routine in hal.c */
hal_rx_frame_t rxframe;

/*---------------------------------------------------------------------------*/
PROCESS(nrf24l01_process, "nRF24L01 receiver");
/*---------------------------------------------------------------------------*/

int nrf24l01_send(const void *data, unsigned short len);
int nrf24l01_read(void *buf, unsigned short bufsize);
void nrf24l01_set_receiver(void (* recv)(const struct radio_driver *d));
int nrf24l01_on(void);
int nrf24l01_off(void);
int nrf24l01_ioctl(int cmd, const void *buf, unsigned short buf_len);

const struct radio_driver nrf24l01_driver =
  {
    nrf24l01_send,
    nrf24l01_read,
    nrf24l01_set_receiver,
    nrf24l01_on,
    nrf24l01_off,
    nrf24l01_ioctl,
  };

static void (* receiver_callback)(const struct radio_driver *);

//signed char rf230_last_rssi;
//uint8_t rf230_last_correlation;
//static uint8_t rssi_val;
uint8_t rx_mode;
/* Radio stuff in network byte order. */
//static uint16_t pan_id;

//static int channel;
/*----------------------------------------------------------------------------*/
/** \brief  This function checks if the radio transceiver is sleeping.
 *
 *  \retval     true    The radio transceiver is in SLEEP or one of the *_NOCLK
 *                      states.
 *  \retval     false   The radio transceiver is not sleeping.
 */
bool radio_is_sleeping(void)
{
    bool sleeping = false;

    /* The radio transceiver will be at SLEEP or one of the *_NOCLK states only if */
    /* the SLP_TR pin is high. */
    /*if (hal_get_slptr() != 0){
        sleeping = true;
    }*/

    return sleeping;
}

 #if 0  //nick start
 /*----------------------------------------------------------------------------*/
/** \brief  This function return the Radio Transceivers current state.
 *
 *  \retval     P_ON               When the external supply voltage (VDD) is
 *                                 first supplied to the transceiver IC, the
 *                                 system is in the P_ON (Poweron) mode.
 *  \retval     BUSY_RX            The radio transceiver is busy receiving a
 *                                 frame.
 *  \retval     BUSY_TX            The radio transceiver is busy transmitting a
 *                                 frame.
 *  \retval     RX_ON              The RX_ON mode enables the analog and digital
 *                                 receiver blocks and the PLL frequency
 *                                 synthesizer.
 *  \retval     TRX_OFF            In this mode, the SPI module and crystal
 *                                 oscillator are active.
 *  \retval     PLL_ON             Entering the PLL_ON mode from TRX_OFF will
 *                                 first enable the analog voltage regulator. The
 *                                 transceiver is ready to transmit a frame.
 *  \retval     BUSY_RX_AACK       The radio was in RX_AACK_ON mode and received
 *                                 the Start of Frame Delimiter (SFD). State
 *                                 transition to BUSY_RX_AACK is done if the SFD
 *                                 is valid.
 *  \retval     BUSY_TX_ARET       The radio transceiver is busy handling the
 *                                 auto retry mechanism.
 *  \retval     RX_AACK_ON         The auto acknowledge mode of the radio is
 *                                 enabled and it is waiting for an incomming
 *                                 frame.
 *  \retval     TX_ARET_ON         The auto retry mechanism is enabled and the
 *                                 radio transceiver is waiting for the user to
 *                                 send the TX_START command.
 *  \retval     RX_ON_NOCLK        The radio transceiver is listening for
 *                                 incomming frames, but the CLKM is disabled so
 *                                 that the controller could be sleeping.
 *                                 However, this is only true if the controller
 *                                 is run from the clock output of the radio.
 *  \retval     RX_AACK_ON_NOCLK   Same as the RX_ON_NOCLK state, but with the
 *                                 auto acknowledge module turned on.
 *  \retval     BUSY_RX_AACK_NOCLK Same as BUSY_RX_AACK, but the controller
 *                                 could be sleeping since the CLKM pin is
 *                                 disabled.
 *  \retval     STATE_TRANSITION   The radio transceiver's state machine is in
 *                                 transition between two states.
 */
uint8_t
radio_get_trx_state(void)
{
    return hal_subregister_read(SR_TRX_STATUS);
}

/*----------------------------------------------------------------------------*/
/** \brief  This function will reset the state machine (to TRX_OFF) from any of
 *          its states, except for the SLEEP state.
 */
void
radio_reset_state_machine(void)
{
    hal_set_slptr_low();
    delay_us(TIME_NOCLK_TO_WAKE);
    hal_subregister_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);
    delay_us(TIME_CMD_FORCE_TRX_OFF);
}
/*----------------------------------------------------------------------------*/
/** \brief  This function will change the current state of the radio
 *          transceiver's internal state machine.
 *
 *  \param     new_state        Here is a list of possible states:
 *             - RX_ON        Requested transition to RX_ON state.
 *             - TRX_OFF      Requested transition to TRX_OFF state.
 *             - PLL_ON       Requested transition to PLL_ON state.
 *             - RX_AACK_ON   Requested transition to RX_AACK_ON state.
 *             - TX_ARET_ON   Requested transition to TX_ARET_ON state.
 *
 *  \retval    RADIO_SUCCESS          Requested state transition completed
 *                                  successfully.
 *  \retval    RADIO_INVALID_ARGUMENT Supplied function parameter out of bounds.
 *  \retval    RADIO_WRONG_STATE      Illegal state to do transition from.
 *  \retval    RADIO_BUSY_STATE       The radio transceiver is busy.
 *  \retval    RADIO_TIMED_OUT        The state transition could not be completed
 *                                  within resonable time.
 */

radio_status_t
radio_set_trx_state(uint8_t new_state)
{
    uint8_t original_state;

    /*Check function paramter and current state of the radio transceiver.*/
    if (!((new_state == TRX_OFF)    ||
          (new_state == RX_ON)      ||
          (new_state == PLL_ON)     ||
          (new_state == RX_AACK_ON) ||
          (new_state == TX_ARET_ON))){
        return RADIO_INVALID_ARGUMENT;
    }

    if (radio_is_sleeping() == true){
        return RADIO_WRONG_STATE;
    }

    // Wait for radio to finish previous operation
    for(;;)
    {
        original_state = radio_get_trx_state();
        if (original_state != BUSY_TX_ARET &&
            original_state != BUSY_RX_AACK &&
            original_state != BUSY_RX && 
            original_state != BUSY_TX)
            break;
    }

    if (new_state == original_state){
        return RADIO_SUCCESS;
    }


    /* At this point it is clear that the requested new_state is: */
    /* TRX_OFF, RX_ON, PLL_ON, RX_AACK_ON or TX_ARET_ON. */

    /* The radio transceiver can be in one of the following states: */
    /* TRX_OFF, RX_ON, PLL_ON, RX_AACK_ON, TX_ARET_ON. */
    if(new_state == TRX_OFF){
        radio_reset_state_machine(); /* Go to TRX_OFF from any state. */
    } else {
        /* It is not allowed to go from RX_AACK_ON or TX_AACK_ON and directly to */
        /* TX_AACK_ON or RX_AACK_ON respectively. Need to go via RX_ON or PLL_ON. */
        if ((new_state == TX_ARET_ON) &&
            (original_state == RX_AACK_ON)){
            /* First do intermediate state transition to PLL_ON, then to TX_ARET_ON. */
            /* The final state transition to TX_ARET_ON is handled after the if-else if. */
            hal_subregister_write(SR_TRX_CMD, PLL_ON);
            delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
        } else if ((new_state == RX_AACK_ON) &&
                 (original_state == TX_ARET_ON)){
            /* First do intermediate state transition to RX_ON, then to RX_AACK_ON. */
            /* The final state transition to RX_AACK_ON is handled after the if-else if. */
            hal_subregister_write(SR_TRX_CMD, RX_ON);
            delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
        }

        /* Any other state transition can be done directly. */
        hal_subregister_write(SR_TRX_CMD, new_state);

        /* When the PLL is active most states can be reached in 1us. However, from */
        /* TRX_OFF the PLL needs time to activate. */
        if (original_state == TRX_OFF){
            delay_us(TIME_TRX_OFF_TO_PLL_ACTIVE);
        } else {
            delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
        }
    } /*  end: if(new_state == TRX_OFF) ... */

    /*Verify state transition.*/
    radio_status_t set_state_status = RADIO_TIMED_OUT;

    if (radio_get_trx_state() == new_state){
        set_state_status = RADIO_SUCCESS;
        /*  set rx_mode flag based on mode we're changing to */
        if (new_state == RX_ON ||
            new_state == RX_AACK_ON){
            rx_mode = true;
        } else {
            rx_mode = false;
    }
    }

    return set_state_status;
}
#endif  //nick end

/*----------------------------------------------------------------------------*/
/** \brief  This function reads data from one of the radio transceiver's registers.
 *
 *  \param  address Register address to read from. See datasheet for register
 *                  map.
 *
 *  \see Look at the at86rf230_registermap.h file for register address definitions.
 *
 *  \returns The actual value of the read register.
 */
inline uint8_t
spi_register_read(uint8_t address)
{
    uint8_t register_value;
    
    //SPI_READ_REG(address, register_value);         
    SPI_ENABLE();		
    SPI_TRANSFER(address);	
    register_value = (uint8_t)SPI_TRANSFER(0);	
    SPI_DISABLE();		

    return register_value;
}

/*----------------------------------------------------------------------------*/
/** \brief  This function reads the value of a specific subregister.
 *
 *  \see Look at the at86rf230_registermap.h file for register and subregister
 *       definitions.
 *
 *  \param  address  Main register's address.
 *  \param  mask  Bit mask of the subregister.
 *  \param  position   Bit position of the subregister
 *  \retval Value of the read subregister.
 */
uint8_t
spi_subregister_read(uint8_t address, uint8_t mask, uint8_t position)
{
    /* Read current register value and mask out subregister. */
    uint8_t register_value = spi_register_read(address);
    register_value &= mask;
    register_value >>= position; /* Align subregister value. */

    return register_value;
}
/*----------------------------------------------------------------------------*/
/** \brief  This function writes a new value to one of the radio transceiver's
 *          subregisters.
 *
 *  \see Look at the at86rf230_registermap.h file for register and subregister
 *       definitions.
 *
 *  \param  address  Main register's address.
 *  \param  mask  Bit mask of the subregister.
 *  \param  position  Bit position of the subregister
 *  \param  value  Value to write into the subregister.
 */
void
spi_subregister_write(uint8_t address, uint8_t mask, uint8_t position,
                            uint8_t value)
{
    /* Read current register value and mask area outside the subregister. */
    volatile uint8_t register_value = spi_register_read(address);
    register_value &= ~mask;

    /* Start preparing the new subregister value. shift in place and mask. */
    value <<= position;
    value &= mask;

    value |= register_value; /* Set the new subregister value. */

    /* Write the modified register value. */
    SPI_WRITE_REG(address, value);
}

/*---------------------------------------------------------------------------*/
static unsigned int
nrf24l01_status(void)
{
  uint8_t status;
  status = spi_register_read(RF_NOP);  
  PRINTF("status=0x%x\r\n", status);
  return (status & (BV(MASK_TX_DS) | BV(MASK_TX_DS) | BV(MASK_RX_DR)) );
}

/*---------------------------------------------------------------------------*/
void
nrf24l01_waitidle(void)
{
	PRINTF("nrf24l01_waitidle");
	uint8_t radio_state;

    for(;;)
    {
	radio_state = spi_register_read(RF_NOP);
	if(radio_state & (0x1 << MASK_TX_DS) || radio_state & (0x1 << MASK_MAX_RT) || radio_state & (0x1 << MASK_RX_DR))
	{
	    PRINTF(".");
	}
	else
	{
	    break;
	}       
    }
}

/*---------------------------------------------------------------------------*/
static uint8_t locked, lock_on, lock_off;

static void
on(void)
{
  uint8_t reg = 0;
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  PRINTF("rf230 internal on +\n");
  rf_radio_on = 1;

  SPI_READ_BUF(READ_REG, &reg, 1);
  reg |= 0x1<<1;
  
  SPI_CHIP_ENABLE();  
  SPI_WRITE_REG(WRITE_REG, reg); //power upr
  SPI_CHIP_DISABLE();
  
  SPI_READ_BUF(READ_REG, &reg, 1);
  PRINTF("rf230 internal on -, state:0x%x\n", reg);
}

static void
off(void)
{
  PRINTF("rf230 internal off\n");
  uint8_t reg = 0;
  rf_radio_on = 0;
  
  /* Wait for transmission to end before turning radio off. */
  //nrf24l01_waitidle(); 

  /* Force the device into TRX_OFF. */   
  //radio_reset_state_machine();
   
  /* Sleep Radio */
  //just power down chip, no matter it is tx or rx mode
  SPI_READ_BUF(READ_REG, &reg, 1);	    			
  reg &= ~0x1<<1;
  SPI_CHIP_ENABLE();
  SPI_WRITE_REG(WRITE_REG, reg);
  SPI_CHIP_DISABLE();

  PRINTF("rf230 internal on -, state:0x%x\n", reg);
  
  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
}
/*---------------------------------------------------------------------------*/
#define GET_LOCK() locked = 1
static void RELEASE_LOCK(void) {
  if(lock_on) {
    on();
    lock_on = 0;
  }
  if(lock_off) {
    off();
    lock_off = 0;
  }
  locked = 0;
}
/*---------------------------------------------------------------------------*/
void
nrf24l01_set_receiver(void (* recv)(const struct radio_driver *))
{
//PRINTF("rf230_set receiver\n");
  receiver_callback = recv;
}

/*---------------------------------------------------------------------------*/
int
nrf24l01_off(void)
{
   PRINTF("rf230_off\n");	
  /* Don't do anything if we are already turned off. */
  if(rf_radio_on == 0) {
    return 1;
  }

  /* If we are called when the driver is locked, we indicate that the
     radio should be turned off when the lock is unlocked. */
  if(locked) {
    lock_off = 1;
    return 1;
  }
  off();

  return 1;
}
/*---------------------------------------------------------------------------*/
int
nrf24l01_on(void)
{
  PRINTF("\nrf24l01_on\n");
  if(rf_radio_on) {
    return 1;
  }
  if(locked) {
    lock_on = 1;
    return 1;
  }
  on();

  return 1;
}

#if 0 //nick start
/*---------------------------------------------------------------------------*/
int
rf230_get_channel(void)
{
    return hal_subregister_read(SR_CHANNEL);
//	return channel;
}
/*---------------------------------------------------------------------------*/
void
rf230_set_channel(int c)
{
 /* Wait for any transmission to end. */
  nrf24l01_waitidle();
	
//channel=c;
  hal_subregister_write(SR_CHANNEL, c);

}
#endif  //nick end

/*---------------------------------------------------------------------------*/
/* Process to handle input packets
 * Receive interrupts cause this process to be polled
 * It calls the core MAC layer which calls rf230_read to get the packet
*/
PROCESS_THREAD(nrf24l01_process, ev, data)
{
  PROCESS_BEGIN();
  
  while(1) 
  {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    
#if RF230_TIMETABLE_PROFILING
    TIMETABLE_TIMESTAMP(rf230_timetable, "poll");
#endif /* RF230_TIMETABLE_PROFILING */
    /*if(ringbuf_elements(&softIrqList) <= 0)
    {
        PRINTF("No soft irq, continue!!\n");
        continue;
    }*/
    //int softIrq = ringbuf_get(&softIrqList);
    //if(ISR_RX_DR == softIrq)
    {
        if(receiver_callback != NULL) 
        {
            receiver_callback(&nrf24l01_driver);
    	  
#if RF230_TIMETABLE_PROFILING
          TIMETABLE_TIMESTAMP(rf230_timetable, "end");
          timetable_aggregate_compute_detailed(&aggregate_time,
    					   &rf230_timetable);
          timetable_clear(&rf230_timetable);
#endif /* RF230_TIMETABLE_PROFILING */
        } 
        else 
        {
          PRINTF("nrf24l01_process not receiving function\n");
          //flushrx();
        }
    }  //recv..
  }

  PROCESS_END();
}
	
/*---------------------------------------------------------------------------*/
/*
 * This routine is called by the radio receive interrupt in hal.c
 * It just sets the poll flag for the rf230 process.
 */
#if RF230_CONF_TIMESTAMPS
static volatile rtimer_clock_t interrupt_time;
static volatile int interrupt_time_set;
#endif /* RF230_CONF_TIMESTAMPS */

#if RF230_TIMETABLE_PROFILING
#define rf230_timetable_size 16
TIMETABLE(rf230_timetable);
TIMETABLE_AGGREGATE(aggregate_time, 10);
#endif /* RF230_TIMETABLE_PROFILING */

void
nrf24l01_interrupt(void)
{
#if RF230_CONF_TIMESTAMPS
  interrupt_time = timesynch_time();
  interrupt_time_set = 1;
#endif /* RF230_CONF_TIMESTAMPS */
  process_poll(&nrf24l01_process); 
  
#if RF230_TIMETABLE_PROFILING
  timetable_clear(&rf230_timetable);
  TIMETABLE_TIMESTAMP(rf230_timetable, "interrupt");
#endif /* RF230_TIMETABLE_PROFILING */
  return;
}

//In enhanced shockburst mode, this means a ACK is received and TX FIFO is removed
//so you can transmit the next package
inline void status_tx_ds_handler(void)
{
    //as Tx, we use pipe 0 as default
    //ringbuf_put(&softIrqList, ISR_TX_DS);
    write_busy = FALSE;
}

//In the enbanced shockburst mode, this means it is up to the max counter of re-transmit
//And the TX FIFO won't be removed, so you need do something here, good luck!
inline void status_max_rt_handler(void)
{
    /*uint8_t plos_cnt =  spi_register_read(OBSERVE_TX);
	
    if(0x0F == (plos_cnt&(~0xF)) >> 4)
    {
	//need reset PLOS_CNT by writting RF_CH or clear TX_FIFO	
	spi_send_cmd(0xE1);   //this command can clear TX_FIFO
	SPI_WRITE_REG(WRITE_REG+RF_CH, 0x02);  //here I need reset the packet loss counter to 0
        //ringbuf_put(&softIrqList, ISR_MAX_RT);
    }*/
}

//Reads the current RX buffer into the data array
//Forces an RX buffer flush
void status_rx_dr_handler(uint8_t rx_p_no)
{        
    /*switch(rx_p_no)
    {
	case 7:
	    dbgmsg("Rx FIFO is empty\r\n");
            return;
	case 6:
	    dbgmsg("Rx not used\r\n");
	    return;
     }*/

    //PRINTF("status_rx_ds_handler, rx pipe number: %d\r\n", rx_p_no);       
#ifndef USE_SPI_SIMULATOR
    uint8_t *rx_data=0;
    //ENTER_CRITICAL_REGION();
    SS_LOW();
    /*Send frame read command.*/
    SPDR = RD_RX_PLOAD;
    while ((SPSR & (1 << SPIF)) == 0) {;}
    uint8_t frame_length = SPDR;

    /*Read frame length.*/
    SPDR = frame_length;
    while ((SPSR & (1 << SPIF)) == 0) {;}
    frame_length = SPDR;

    /*Check for correct frame length.*/
    if ((frame_length >= 1) && (frame_length <= PLOAD_WIDTH))
    {
        uint16_t crc = 0;
        rx_data = (rxframe.data);
        rxframe.length = frame_length;
        /*Upload frame buffer to data pointer. Calculate CRC.*/
        SPDR = frame_length;
        while ((SPSR & (1 << SPIF)) == 0) {;}

        do
        {
            uint8_t tempData = SPDR;
            SPDR = 0;       /*  dummy write */
            *rx_data++ = tempData;

            crc = _crc_ccitt_update(crc, tempData);

            while ((SPSR & (1 << SPIF)) == 0) {;}

        } while (--frame_length > 0);
        rxframe.crc = true;  //(crc == HAL_CALCULATED_CRC_OK);
        rxframe.lqi = 0;
    }
    else
    {       
        rxframe.length = 0;
        rxframe.lqi    = 0;
        rxframe.crc    = false;
    }
    SS_HIGH();
    //LEAVE_CRITICAL_REGION();
#else
    uint16_t crc = 0;
    uint8_t cnt =0;
    SPI_READ_BUF(RD_RX_PLOAD, rxframe.data, PLOAD_WIDTH);
    rxframe.length = PLOAD_WIDTH;
    //calculate the CRC
    for(cnt=0; cnt<rxframe.length; cnt++)
    {
        crc = _crc_ccitt_update(crc, rxframe.data[cnt]);
    }
    rxframe.crc = true;  //(crc == HAL_CALCULATED_CRC_OK);
    rxframe.lqi = 0;
#endif            
    //ringbuf_put(&softIrqList, ISR_RX_DR);
    nrf24l01_interrupt();
}

//configure INT0 interrupt for IRQ
static void nRF24L01_enable_intr(void)
{
#ifdef USE_TX_MODE
	cli();   //disable all the interrupts before set EICRA, or setting EICRA will cause intertupts
	EIMSK &= ~(1<<INT0);   //disable INT0 before setting EICRA
	EICRA |= (0<<ISC01) | (0<<ISC00);    //only low level interrupt
        DDR(IRQ_PORT) &= ~BV(IRQPIN);    //set INT0(PD2) as input
        //PORT(IRQ_PORT) |= BV(IRQPIN);  //drive high
        EIMSK |= (1<<INT0);   //enable INT0 after set EICRA       
	sei();  //enable all intertupts
#else
        cli();
        PCMSK0 &= ~(1<<PCINT4); //disable PCINT4 before setting PCICR
        PCICR |= (1<<PCIE0);   //enable Pin Change interrupt control register
        PCIFR |= (1<<PCIF0);    //clear flag
        DDR(IRQ_PORT) &= ~BV(IRQPIN);    //set PCINT4(PA4) as input
        PCMSK0 |= (1<<PCINT4);  //enable
        sei();  //enable all intertupts     
#endif    
}

ISR(RADIO_VECT)
{
  //cli();  //HW will disable itself, so we needn't do this
#if RF230_CONF_TIMESTAMPS
  interrupt_time = timesynch_time();
  interrupt_time_set = 1;
#endif /* RF230_CONF_TIMESTAMPS */  

  volatile uint8_t status = 0;      
  
  status = spi_send_cmd(RF_NOP);
  //PRINTF("ISR: status=0x%x, isrtype: %d\r\n", status, getISRType());
  //Rx FIFO data ready
  if(status & (1 << MASK_RX_DR))
  {                
	uint8_t rx_pipe_no = (status & 0x07);
	if(rx_pipe_no < 6)
	{
            status_rx_dr_handler(rx_pipe_no);
	}
	sbi(status, MASK_RX_DR);        
    }
    //Tx FIFO data has been sent successfully
    else if(status & (1 << MASK_TX_DS))
    {    	
        status_tx_ds_handler();
	sbi(status, MASK_TX_DS);  //clear intertupt    	
    }
    //Re transmit is up to the max counter, 
    //Note: 
    // 1. If you don't clear this interrupt, no data can be transmitted any more
    // 2. The package lost counter(PLOS_CNT) is incremented at each MAX_RT interrupt	
    else if(status & (1 << MASK_MAX_RT))
    {
	 //Re transmit is up to the max counter, 
        //Note: 
	// 1. If you don't clear this interrupt, no data can be transmitted any more
	// 2. The package lost counter(PLOS_CNT) is incremented at each MAX_RT interrupt	
	status_max_rt_handler();
	sbi(status, MASK_MAX_RT);  //clear the interrupt        
    }
    else
    {        
    }
    //nrf24l01_interrupt();
    
    //must!! clean interrupts
    SPI_WRITE_REG(WRITE_REG+STATUS, status);        

    #if RF230_TIMETABLE_PROFILING
      timetable_clear(&rf230_timetable);
      TIMETABLE_TIMESTAMP(rf230_timetable, "interrupt");
    #endif /* RF230_TIMETABLE_PROFILING */
}
  
/*---------------------------------------------------------------------------*/
void
nrf24l01_set_txpower(uint8_t power)
{
  if (power > TX_PWR_0DBM){
    power=TX_PWR_0DBM;
  }
  if (radio_is_sleeping() ==true) 
  {
	PRINTF("nrf24l01_set_txpower:Sleeping");
  } 
  else
  {
    spi_subregister_write(RF_SETUP_POWER, power);
  }
}
/*---------------------------------------------------------------------------*/
int
nrf24l01_get_txpower(void)
{
  if (radio_is_sleeping() ==true) {
	PRINTF("rf230_get_txpower:Sleeping");
	return 0;
  } else {
    return spi_subregister_read(RF_SETUP_POWER);
  }
}

/*---------------------------------------------------------------------------*/
int
nrf24l01_send(const void *payload, unsigned short payload_len)
{
  uint8_t total_len,buffer[NRF24L01_MAX_TX_FRAME_LENGTH],*pbuf;
  volatile uint8_t tx_result = RADIO_TX_OK;
  //uint8_t counter = 0;
#if RF230_CONF_TIMESTAMPS
  struct timestamp timestamp;
#endif /* RF230_CONF_TIMESTAMPS */
#if RF230_CONF_CHECKSUM
  uint16_t checksum;
#endif /* RF230_CONF_CHECKSUM */

#if RADIOSTATS
  RF230_sendpackets++;
#endif

  GET_LOCK();
  
  RIMESTATS_ADD(lltx);

#if RF230_CONF_CHECKSUM
  checksum = crc16_data(payload, payload_len, 0);
#endif /* RF230_CONF_CHECKSUM */
  total_len = payload_len + AUX_LEN;
  /*Check function parameters and current state.*/
  if (total_len > NRF24L01_MAX_TX_FRAME_LENGTH){
#if RADIOSTATS
    RF230_sendfail++;
#endif   
    PRINTF("failed, total_len=%d bytes\n", total_len);
    return -1;
  }
  pbuf=&buffer[0];
  *pbuf= payload_len;
  memcpy(pbuf+1,payload,payload_len);
  pbuf+=payload_len+1;
  
#if RF230_CONF_CHECKSUM
  memcpy(pbuf,&checksum,CHECKSUM_LEN);
  pbuf+=CHECKSUM_LEN;
#endif /* RF230_CONF_CHECKSUM */

#if RF230_CONF_TIMESTAMPS
  timestamp.authority_level = timesynch_authority_level();
  timestamp.time = timesynch_time();
  memcpy(pbuf,&timestamp,TIMESTAMP_LEN);
  pbuf+=TIMESTAMP_LEN;
#endif /* RF230_CONF_TIMESTAMPS */

 /* If radio is sleeping we have to turn it on first */
 //

 /* if need radio calibrate, do it here */
//

 /* Wait for any previous transmission to finish. */
  //nrf24l01_waitidle();

  /* set tx mode */
  //

  /* set some features here like auto ack */
  //

  /* get the current power and save, then adjust the power to send */
  //
    if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
	PRINTF("power = 0x%x\n", packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER));
        nrf24l01_set_txpower(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) - 1);
  } else {
    nrf24l01_set_txpower(TX_PWR_18DBM);	//-18dbm
  }

  
 /* Now start transmitting... */
  //PRINTF("start sending %d bytes, totallen=%d\n", payload_len, total_len); 

  //send these data
  SPI_WRITE_BUF(WR_TX_PLOAD, buffer, total_len); 
   
   if(1) {
#if RF230_CONF_TIMESTAMPS
      rtimer_clock_t txtime = timesynch_time();
#endif /* RF230_CONF_TIMESTAMPS */

#if 0  //zbs 2012.3.26
      if(rf_radio_on) {
	     ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
      }
      ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
#endif   //zbs 2012.3.26    
      /* We wait until transmission has ended so that we get an
	  accurate measurement of the transmission time.*/
      //nrf24l01_waitidle();
      //radio_set_trx_state(RX_AACK_ON);//Re-enable receive mode
      //BUSYWAIT_UNTIL( (0!=isr_event_write.isr_type), RTIMER_SECOND / 100); 
      uint8_t timeout = 150;
      while(1)
      {               
          if(!write_busy)
          {
                tx_result = RADIO_TX_OK;
                write_busy = TRUE;
                break;
          }
          else if(timeout == 0)
          {
                tx_result = RADIO_TX_ERR;
                break;
           }
          delay_ms(200);
          timeout --;          
      };
	       
      PRINTF("tx_result=0x%x\n", tx_result);
#if RF230_CONF_TIMESTAMPS
      setup_time_for_transmission = txtime - timestamp.time;

      if(num_transmissions < 10000) {
	    total_time_for_transmission += timesynch_time() - txtime;
	    total_transmission_len += total_len;
	    num_transmissions++;
      }
#endif /* RF230_CONF_TIMESTAMPS */

#if 0  //zbs 2012.3.26
#ifdef ENERGEST_CONF_LEVELDEVICE_LEVELS
      ENERGEST_OFF_LEVEL(ENERGEST_TYPE_TRANSMIT,rf230_get_txpower());
#endif
      ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
      if(rf_radio_on) {
	    ENERGEST_ON(ENERGEST_TYPE_LISTEN);
      }
#endif  //zbs 2012.3.26
      RELEASE_LOCK();
      return tx_result;
    }
}

/*---------------------------------------------------------------------------*/
int
nrf24l01_read(void *buf, unsigned short bufsize)
{
  uint8_t *framep;
  uint8_t len;
 
#if RF230_CONF_CHECKSUM
  uint16_t checksum;
#endif /* RF230_CONF_CHECKSUM */

#if RF230_CONF_TIMESTAMPS
  struct timestamp t;
#endif /* RF230_CONF_TIMESTAMPS */

  //PRINTF("nrf24l01_read: %u bytes, lqi: %u, crc: %u\n",rxframe.length,rxframe.lqi,rxframe.crc);

#if DEBUG
  //for (len=0;len<rxframe.length;len++) PRINTF(" %x",rxframe.data[len]);PRINTF("\n");
#endif
  if (rxframe.length==0) {
    return 0;
  }

#if RF230_CONF_TIMESTAMPS
bomb 
  if(interrupt_time_set) {
    rf230_time_of_arrival = interrupt_time;
    interrupt_time_set = 0;
  } else {
    rf230_time_of_arrival = 0;
  }
  rf230_time_of_departure = 0;
#endif /* RF230_CONF_TIMESTAMPS */
  GET_LOCK();
//  if(rxframe.length > RF230_MAX_PACKET_LEN) {
//    // Oops, we must be out of sync.
//  flushrx();
//    RIMESTATS_ADD(badsynch);
//    RELEASE_LOCK();
//    return 0;
//  }

//hal returns two extra bytes containing the checksum
//below works because auxlen is 2
  len = rxframe.length;
  if(len <= AUX_LEN) {
 // flushrx();
    RIMESTATS_ADD(tooshort);
    RELEASE_LOCK();
    return 0;
  }
  
  if(len - AUX_LEN > bufsize) {
//  flushrx();
    RIMESTATS_ADD(toolong);
    RELEASE_LOCK();
    return 0;
  }
  /* Transfer the frame, stripping the checksum */
  framep=&(rxframe.data[0]);
  memcpy(buf,framep, len);
  
  /* Clear the length field to allow buffering of the next packet */
  rxframe.length=0;

#if RADIOSTATS
  RF230_receivepackets++;
#endif

#if RF230_CONF_CHECKSUM
bomb
  memcpy(&checksum,framep,CHECKSUM_LEN);
  framep+=CHECKSUM_LEN;
#endif /* RF230_CONF_CHECKSUM */
#if RF230_CONF_TIMESTAMPS
bomb
  memcpy(&t,framep,TIMESTAMP_LEN);
  framep+=TIMESTAMP_LEN;
#endif /* RF230_CONF_TIMESTAMPS */
 
#if RF230_CONF_CHECKSUM
bomb
  if(checksum != crc16_data(buf, len - AUX_LEN, 0)) {
    PRINTF("rf230: checksum failed 0x%04x != 0x%04x\n",
	   checksum, crc16_data(buf, len - AUX_LEN, 0));
  }
#else
  if (rxframe.crc) {
#endif /* RF230_CONF_CHECKSUM */

 #if RADIOSTATS
    RF230_rsigsi=hal_subregister_read( SR_RSSI );
#endif      
    //nick packetbuf_set_attr(PACKETBUF_ATTR_RSSI, hal_subregister_read( SR_RSSI ));
    //nick packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, rxframe.lqi);
    
    RIMESTATS_ADD(llrx);
    
#if RF230_CONF_TIMESTAMPS
bomb
    rf230_time_of_departure =
      t.time +
      setup_time_for_transmission +
      (total_time_for_transmission * (len - 2)) / total_transmission_len;
  
    rf230_authority_level_of_sender = t.authority_level;

    packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, t.time);
#endif /* RF230_CONF_TIMESTAMPS */
  
  } else {
    PRINTF("rf230: Bad CRC\n");

#if RADIOSTATS
    RF230_receivefail++;
#endif

    RIMESTATS_ADD(badcrc);
    len = AUX_LEN;
  }
// if (?)
     /* Another packet has been received and needs attention. */
//    process_poll(&nrf24l01_process);
//  }
  
  RELEASE_LOCK();
  
  if(len < AUX_LEN) {
    return 0;
  }

  return len - AUX_LEN;
  }

/*---------------------------------------------------------------------------*/
void
nrf24l01_init(uint8_t mode)
{
  PRINTF("\n nrf24l01_init start, mode: %d \n", mode);

  ringbuf_init(&softIrqList, softIrqBuf, SOFT_IRQ_SIZE);
  
  //init the spi functions
  spi_init();  

  SPI_CHIP_ENABLE();

  if(RF_TX_MODE == mode)  //tx mode
  {              
       #if 0 //just for test SPI
	    SPI_ENABLE();
	    SPDR = (WRITE_REG);
            while ((SPSR & (1 << SPIF)) == 0)
	    {
	        PRINTF("\nSPSR1=0x%x\n", SPSR);
	    }
	    SPDR = (0x0C);
            while ((SPSR & (1 << SPIF)) == 0)
	    {
	        PRINTF("\nSPSR2=0x%x\n", SPSR);
	    }
            SPI_DISABLE();
        #endif //
	    
            SPI_WRITE_REG(WRITE_REG, 0x0C); //RX_DR, TX_DS, MAX_RT interrupts enabled, CRC enabled, CRC encoding with 2bytes and as PTX
	    //SPI_WRITE_REG(WRITE_REG+EN_AA, 0x3F); //Enable auto-acknowledge for all data pipe
	    //SPI_WRITE_REG(WRITE_REG+EN_RXADDR, 0x01); //Enable RX address data pipe 0, reset value is 0x03 (data pipe 0 and 1)
	    //SPI_WRITE_REG(WRITE_REG+SETUP_AW, 0x03); //Setup of address width, reset value is 0x33--5 bytes
	    SPI_WRITE_REG(WRITE_REG+SETUP_RETR, 0x1F); //Set auto-retransmision, ARD(auto retransmit delay): 1111, ARC(auto retransmit count): 1111
	    //SPI_WRITE_REG(WRITE_REG+RF_CH, 0x02); //RF Channel 2 (default, not really needed)
	    SPI_WRITE_REG(WRITE_REG+RF_SETUP, 0x01); //Air data rate 1Mbit, -12dBm, Setup LNA

	    //SPI_WRITE_BUF(WRITE_REG+RX_ADDR_P0, (uint8_t*)&TX_ADDRESS, ADDR_WIDTH); //Set receive address to receive aut-ACK, it is equal to TX_ADDR if enbel enhanced shock burst
	    //SPI_WRITE_BUF(WRITE_REG+TX_ADDR, (uint8_t*)&TX_ADDRESS, ADDR_WIDTH); //Set transmit address: 0xE7E7E7E7E7
	    SPI_WRITE_REG(WRITE_REG+RX_PW_P0, PLOAD_WIDTH); // 32 byte receive payload
	
	    //the next 2 commands are relative with dynamic payload lenght, you need disable them if use static payload length
	    //enable dynamic payload length, enable dynamic ACK in specific package(then you can 
	    //use command W_TX_PAYLOAD_NOACK to set a specfic package that needn't wait for ACK)
	    SPI_WRITE_REG(WRITE_REG+DYNPD, 0x01);  //as PTX, at least enable pipe 0	
	    SPI_WRITE_REG(WRITE_REG+FEATURE, 0x05);  			
	    //SPI_WRITE_REG(WRITE_REG, 0x0E); //power upr	            	    
        }
	else   //rx mode
	{	    
            SPI_WRITE_REG(WRITE_REG, 0x0D); //RX_DR, TX_DS, MAX_RT interrupts enabled, CRC enabled, CRC encoding with 2bytes and as PRX
	    //SPI_WRITE_REG(WRITE_REG+EN_AA, 0x3F); //Enable auto-acknowledge for all data pipes(0-5)
	    SPI_WRITE_REG(WRITE_REG+EN_RXADDR, 0x3F); //Enable RX address for all data pipes(0-5)     
	    //SPI_WRITE_REG(WRITE_REG+SETUP_AW, 0x03); //Setup of address width, reset value is 0x33--5 bytes
	    SPI_WRITE_REG(WRITE_REG+SETUP_RETR, 0x1F); //Set auto-retransmision, ARD(auto retransmit delay): 1111, ARC(auto retransmit count): 1111
	    //SPI_WRITE_REG(WRITE_REG+RF_CH, 0x02); //RF Channel 2 (default, not really needed)
	    SPI_WRITE_REG(WRITE_REG+RF_SETUP, 0x01); //Air data rate 1Mbit, -12dBm, Setup LNA

	    //SPI_WRITE_BUF(WRITE_REG+RX_ADDR_P0, (uint8_t*)&TX_ADDRESS, ADDR_WIDTH); //Set receive address for pipe0
	    //SPI_WRITE_BUF(WRITE_REG+RX_ADDR_P1, (uint8_t*)&RX_ADDRESS, ADDR_WIDTH); //Set receive address for pipe1
	    //SPI_WRITE_REG(WRITE_REG+RX_ADDR_P2, 0XC3); //Set receive address for pipe2
	    //SPI_WRITE_REG(WRITE_REG+RX_ADDR_P3, 0XC4); //Set receive address for pipe3
	    //SPI_WRITE_REG(WRITE_REG+RX_ADDR_P4, 0XC5); //Set receive address for pipe4
	    //SPI_WRITE_REG(WRITE_REG+RX_ADDR_P5, 0XC6); //Set receive address for pipe5

	    SPI_WRITE_REG(WRITE_REG+RX_PW_P0, PLOAD_WIDTH); // 32 byte receive payload for pipe 0
	    SPI_WRITE_REG(WRITE_REG+RX_PW_P1, PLOAD_WIDTH); // 32 byte receive payload
	    SPI_WRITE_REG(WRITE_REG+RX_PW_P2, PLOAD_WIDTH); // 32 byte receive payload
	    SPI_WRITE_REG(WRITE_REG+RX_PW_P3, PLOAD_WIDTH); // 32 byte receive payload
	    SPI_WRITE_REG(WRITE_REG+RX_PW_P4, PLOAD_WIDTH); // 32 byte receive payload
	    SPI_WRITE_REG(WRITE_REG+RX_PW_P5, PLOAD_WIDTH); // 32 byte receive payload for pipe 5	
	
	    //don't use static payload length
	    /*SPI_WRITE_REG(WRITE_REG+RX_PW_P0, 0xFF);  //data pipe 0, number of bytes: 32
	    SPI_WRITE_REG(WRITE_REG+RX_PW_P1, 0xFF);  //data pipe 1, number of bytes: 32
	    SPI_WRITE_REG(WRITE_REG+RX_PW_P2, 0xFF);  //data pipe 2, number of bytes: 32
	    SPI_WRITE_REG(WRITE_REG+RX_PW_P3, 0xFF);  //data pipe 3, number of bytes: 32
	    SPI_WRITE_REG(WRITE_REG+RX_PW_P4, 0xFF);  //data pipe 4, number of bytes: 32
	    SPI_WRITE_REG(WRITE_REG+RX_PW_P5, 0xFF);  //data pipe 5, number of bytes: 32
	    */
	    SPI_WRITE_REG(WRITE_REG+DYNPD, 0x3F);  //as PRX, enable all pipes
	    SPI_WRITE_REG(WRITE_REG+FEATURE, 0x05);  //enable dynamic payload length	

	    //SPI_WRITE_REG(WRITE_REG, 0x0F); //power upr
	}	

	SPI_CHIP_DISABLE();	

        //init interrupt
        nRF24L01_enable_intr();  

	PRINTF("\nnrf24l01_init end\n");
  /* Start the packet receive process */
  process_start(&nrf24l01_process, NULL);
}

int nrf24l01_ioctl(int cmd, const void *buf, unsigned short buf_len)
{
    if(!buf)    return 0;

    uint8_t reg = 0;
    if(IOCTL_SET_MODE == cmd)
    {
        if(RF_TX_MODE == *((uint8_t*)buf)){  //tx mode
            SPI_READ_BUF(READ_REG, &reg, 1);	    			
	    reg &= ~(0x1 | 0x1 << MASK_TX_DS | 0x1 << MASK_MAX_RT);
            reg |= 0x1 << MASK_RX_DR;  //disable RX interrupt
	    SPI_CHIP_ENABLE();
            SPI_WRITE_REG(WRITE_REG, reg); //RX_DR, TX_DS, MAX_RT interrupts enabled, CRC enabled, CRC encoding with 2bytes and as PTX
            SPI_CHIP_DISABLE();	    
            SPI_READ_BUF(READ_REG, &reg, 1);	    
	    PRINTF("nrf24l01_ioctl, tx=0x%x\n", reg);
        }
	else{  //rx mode
	    SPI_READ_BUF(READ_REG, &reg, 1);
	    reg |= 0x1 | 0x1 << MASK_TX_DS | 0x1 << MASK_MAX_RT;    //disable tx interrupts        
	    reg &= ~(0x1 << MASK_RX_DR);
	    SPI_CHIP_ENABLE();
	    SPI_WRITE_REG(WRITE_REG, reg); //RX_DR, TX_DS, MAX_RT interrupts enabled, CRC enabled, CRC encoding with 2bytes and as PRX
	    SPI_CHIP_DISABLE();	    
	    SPI_READ_BUF(READ_REG, &reg, 1);	    
	    PRINTF("nrf24l01_ioctl, rx=0x%x\n", reg);
	}
     }
    else if(IOCTL_GET_MODE  == cmd)
    {
        SPI_READ_BUF(READ_REG, &reg, 1);	
        *((uint8_t*)buf) = reg;
    }
    else{
     }
     return 1;
}
