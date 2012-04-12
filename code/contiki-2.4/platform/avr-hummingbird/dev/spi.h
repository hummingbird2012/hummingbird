#ifndef _SPI_H_
#define _SPI_H_

/*
 * SPI bus configuration for the Hummingbird.
 */

/* init values below */
#define SSPORT	D
#define SSPIN      (0x05)
#define SPIPORT    B
#define MOSIPIN    (0x05)
#define MISOPIN    (0x06)
#define SCKPIN     (0x07)

/* Set the default value if no defined */
#ifndef CEPIN
#define CEPIN      (0x06)
#endif 

#ifndef SSPIN
#define SSPIN      (0x04)
#endif 

#ifndef MOSIPIN
#define MOSIPIN    (0x05)
#endif

#ifndef MISOPIN
#define MISOPIN    (0x06)
#endif

#ifndef SCKPIN
#define SCKPIN     (0x07)
#endif

#ifndef SPIPORT
#define SPIPORT  B
#endif

/* For architectures that have all SPI signals on the same port */
#ifndef SSPORT
#define SSPORT SPIPORT
#endif

#ifndef SCKPORT
#define SCKPORT SPIPORT
#endif

#ifndef MOSIPORT
#define MOSIPORT SPIPORT
#endif

#ifndef MISOPORT
#define MISOPORT SPIPORT
#endif

/**
 * \name Macros used to generate read register names from platform-specific definitions of ports.
 * \brief The various CAT macros (DDR, PORT, and PIN) are used to
 * assign port/pin/DDR names to various macro variables.  The
 * variables are assigned based on the specific connections made in
 * the hardware.  For example TCCR(TICKTIMER,A) can be used in place of TCCR0A
 * if TICKTIMER is defined as 0.
 * \{
 */
#define CAT(x, y)      x##y
#define CAT2(x, y, z)  x##y##z
#define DDR(x)         CAT(DDR,  x)
#define PORT(x)        CAT(PORT, x)
#define PIN(x)         CAT(PIN,  x)
/** \} */

#ifndef BV
#define BV(bitno) _BV(bitno)
#endif

//#define USE_SPI_SIMULATOR 1

#if defined (__AVR_ATmega644P__)
#define SPI_CHIP_ENABLE()    ( PORT(SSPORT) &= ~BV(CEPIN) ) /* ENABLE CEn (active low) */
#define SPI_CHIP_DISABLE()   ( PORT(SSPORT) |=  BV(CEPIN) ) /* DISABLE CEn (active low) */
#else 
#error "Wrong SPI configuration in Contiki-conf.h"
#endif


/* SPI input/output registers. */
//#define SPI_TXBUF SPDR
//#define SPI_RXBUF SPDR

/** This macro will protect the following code from interrupts.*/
#define ENTER_CRITICAL_REGION( ) {uint8_t volatile saved_sreg = SREG; cli( )
/** This macro must always be used in conjunction with AVR_ENTER_CRITICAL_REGION
    so that interrupts are enabled again.*/
#define LEAVE_CRITICAL_REGION( ) SREG = saved_sreg; sei()}

/* Start the SPI transaction by pulling the Slave Select low. */
#define SS_LOW()    ( PORT(SSPORT) &= ~BV(SSPIN) ) /* ENABLE CSn (active low) */

/* End the transaction by pulling the Slave Select High. */
#define SS_HIGH()   ( PORT(SSPORT) |=  BV(SSPIN) ) /* DISABLE CSn (active low) */

#define SPI_ENABLE() { \
  SS_LOW(); /* Start the SPI transaction by pulling the Slave Select low. */

#define SPI_DISABLE() \
    SS_HIGH(); /* End the transaction by pulling the Slave Select High. */ \
    }
  
void spi_init(void);
uint8_t spi_send_cmd(uint8_t cmd);

#if USE_SPI_SIMULATOR
uint8_t spi_rw_byte(uint8_t outgoing);
uint8_t spi_write_buf(uint8_t cmd, uint8_t* const data, uint16_t size);

#define SPI_TRANSFER(to_write)  spi_rw_byte(to_write);				
#else
#define SPI_TRANSFER_WRITE(to_write) (SPDR = (to_write))
#define SPI_TRANSFER_WAIT() ({while ((SPSR & (1 << SPIF)) == 0) {;}}) /* gcc extension, alternative inline function */
#define SPI_TRANSFER_READ() (SPDR)
#define SPI_TRANSFER(to_write) (	  \
				    SPI_TRANSFER_WRITE(to_write),	\
				    SPI_TRANSFER_WAIT(),		\
				    SPI_TRANSFER_READ() )
#endif

/* Write register by SPI */
#define SPI_WRITE_REG(addr, data)   \
  do {                   \
    SPI_ENABLE();		\
    SPI_TRANSFER(addr);	\
    SPI_TRANSFER(data);	\
    SPI_DISABLE();		\
  } while(0) 

/* Write to RAM  */
#define SPI_WRITE_BUF(addr,buffer,count)                 \
  do {                                                       \
    uint8_t i;                                               \
    SPI_ENABLE();                                     \
    SPI_TRANSFER(addr);                   \
    for(i = 0; i < (count); i++) {                           \
      SPI_TRANSFER(((uint8_t*)(buffer))[i]);               \
    }                                                        \
    SPI_DISABLE();                                    \
  } while(0)

/* Read from RAM */
#define SPI_READ_BUF(addr,buffer,count)                    \
  do {                                                       \
    uint8_t i;                                               \
    SPI_ENABLE();                                     \
    SPI_TRANSFER(addr);                        \
    for(i = 0; i < (count); i++) {                           \
      ((uint8_t*)(buffer))[i] = SPI_TRANSFER(0);                     \
    }                                                        \
    SPI_DISABLE();                                    \
  } while(0)
  
#endif  //_SPI_H
