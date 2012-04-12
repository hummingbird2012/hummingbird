

#include "contiki-conf.h"

#include <util/delay.h>
#include <avr/io.h>
#include "spi.h"

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
  printf("\n spi_init starting \n");
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
  //PORT(SPIPORT) |= BV(SCKPIN) | BV(MOSIPIN);		//driven to high
  PORT(SSPORT) |= BV(SSPIN) | BV(CEPIN); 		//driven to high
  
  //Enable pull-up resistors (page 74), if it is input, set PORTx means pull-up
  PORT(SSPORT) |= BV(MISOPIN); //Pulling up a pin that is grounded will cause 90uA current leak
  printf("\n PORTB=0x%x, DDRB=0x%x, PORTD:0x%x, DDRD=0x%x \n", PORT(SPIPORT), DDR(SPIPORT), PORT(SSPORT), DDR(SSPORT) );
#else
#error "\n No CPU select, spi_init() failed in spi.c \n"
#endif

#if USE_SPI_SIMULATOR
#else
  /* In the SPI master mode, must set SS(SlaveSelect) to output, actually this pin should connect to the CS pin the SPI slave device */
  DDR(SPIPORT) |= BV(4);
  PORT(SPIPORT) |= BV(4);
  /* Enable SPI module and master operation. */
  SPCR = BV(SPE) | BV(MSTR) |BV(SPR0); // | BV(SPIE);  

  /* Enable doubled SPI speed in master mode. */
  //SPSR = BV(SPI2X);  
  printf("\n spi end, SPCR=0x%x\n", SPCR);
#endif    
}


#if USE_SPI_SIMULATOR
/* simulate the SPI */
//Basic SPI RW to nRF
#define  RF_DELAY                      55
#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

uint8_t spi_rw_byte(uint8_t outgoing)
{
    uint8_t i, incoming;
    incoming = 0;

    //Send outgoing byte
    for(i = 0 ; i < 8 ; i++)
    {
        //send from MSB to LSB
        if(outgoing & 0b10000000)
	    PORT(MOSIPORT) |= BV(MOSIPIN);	   
        else
	    PORT(MOSIPORT) &= ~BV(MOSIPIN);
        
        PORT(SCKPORT) |= BV(SCKPIN);  //SPI_CLK = 1;
        _delay_us(RF_DELAY);

        //MISO bit is valid after clock goes going high
        incoming <<= 1;
        if( PIN(MISOPORT) & (1<<MISOPIN) ) incoming |= 0x01;

        PORT(SCKPORT) &= ~BV(SCKPIN);  //SPI_CLK = 0; 
        _delay_us(RF_DELAY);
        
        outgoing <<= 1;
    }

    return(incoming);
}

//Sends one byte command
uint8_t spi_send_cmd(uint8_t cmd)
{
    uint8_t status;
	
    SPI_ENABLE(); //Select chip
    status = spi_rw_byte(cmd);
    SPI_DISABLE(); //Deselect chip
    
    return(status);
}

//write buffer
uint8_t spi_write_buf(uint8_t cmd, uint8_t* const data, uint16_t size)
{
    uint8_t status =0;
    uint8_t* pdata = data;
    uint8_t i = 0;
	
    SPI_ENABLE(); //Select chip
    status = spi_rw_byte(cmd);    //write tx payload
    
    for(i = 0 ; i < size; i++)
    {        
        printf("0x%x,",*pdata);
        spi_rw_byte(*pdata++);                        
    }

    SPI_DISABLE(); //Deselect chip

    return status;
}
#else
//Sends one byte command
uint8_t spi_send_cmd(uint8_t cmd)
{
    uint8_t status;
	
    SPI_ENABLE(); //Select chip
    status = SPI_TRANSFER(cmd);
    SPI_DISABLE(); //Deselect chip
    
    return(status);
}
#endif  //USE_SPI_SIMULATOR
