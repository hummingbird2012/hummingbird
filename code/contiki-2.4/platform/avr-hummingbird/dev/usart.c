
    #include "usart.h"

    /*
     * active shell file stream
     */
    FILE * gfile_shell = NULL;

    /*
     * Debug file stream
     */
    FILE gfile_dbg =
    FDEV_SETUP_STREAM(usart_putchar, usart_getchar, _FDEV_SETUP_RW);

    //----------------------------------------------------------------------------
    /**
     *	\ingroup usart
     * 
     */
    void usart_init(unsigned long baudrate) 
    { 

        //Serielle Schnittstelle 1
        //Enable TXEN im Register UCR TX-Data Enable
        UCR =(1 << TXEN | 1 << RXEN | 1<< RXCIE);
        // 0 = Parity Mode Disabled
        // 1 = Parity Mode Enabled, Even Parity
        // 2 = Parity Mode Enabled, Odd Parity
        //UCSRC = 0x06 + ((parity+1)<<4);
        //UCSRC |= (1<<USBS);
        //Teiler wird gesetzt 
        UBRR=(F_CPU / (baudrate * 16L) - 1);
    }

    //----------------------------------------------------------------------------
    /**
     *	\ingroup usart
     * 
     */
    int usart_putchar(char c, FILE *stream)
    {
      loop_until_bit_is_set(USR, UDRE);
      UDR = c;
      return 0;
    }

    //----------------------------------------------------------------------------
    int usart_getchar(FILE *stream)
    {
        loop_until_bit_is_set(USR, RXC);
        return UDR;
    }
