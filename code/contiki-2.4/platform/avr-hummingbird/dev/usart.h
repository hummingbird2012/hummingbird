#ifndef _UART_H
#define _UART_H

#include <avr/pgmspace.h>
#include <stdio.h>
#include <avr/io.h>
	
	#if defined (__AVR_ATmega128__)
		#define USR UCSR0A
		#define UCR UCSR0B
		#define UDR UDR0
		#define UBRR UBRR0L
                #define RXC RXC0
		#define USART_RX USART0_RX_vect 
	#endif
	
	#if defined (__AVR_ATmega644__) || (defined (__AVR_ATmega644P__) && !USART_USE1)
		#define USR UCSR0A
		#define UCR UCSR0B
		#define UBRR UBRR0L
		#define EICR EICRB
		#define TXEN TXEN0
		#define RXEN RXEN0
		#define RXCIE RXCIE0
		#define UDR UDR0
		#define UDRE UDRE0
                #define RXC RXC0
		#define USART_RX USART0_RX_vect  
	#endif

	#if defined (__AVR_ATmega644P__) && USART_USE1
		#define USR UCSR1A
		#define UCR UCSR1B
		#define UBRR UBRR1L
		#define EICR EICRB
		#define TXEN TXEN1
		#define RXEN RXEN1
		#define RXCIE RXCIE1
		#define UDR UDR1
		#define UDRE UDRE1
                #define RXC RXC1
		#define USART_RX USART1_RX_vect   
	#endif
	
	#if defined (__AVR_ATmega32__)
		#define USR UCSRA
		#define UCR UCSRB
		#define UBRR UBRRL
		#define EICR EICRB            
		#define USART_RX USART_RXC_vect  
	#endif
	
	#if defined (__AVR_ATmega8__)
		#define USR UCSRA
		#define UCR UCSRB
		#define UBRR UBRRL
	#endif
	
	#if defined (__AVR_ATmega88__)
		#define USR UCSR0A
		#define UCR UCSR0B
		#define UBRR UBRR0L
		#define TXEN TXEN0
		#define UDR UDR0
		#define UDRE UDRE0
                #define RXC RXC0
	#endif
	//----------------------------------------------------------------------------
	
	void usart_init(unsigned long baudrate); 
	int  usart_putchar(char, FILE *);
	int usart_getchar(FILE *);

	//----------------------------------------------------------------------------

#endif //_UART_H
