#ifndef __COMPILER_H__
#define __COMPILER_H__

#ifdef __IMAGECRAFT__


// choose your AVR device here
#if defined (__AVR_ATmega128__)
#include <iom128.h>
#elif defined (__AVR_ATmega644P__)
#include <iom644.h>
#else
#error "Please define  your CPU type in compiler.h"
#endif

#include <macros.h>

#define outp(val, reg)  (reg = val)
#define inp(reg)        (reg)

#define cli()           CLI()
#define sei()           SEI()
#define cbi(reg, bit)   (reg &= ~BIT(bit))
#define sbi(reg, bit)   (reg |= BIT(bit))

#define SIGNAL(x)       void x(void)  

#define nop() NOP()


#else /* --- GCC --- */
/*#ifndef __AVR_ATmega128__
#define __AVR_ATmega128__
#endif*/
#include <avr/signal.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define nop() asm volatile("nop\n\t"::);

#endif /* Compiler Used */



#endif /* __COMPILER_H__ */
