
#include <stdio.h> /* For printf() */
#include <string.h> /* For memcpy */
#include "dev_config.h"
#include "sensor_config.h"
#include "atcmd.h"
#include "dev/radio/nrf24l01.h"
#include "dev/uart.h"

static const char STR_ERROR[] = "\r\nERROR\r\n";
static const char STR_OK[] = "\r\nOK\r\n";


static void
(*reset)(void) = 0x0000;

int
at_reset(void)
{
  reset();
  return TRUE;
}

#ifdef USE_TX_MODE

#else

#endif  //#ifdef USE_TX_MODE
