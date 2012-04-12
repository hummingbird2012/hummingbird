/**
 * \file
 *         sensor, receiver
 * \author
 *         Nicolas Chang <zhbsh.zhbsh@gmail.com>
 */

#include <stdio.h> /* For printf() */
#include <string.h> /* For memcpy */
#include "contiki.h"
#include "net/rime.h"
#include "atcmd.h"
#include "dev/radio/nrf24l01.h"
#include "dev/uart.h"

#define GW_BUFFER_SIZE  40
typedef struct gatewayDev
{
    uint8_t node_buf[GW_BUFFER_SIZE];
    uint8_t node_buf_len;
    uint8_t gw_buf[GW_BUFFER_SIZE];
    uint8_t gw_buf_len;
}gatewayDev_t;

/*---------------------------------------------------------------------------*/
PROCESS(wsn_rx_process, "WSN RX process");
AUTOSTART_PROCESSES(&wsn_rx_process);

/* Local varients */
static const char* ROMSTR_VER = "\r\n*VER:1\r\n";
static gatewayDev_t gwDevInstance;
static process_event_t g_data_rec_event;
static struct etimer e_timer;
static struct broadcast_conn bc;

static void recv_bc(struct broadcast_conn *c, rimeaddr_t *from);
static const struct broadcast_callbacks broadcast_callbacks = {recv_bc};

/*Local functions */
static int atcmd_server_feedback(const char* str, const uint16_t str_len)
{
    if(!str)    return FALSE;

    uart_send_frame((uint8_t*)str, str_len);

    return TRUE;
}

static int at_server_ok(void)
{
    return atcmd_server_feedback(STR_OK, strlen(STR_OK));
}

static int at_server_error(void)
{
    return atcmd_server_feedback(STR_ERROR, strlen(STR_ERROR));
}

static int at_server_ver(void)
{
    if(TRUE == atcmd_server_feedback(ROMSTR_VER, strlen(ROMSTR_VER)))
        return at_server_ok();
    else
        return at_server_error();       
}

struct AT_CMMD_HANDLES gw_cmmd_handles[] =
{				    
    { "AT", &at_server_ok }, // return ok
    { "AT+VER", &at_server_ver }, // kernel version
    { "AT+RST", &at_reset }, // system reset 
}; 

static void recv_bc(struct broadcast_conn *c, rimeaddr_t *from)
{
     uint8_t len=packetbuf_datalen();
     const uint8_t  *packetBuf  = (uint8_t*)packetbuf_dataptr();

     if(packetBuf && gwDevInstance.node_buf && len <= GW_BUFFER_SIZE)
     {        
        /* clear global buffer */
        memset(gwDevInstance.node_buf, 0, sizeof(gwDevInstance.node_buf));
        /* copy buffer and exit ISR */
        memcpy(gwDevInstance.node_buf, packetBuf, len);
        gwDevInstance.node_buf_len = len;
        
        process_post(&wsn_rx_process, g_data_rec_event, &gwDevInstance);
     }
     else
     {
        printf("recv_bc error!!\n");
     }
}

static uint8_t wireless_send_frame(uint8_t *payload, uint8_t payload_length)
{
    uint8_t ret = 0;
    uint8_t rf_mode = RF_TX_MODE;   

    if(!payload)
    {
        return FALSE;
    }
    //printf("wireless_send_frame>[%d]%s", payload_length, payload);	
    
    nrf24l01_driver.ioctl(IOCTL_SET_MODE, &rf_mode, 1);
                            
    packetbuf_copyfrom(payload, payload_length);
    ret = broadcast_send(&bc);

     rf_mode = RF_RX_MODE;  //switch to rx mode
     nrf24l01_driver.ioctl(IOCTL_SET_MODE, &rf_mode, 1);

     return ret;
}

static uint8_t uart_recv_frame()
{
    volatile uint8_t ch;
    uint8_t count = 0;
    
    char* _shell_cmd = NULL;

    if (!is_uart_data_ready())
    {
        return FALSE;
    }

    memset(gwDevInstance.gw_buf, 0, sizeof(gwDevInstance.gw_buf));
    while(1) 
    {
        ch = uart_read_char();
        if (ch == TIMEOUT)
        {
            printf("timeout, no avaliable data\n");
            return FALSE;
        }
        gwDevInstance.gw_buf[count] = ch;
        
        if(ch == '\r') 
        {
            ch = uart_read_char();
            gwDevInstance.gw_buf[++count] = ch;
            if(ch == '\n')
            {
                if(count > 2)
                {
                    gwDevInstance.gw_buf[++count] = ch;
                    if(gwDevInstance.gw_buf[0] == 'A' && gwDevInstance.gw_buf[1] == 'T')
                    {
                        if(gwDevInstance.gw_buf[2] == '*')  //this cmd is for node
                        {
                            gwDevInstance.gw_buf_len = count;
                            wireless_send_frame(gwDevInstance.gw_buf, gwDevInstance.gw_buf_len);

                            break;
                        }
                        else if(gwDevInstance.gw_buf[2] == '+' || gwDevInstance.gw_buf[2] == '\r')  //this cmd is for GW
                        {
                            //here need parset AT Command...
                            //clean shell commands
                            _shell_cmd = AtCmdClean((char*)gwDevInstance.gw_buf, count);
                            if( _shell_cmd != NULL )
                            {			
                        	printf(">%s", _shell_cmd);															
                        	AtParserCmd(_shell_cmd);
                        	// execute user command							
                        	if(TRUE == AtCmdMainProc())
                                {                                    
                                    break;
                        	}
                            }			
                        }  // '+'
                    } //if(cmd_buffer[0] == 'A' && cmd_buffer[1] == 'T')
                }

                //we get the wrong command, so return erro directly
                at_server_error();
                break;  //return error
            }
        }
   
        count ++;
    }    

    return TRUE;
        
  }

/** \}   */

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(wsn_rx_process, ev, data)
{

  PROCESS_BEGIN();

  uint8_t bTimeout = 0;
  uint8_t rf_mode = RF_RX_MODE;
  uint8_t count = 0;
  
  printf("\nStarting wsn rx...\n");

  uart_init();

  //allocate state change event
  g_data_rec_event = process_alloc_event();
 
  //initialize gateway dev
  memset(gwDevInstance.gw_buf, 0, sizeof(gwDevInstance.gw_buf));
  memset(gwDevInstance.node_buf, 0, sizeof(gwDevInstance.node_buf)); 

  //set to rx mode as default
  nrf24l01_driver.ioctl(IOCTL_SET_MODE, &rf_mode, 1);
  
  //initialzie at command handler
  setAtCmdHandler(gw_cmmd_handles, SIZEOF(gw_cmmd_handles));
  broadcast_open(&bc, 128, &broadcast_callbacks);

  while(1) {
    bTimeout = 0;
    etimer_set(&e_timer, CLOCK_SECOND);    
    PROCESS_WAIT_EVENT_UNTIL( (ev == g_data_rec_event) || 0 != (bTimeout = etimer_expired(&e_timer)) );   
    etimer_reset(&e_timer);

    if(0 != bTimeout)  //triggered by timeout
    {
        //
        uart_recv_frame();
    }
    else
    {        
        /*
        for(count = 0; count < gwDevInstance.node_buf_len; count++)
        {
            printf("0x%x, ", gwDevInstance.node_buf[count]);
        }
        printf("\n");
        */
        printf("%d, >%s\n", gwDevInstance.node_buf_len, gwDevInstance.node_buf);
        uart_send_frame(gwDevInstance.node_buf, gwDevInstance.node_buf_len);        
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
