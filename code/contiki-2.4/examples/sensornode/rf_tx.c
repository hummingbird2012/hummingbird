/**
 * \file
 *         sensor, tx
 * \author
 *         Nicolas Chang <zhbsh.zhbsh@gmail.com>
 */


#include <stdio.h> /* For printf() */
#include <string.h> /* For memcpy */
#include "contiki.h"
#include "net/rime.h"
#include "atcmd.h"
#include "sensor_device.h"
#include "dev/radio/nrf24l01.h"
#include "dev/1-wire/ds18b20.h"

#define SENSOR_BUFFER_SIZE  40

/*---------------------------------------------------------------------------*/
PROCESS(wsn_tx_process, "WSN TX process");
AUTOSTART_PROCESSES(&wsn_tx_process);

static const char* ROMSTR_VER = "\r\n*VER:1\r\n";
static process_event_t g_data_rec_event;
static sensorDev_t g_sensor;
static sensorAttr_t g_sensor_attrs;
static struct etimer et;
static struct broadcast_conn bc;
static uint8_t g_cmd_buffer[SENSOR_BUFFER_SIZE];

static void recv_bc(struct broadcast_conn *c, rimeaddr_t *from);
static const struct broadcast_callbacks broadcast_callbacks = {recv_bc};
static uint8_t wireless_send_frame(uint8_t *payload, uint8_t payload_length);

static int atcmd_feedback(const char* str, const uint16_t str_len)
{
    uint8_t ret = 0;
    if(!str)    return ret;
    
    uint8_t rf_mode = RF_TX_MODE;
    //set to rx mode as default
    nrf24l01_driver.ioctl(IOCTL_SET_MODE, &rf_mode, 1);
    
    packetbuf_copyfrom(str, str_len);
    ret = broadcast_send(&bc);

    if(TRUE == ret)
    {
        packetbuf_copyfrom(STR_OK, strlen(STR_OK));
        ret = broadcast_send(&bc);
    }
    else
    {
        packetbuf_copyfrom(STR_ERROR, strlen(STR_ERROR));
        ret = broadcast_send(&bc);
    }

    rf_mode = RF_RX_MODE;  //switch to rx mode
    nrf24l01_driver.ioctl(IOCTL_SET_MODE, &rf_mode, 1);

    return ret;
}

static int setAttr(const void *attrs)
{
    if(!attrs)  return 0;
    
    memcpy(g_sensor.attrs, attrs, sizeof(g_sensor.attrs));
    
    return 1;
}

static int getAttr(void* attrs)
{
    if(!attrs)  return 0;
    
    memcpy(attrs, g_sensor.attrs, sizeof(g_sensor.attrs));
    
    return 1;
}

int
at_ok(void)
{
    return wireless_send_frame((uint8_t*)STR_OK, strlen(STR_OK));
}

int
at_error(void)
{
    return wireless_send_frame((uint8_t*)STR_ERROR, strlen(STR_ERROR));
}

int
at_ver(void)
{
    atcmd_feedback(ROMSTR_VER, strlen(ROMSTR_VER));
}

int 
at_setattr(void)
{
    do
    {
        if(g_user_rtcmmd.param_count < 1)    break;
        if(0 != g_sensor.setAttrs(g_user_rtcmmd.params[0]))
            return at_ok();
    }
  while (0);
  
  return at_error();
}

int
at_getattr(void)
{
    do
    {
        if(g_user_rtcmmd.param_count < 1)    break;
        if(0 != g_sensor.getAttrs(g_sensor.packet_buf))
        {         
            return atcmd_feedback((const char*)g_sensor.packet_buf, g_sensor.packet_len);
        }
    }
  while (0);
  
  return at_error();
}

struct AT_CMMD_HANDLES sensor_cmmd_handles[] =
{				    
    { "AT*VER", &at_ver }, // kernel version
    { "AT*RESET", &at_reset }, // system reset 
    { "AT*SETATTR", &at_setattr }, // set sensor attributes
    { "AT*GETATTR", &at_getattr }, // get sensor attributes		    
}; 	 

static uint8_t wireless_send_frame(uint8_t *payload, uint8_t payload_length)
{
    uint8_t ret = 0;
    uint8_t rf_mode = RF_TX_MODE;   

    if(!payload)
    {
        return FALSE;
    }
    //printf("wireless_send_frame>[%d] %s", payload_length, payload );	
    
    nrf24l01_driver.ioctl(IOCTL_SET_MODE, &rf_mode, 1);
                            
    packetbuf_copyfrom(payload, payload_length);
    ret = broadcast_send(&bc);  //ret=0 means fail
    
     rf_mode = RF_RX_MODE;  //switch to rx mode
     nrf24l01_driver.ioctl(IOCTL_SET_MODE, &rf_mode, 1);

     return ret;
}

static void recv_bc(struct broadcast_conn *c, rimeaddr_t *from)
{
     uint8_t len=packetbuf_datalen();
     const uint8_t  *packetBuf  = (uint8_t*)packetbuf_dataptr();
     if(packetBuf && g_sensor.packet_buf && len <= SENSOR_BUFFER_SIZE)
     {        
        /* clear global buffer */
        memset(g_sensor.packet_buf, 0, sizeof(g_sensor.packet_buf));
        /* copy buffer and exit ISR */
        memcpy(g_sensor.packet_buf, packetBuf, len);
        g_sensor.packet_len = len;
        
        process_post(&wsn_tx_process, g_data_rec_event, &g_sensor);
     }
     else
     {
        printf("recv_bc error!!\n");
     }
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(wsn_tx_process, ev, data)
{

  PROCESS_BEGIN();

  uint8_t bTimeout = 0;
  uint8_t rf_mode = RF_RX_MODE;
  char* _shell_cmd = NULL;
  
  printf("\nStarting wsn tx...\n");

  //allocate state change event
  g_data_rec_event = process_alloc_event();
  
  //initialize client sensor
  memset(&g_sensor_attrs, 0, sizeof(g_sensor_attrs));
  memset(&g_sensor, 0, sizeof(g_sensor));
 
  //initialize sensor
  g_sensor_attrs.type = TEMPERATURE;
  g_sensor_attrs.id = 0xfe;
  g_sensor_attrs.state = STARTED;
  g_sensor_attrs.interval = 60;    // 5 sec   
  g_sensor_attrs.power = 100;

  g_sensor.attrs = &g_sensor_attrs;
  g_sensor.packet_buf = g_cmd_buffer;
  g_sensor.platform_data = 0;
  g_sensor.setAttrs = setAttr;
  g_sensor.getAttrs = getAttr;

  nrf24l01_driver.ioctl(IOCTL_SET_MODE, &rf_mode, 1);
  //initialzie at command handler
  setAtCmdHandler(sensor_cmmd_handles, SIZEOF(sensor_cmmd_handles));
  broadcast_open(&bc, 128, &broadcast_callbacks);

  while(1) {
    bTimeout = 0;
    etimer_set(&et, g_sensor.attrs->interval*CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL( (ev == g_data_rec_event) || 0 != (bTimeout = etimer_expired(&et)) );
    etimer_reset(&et);    

    if(0 != bTimeout)  //triggered by timeout
    {
        //start capture temp here...
        //uint16_t temp = ds18b20_get_temp();      
        uint16_t temp = 0x3132;
        //rf_mode = RF_RX_MODE;  //swith to rx to receive controll command from server        
        g_sensor.packet_buf[0]= 0x33;
        g_sensor.packet_buf[1] = temp >> 8;   //high temp
        g_sensor.packet_buf[2] = temp;  //low temp	
        g_sensor.packet_len= 3;	
        wireless_send_frame(g_sensor.packet_buf, g_sensor.packet_len);
    }
    else
    {
        //here need parset AT Command...
        //clean shell commands
        _shell_cmd = AtCmdClean((const char*) g_sensor.packet_buf, g_sensor.packet_len);
        if( _shell_cmd != NULL )
	{			
	    printf(">%s", _shell_cmd);															
	    AtParserCmd(_shell_cmd);								
	    // execute user command							
	    AtCmdMainProc();	//use RF to send instead of uart
	    continue;
	 }			
    }    
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
