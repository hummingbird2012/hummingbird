/**
 * \file
 *         sensor, receiver
 * \author
 *         Nicolas Chang <zhbsh.zhbsh@gmail.com>
 */

#include "contiki.h"
#include "net/rime.h"
#include <stdio.h> /* For printf() */
#include "dev/1-wire/sensornode.h"
#include "dev/radio/nrf24l01.h"
#include <string.h> /* For memcpy */

#define USE_BROADCAST_SEND	1
#define MAX_TIMEOUT_CNT  2
#define MAX_FAIL_CNT  5

typedef enum stateMachine
{
    UNKNOWN = 0,
    SCANING, 
    ASSOCIATING,
    AUTHENTICATING,
    CONNECTED    
}networkState;

  //create a sensor object for transmitter
  static sensorObj_t sensor;
  //createa sensor object for the receiver
  static sensorObj_t sensor_ser;

  process_event_t state_change_event;
  uint8_t rf_mode = 0;
  uint8_t timeout_cnt = 0;
  uint8_t bTimeout = 0;
  uint8_t ret = 0;
  uint8_t sendfail_cnt = 0;
    
/*---------------------------------------------------------------------------*/
PROCESS(rf_test_process, "RF test TX process");
AUTOSTART_PROCESSES(&rf_test_process);

static void recv_bc(struct broadcast_conn *c, rimeaddr_t *from);
static void recv_uc(struct unicast_conn *c, rimeaddr_t *from);

static struct etimer et;

static struct broadcast_conn bc;
static const struct broadcast_callbacks broadcast_callbacks = {recv_bc};

static struct unicast_conn uc;
static const struct unicast_callbacks unicast_callbacks = {recv_uc};

rimeaddr_t addr;

static void
recv_bc(struct broadcast_conn *c, rimeaddr_t *from)
{
     uint8_t len=packetbuf_datalen();
     const packetSensor_t  *packetBuf  = (packetSensor_t*)packetbuf_dataptr();
  
     if(MGR_TYPE == packetBuf->type && (sensor.state < CONNECTED && sensor.state > UNKNOWN) )
      {
          sensor.frame_type = MGR_TYPE;
          if(SUB_TYPE_PROBE_RES == packetBuf->sub_type && sensor.state == SCANING)
          {
               if(ERR_OK ==getSensorCap(&sensor_ser, packetBuf)) 
	       {
	             //valid sensor server, push state machine to next step
		     sensor.state = ASSOCIATING;
		     printf("PROBE_RES\n");
	        }
	        else
	        {
	             //invalid sensor server, so keep the current state
		     sensor.state = SCANING;
                 }
       	   }
           else if(SUB_TYPE_ASSOCIA_RES == packetBuf->sub_type && sensor.state == ASSOCIATING)
          {
	      if(ERR_OK == isSensorCapSupport(&sensor, packetBuf))
	      {
	            //can support, push state machine to next step
		    sensor.state = AUTHENTICATING;
		    printf("ASSOCIA_RES\n");
	       }
	       else
	       {
		    //can't support, so disconnect and warn
		    sensor.state = UNKNOWN;						
		}
            }
	    else if(SUB_TYPE_AUTH_RES == packetBuf->sub_type && sensor.state == AUTHENTICATING)
	    {
		//handshake is OK, this sensor can start to work now
		sensor.state = CONNECTED;
		printf("AUTHEN_RES\n");
	    }
            else {return;}		    
      }		
      else if(CTL_TYPE == packetBuf->type && sensor.state == CONNECTED)
      { 
            printf("CTL_REQ\n");
            sensor.frame_type = CTL_TYPE;
            if(SUB_TYPE_CTL_REQ == packetBuf->sub_type)
            {
                memcpy(&(sensor.packet), packetBuf, sizeof(packetSensor_t));
            }
            else {return;}
      }
      else {return;}
      process_post(&rf_test_process, state_change_event, &sensor);
}

static void
recv_uc(struct unicast_conn *c, rimeaddr_t *from)
{
  printf("unicast from %02x.%02x len = %d buf = %s\n",
		  from->u8[0],
		  from->u8[1],
		  packetbuf_datalen(),
		  (char *)packetbuf_dataptr());
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(rf_test_process, ev, data)
{

  PROCESS_BEGIN();

  printf("\nStarting nRF24L01 RF test suite...\n");

  //allocate state change event
  state_change_event = process_alloc_event();
  
  //initialize server sensor
  memset(&sensor_ser, 0, sizeof(sensor_ser));
  
  //initialize client sensor
  memset(&sensor.packet, 0, sizeof(sensor.packet));
  sensor.packet.len = BUFFER_LENGTH;
  
  //generate ID
  sensor.id = 0xfe;
  //set cap
  sensor.cap = 0;
  sensor.cap &= ~(1<<7);  //9bit
  sensor.cap |= (1<<8);  //client, 
  //set state
  sensor.state = UNKNOWN;
  //set interval
  sensor.interval = 3;    // 3 sec   
  //set frame type
  sensor.frame_type = NONE_TYPE;

  /* starting the state machine */
  if(UNKNOWN == sensor.state)
  {
      sensor.state = SCANING;
  }

#if USE_BROADCAST_SEND	
  broadcast_open(&bc, 128, &broadcast_callbacks);
#else
  unicast_open(&uc, 128, 0);
#endif  //USE_BROADCAST_SEND

  while(1) {
    bTimeout = 0;
    etimer_set(&et, sensor.interval*CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL( (ev == state_change_event) || 0 != (bTimeout = etimer_expired(&et)) );
    etimer_reset(&et);    

#if USE_BROADCAST_SEND	
    //printf("\nSending broadcast packet\n");  

    //nrf24l01_driver.ioctl(IOCTL_GET_MODE, &rf_mode, 1);
    //if(rf_mode == RF_RX_MODE)
    {
        rf_mode = RF_TX_MODE;  //switch to tx mode
        nrf24l01_driver.ioctl(IOCTL_SET_MODE, &rf_mode, 1);
    }

    if(CTL_TYPE == sensor.frame_type)  //control frame has the highest priority
    {
        sensor.frame_type = NONE_TYPE;  //clear this type to avoid coming here in the next loop
        if(sensor.state < CONNECTED)
        {
            continue;
        }        
	//parse command here and do some action, remember ACK
	//...
	sensor.packet.type = CTL_TYPE;
	sensor.packet.sub_type = SUB_TYPE_CTL_RES;
	/* you can set the sensor capbility in the buffer */	    
	memset(&sensor.packet.buf, 0, BUFFER_LENGTH);
	sensor.interval = 3;  //waitfor 3s	 
	setSensorCap(&(sensor.packet), &sensor);
	rf_mode = RF_TX_MODE;  
	printf("CTL_RES\n");
    }
    else if(MGR_TYPE == sensor.frame_type)
    {        
        if( UNKNOWN > sensor.state ||CONNECTED < sensor.state )
        {
            //already connected or in unknow state, so break;
            continue;
        }     
        
        if(0 != bTimeout)  //this means timeout
        {
            //timeout            
            if(MAX_TIMEOUT_CNT < timeout_cnt)  //give up and re-try in the beginning
            {
                sensor.frame_type = NONE_TYPE;  //clear this type to avoid coming here in the next loop
                sensor.state = SCANING;
		timeout_cnt = 0;
                printf("timeout!!");
		continue;
            }
            timeout_cnt ++;
        }
			
        if(ASSOCIATING == sensor.state)
        {
	    sensor.packet.type = MGR_TYPE;
	    sensor.packet.sub_type = SUB_TYPE_ASSOCIA_REQ;
	    /* you can set the sensor capbility in the buffer */	    
	    memset(&sensor.packet.buf, 0, BUFFER_LENGTH);
	    sensor.interval = 3;  //waitfor 3s for the ASSOCIA_RES	 
	    setSensorCap(&(sensor.packet), &sensor);
	    rf_mode = RF_RX_MODE;  //need switch to rx
		
	    printf("ASSOCIATING_REQ\n");
        }
        else if(AUTHENTICATING == sensor.state)
        {
	    sensor.packet.type = MGR_TYPE;
	    sensor.packet.sub_type = SUB_TYPE_AUTH_REQ;
	    /* reset the sensor capbility follow the server's request */
	    memset(&sensor.packet.buf, 0, BUFFER_LENGTH);
	    sensor.interval = 3;  //waitfor 3s for the AUTH_RES	 
	    setSensorCap(&(sensor.packet), &sensor);
	    rf_mode = RF_RX_MODE;  //need switch to rx
	    printf("AUTHEN_REQ\n");
        }
        else if(CONNECTED == sensor.state)
        {	
            //state machine: connected state, prepare to captuere temperature in the next loop
            sensor.frame_type = NONE_TYPE;  //break
            sensor.interval = 1;
            continue;            
        }
        else// if(UNKNOWN == sensor.state)
        {
	    //if come here, that means this sensor can't meet server's request,
	    //so make a beep with led flush
	    printf("Connecting ERROR!!\n");  	
	    sensor.interval = 1;
	    continue;  //
        }
    }
    else
    {
        if(SCANING == sensor.state)
        {
	    sensor.packet.type = MGR_TYPE;
	    sensor.packet.sub_type = SUB_TYPE_PROBE_REQ;
	    /* you can set the sensor capbility in the buffer */
	    memset(&sensor.packet.buf, 0, BUFFER_LENGTH);
	    //switch to Rx mode for the PROBE_RES or waitfor timeout
	    sensor.interval = 3;  //scan in every 2 sec    
	    setSensorCap(&(sensor.packet), &sensor);	    
            rf_mode = RF_RX_MODE;  //need switch to rx
            printf("PROBE_REQ\n");
	    
        }
        else if(CONNECTED == sensor.state)
        {
            sensor.packet.type = DAT_TYPE;
	    sensor.packet.sub_type = SUB_TYPE_DATA;
	    /* you can fill the sensor data in the buffer */
	    memset(&sensor.packet.buf, 0, BUFFER_LENGTH);
	    sensor.interval = 5; //transmit every 5 sec, this shoule be controlled by the server
	    //start capture temp here...
	    captureAndPacketSensorTemp(&sensor);
	    rf_mode = RF_RX_MODE;  //swith to rx to receive controll command from server
	    printf("capture temperature!!\n");  	
        }
        else// if(UNKNOWN == sensor.state)
        {
	    //if come here, that means this sensor can't meet server's request,
	    //so make a beep with led flush
	    printf("Working ERROR!!\n");  	
	    sensor.interval = 2;
	    continue;  //
        }        
    }
    packetbuf_copyfrom(&sensor.packet, BUFFER_LENGTH+3);
    ret = broadcast_send(&bc);
    if(0 == ret)
    {                
        if(sensor.state == CONNECTED)   //just disconnecte that when it is in connected state.
        {
            sendfail_cnt++;
            if(sendfail_cnt > MAX_FAIL_CNT)
            {
                printf("fail %d times, Disconnect!!\n", sendfail_cnt);
                sendfail_cnt = 0;
                sensor.state = SCANING;   
                sensor.interval = 3;    // 3 sec   
                sensor.frame_type = NONE_TYPE;
            }
            continue;
        }
    }
    sendfail_cnt = 0;  //send ok, so reset to 0
    if(RF_RX_MODE == rf_mode)
        nrf24l01_driver.ioctl(IOCTL_SET_MODE, &rf_mode, 1);
	
#else    
    memset(&addr, 0, sizeof(rimeaddr_t));
    //the receiver uses the dedicated mac address 0xff:0xfe
    addr.u8[0] = 0xff;
    addr.u8[1] = 0xfe;
    printf("Sending unicast packet\n"); 
    packetbuf_copyfrom("HelloWorld!", 11);
    unicast_send(&uc, &addr);
 #endif  //USE_BROADCAST_SEND
 
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
