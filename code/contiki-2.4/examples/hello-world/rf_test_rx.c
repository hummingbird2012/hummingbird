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

#define USE_BROADCAST_SEND	1
#define MAX_TIMEOUT_CNT  2

typedef enum stateMachine
{
    UNKNOWN = 0,
    LISTENING, 
    PROBED,
    ASSOCIATED,
    AUTHENTICATED,
    CONNECTED    
}networkState;

  //create a sensor object for transmitter
  static sensorObj_t sensor;
  //createa sensor object for the receiver
  static sensorObj_t sensor_client;

  process_event_t state_change_event;
  uint8_t timeout_cnt = 0;
  uint8_t rf_mode = 0;
  uint8_t bTimeout = 0;
  uint8_t nSyncTime = 0;
  
/*---------------------------------------------------------------------------*/
PROCESS(rf_test_process, "RF test RX process");
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
     if(MGR_TYPE == packetBuf->type && (sensor.state < CONNECTED && sensor.state > UNKNOWN))
     {
          sensor.frame_type = MGR_TYPE;
          if(SUB_TYPE_PROBE_REQ == packetBuf->sub_type && sensor.state == LISTENING)   //receive Probe Request
          {
               if(ERR_OK ==getSensorCap(&sensor_client, packetBuf)) 
	       {
	             //valid sensor server, push state machine to next step
		     sensor.state = PROBED;
		     printf("PROBED_REQ\n");
	        }
	        else
	        {
	             //invalid sensor server, so keep the current state
		     sensor.state = LISTENING;
		     printf("listening\n");
                 }
       	}
        else if(SUB_TYPE_ASSOCIA_REQ == packetBuf->sub_type && sensor.state == PROBED)
        {
	      if(ERR_OK == isSensorCapSupport(&sensor, packetBuf))  //check whether clients cap can work with server?
	      {
	            //can support, push state machine to next step
		    sensor.state = ASSOCIATED;
		    printf("ASSOCIA_REQ\n");
	       }
	       else
	       {
		    //can't support, so disconnect and warn
		    sensor.state = LISTENING;						
		}
         }
	 else if(SUB_TYPE_AUTH_REQ == packetBuf->sub_type && sensor.state == ASSOCIATED)
	 {
		//handshake is OK, this sensor can start to work now
		sensor.state = AUTHENTICATED;
		printf("AUTH_REQ\n");
	 }
         else {return;}
	    
      }		
      else if(DAT_TYPE == packetBuf->type && sensor.state == CONNECTED)
      {
            sensor.frame_type = DAT_TYPE;
            //receive the data
      }
      else if(CTL_TYPE == packetBuf->type && sensor.state == CONNECTED)
      {
            sensor.frame_type = NONE_TYPE;  //just remove the CTL_REQ, means we get the ACK or it will be timeout
            timeout_cnt = 0;  //clear the timeout cnt
            printf("CTL_SYNC_RES\n");
            /*/sensor.frame_type = CTL_TYPE;            
            //receive control ACK
            if(SUB_TYPE_CTL_RES == packetBuf->sub_type)
            {
                memcpy(&(sensor.packet), packetBuf, sizeof(packetSensor_t));
            }
            else {return;}*/
      }
      else { return; }
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
  memset(&sensor_client, 0, sizeof(sensor_client));
  
  //initialize client sensor
  memset(&sensor.packet, 0, sizeof(sensor.packet));
  sensor.packet.len = BUFFER_LENGTH;
  
  //generate ID random
  sensor.id = 0xfe;
  //set cap
  sensor.cap = 0;
  sensor.cap &= ~(1<<7);  //9bit
  sensor.cap |= (1<<8);  //client, 
  //set state
  sensor.state = UNKNOWN;
  //set interval
  sensor.interval = 1;    // 1 sec 
  //set frame type
  sensor.frame_type = NONE_TYPE;

  /* starting the state machine */
  if(UNKNOWN == sensor.state)
  {
      sensor.state = LISTENING;
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

    nrf24l01_driver.ioctl(IOCTL_GET_MODE, &rf_mode, 1);
    if(rf_mode == RF_TX_MODE)
    {
        rf_mode = RF_RX_MODE;  //switch to rx mode
        nrf24l01_driver.ioctl(IOCTL_SET_MODE, &rf_mode, 1);
    }
	
#if USE_BROADCAST_SEND	 
    if(CTL_TYPE == sensor.frame_type)  //control frame has the highest priority, this is controlled by the remove server
    {                        
        if(sensor.state < CONNECTED)
        {
            continue;
        }           
        if(0 != bTimeout)  //this means timeout
        {
            //timeout
            if(timeout_cnt > 3*MAX_TIMEOUT_CNT)  // 1st time will cause one time timeout
            {
                sensor.frame_type = NONE_TYPE;
                sensor.state = LISTENING;
		timeout_cnt = 0;
                printf("timeout, lost connection!!\n");
		continue;
            }
            timeout_cnt ++;
            printf("ctl timeout: %d\n", timeout_cnt);
        }
	//parse command here and do some action, remember ACK
	//...
	sensor.packet.type = CTL_TYPE;
	sensor.packet.sub_type = SUB_TYPE_CTL_REQ;
	/* you can set the sensor capbility in the buffer */	    
	memset(&sensor.packet.buf, 0, BUFFER_LENGTH);
	sensor.interval = 3;  //waitfor 3s	 
	setSensorCap(&(sensor.packet), &sensor);
	rf_mode = RF_TX_MODE;  
	printf("CTL_REQ_SYNC\n");
    }
    else if(DAT_TYPE == sensor.frame_type)
    {
        sensor.frame_type = NONE_TYPE;
        if(CONNECTED > sensor.state)
        {
            continue;
        }
	//get the data here
	rf_mode = RF_RX_MODE;  //keep rx mode
	printf("got temperature!!\n");  		
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
            if(timeout_cnt > MAX_TIMEOUT_CNT)  //give up and re-try in the beginning
            {
                sensor.frame_type = NONE_TYPE;
                sensor.state = LISTENING;
		timeout_cnt = 0;
                printf("timeout, go to listening\n");
		continue;
            }
            timeout_cnt ++;
            printf("timeout: %d\n", timeout_cnt);
        }
                
        if(PROBED == sensor.state)
        {
            sensor.packet.type = MGR_TYPE;
	    sensor.packet.sub_type = SUB_TYPE_PROBE_RES;
	    /* set the server's sensor capbility and sent to client */
	    memset(&sensor.packet.buf, 0, BUFFER_LENGTH);
	    sensor.interval = 3;  // loop to check tx client to send associa_req
	    setSensorCap(&(sensor.packet), &sensor);  //tell the client server's cap.  
	    rf_mode = RF_TX_MODE;  //need switch to tx mode
	    printf("PROBE_RES\n");
        }
        else if(ASSOCIATED == sensor.state)
        {
            sensor.packet.type = MGR_TYPE;
            sensor.packet.sub_type = SUB_TYPE_ASSOCIA_RES;
            /* reset the sensor capbility follow the server's request */
            memset(&sensor.packet.buf, 0, BUFFER_LENGTH);
            sensor.interval = 3;  // loop to check tx client to send authen_req
    	    /* check client's cap here, if it isn't ok, reject */
            setSensorCap(&(sensor.packet), &sensor);  //notify the client what cap need it change.
            rf_mode = RF_TX_MODE;  //need switch to tx mode
            printf("ASSOCIA_RES\n");
        }
        else if(AUTHENTICATED == sensor.state)
        {
            sensor.packet.type = MGR_TYPE;
    	    sensor.packet.sub_type = SUB_TYPE_AUTH_RES;
    	    /* reset the sensor capbility follow the server's request */
    	    memset(&sensor.packet.buf, 0, BUFFER_LENGTH);	
    	    /* check client's cap here, if it isn't ok, reject, or prepare to receive */	
    	    sensor.state = CONNECTED;
    	    sensor.interval = 3;  // loop to check tx client to work	
    	    rf_mode = RF_TX_MODE;  //need switch to tx mode
    	    printf("AUTHEN_RES\n");
        }
        else if(CONNECTED == sensor.state)
        {
            sensor.frame_type = NONE_TYPE;
            rf_mode = RF_RX_MODE;  //needn't switch to tx mode
        
            //do nothing, just listening
            sensor.interval = 1;
            printf("CONNECTED\n");
            continue;
        }
        else    //if(LISTENING == sensor.state)   //for server, it means LISTENING
        {
            rf_mode = RF_RX_MODE;  //needn't switch to tx mode
        
            //do nothing, just listening
            sensor.interval = 1;
            printf("ERROR, listen...\n");
	    continue;  //re-loop while
        }    
    }
    else
    {
        //do nothing, just listening
        sensor.interval = 1;
        if(sensor.state == CONNECTED)
        {
            nSyncTime++;
            //if(nSyncTime%3 == 0)
            {
                /* every 3 sec to send a sync frame */
                sensor.frame_type = CTL_TYPE;   //this method isn't standard, need modify when the server side is ok
                nSyncTime = 0;
            }
        }
        printf("listen...\n");
	continue;  //re-loop while
    }
    if(RF_TX_MODE == rf_mode)
    {
        nrf24l01_driver.ioctl(IOCTL_SET_MODE, &rf_mode, 1);
	 
        packetbuf_copyfrom(&sensor.packet, BUFFER_LENGTH+3);
        broadcast_send(&bc);	

        rf_mode = RF_RX_MODE;  //after tx, switch to rx at once to wait for the response or next request
        nrf24l01_driver.ioctl(IOCTL_SET_MODE, &rf_mode, 1);
    }
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
