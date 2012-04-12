#ifndef  __SENSOR_NODE_H__
#define  __SENSOR_NODE_H__

#define ERR_OK    0
#define ERR_FAIL    -1

/* default none type*/
#define NONE_TYPE 0x00
/* management type*/
#define MGR_TYPE	   0x01
/* control type*/
#define CTL_TYPE     0x02
/* data type*/
#define DAT_TYPE	    0x03

/* probe request */
#define SUB_TYPE_PROBE_REQ    0x00
/* probe response */
#define SUB_TYPE_PROBE_RES    0x01
/* association request */
#define SUB_TYPE_ASSOCIA_REQ    0x02
/* association response */
#define SUB_TYPE_ASSOCIA_RES    0x03
/* authentication request */
#define SUB_TYPE_AUTH_REQ    0x04
/* authentication response */
#define SUB_TYPE_AUTH_RES    0x05
/* disassociation */
#define SUB_TYPE_DISASSOC   0x06

/* control request */
#define SUB_TYPE_CTL_REQ   0x00
/* disassociation */
#define SUB_TYPE_CTL_RES   0x01

/* data */
#define SUB_TYPE_DATA  0x00

#define BUFFER_LENGTH  10
typedef struct packetSensor {
  uint8_t type;
  uint8_t sub_type;
  uint8_t len;
  uint8_t buf[BUFFER_LENGTH];
}packetSensor_t;

typedef struct SensorObj
{
    /*    
    byte[1]: HW cap
         bit0: cs, 0--server, 1--client;  
	 bit1-2: precision, 00-->11---9-->12;
	 bit3--7: reserved.
    */
    uint8_t cap;
    uint8_t id;
    uint8_t interval;    
    uint8_t state;
    uint8_t frame_type;

    packetSensor_t packet;
}sensorObj_t;

extern uint8_t getSensorCap(sensorObj_t * sensorObj, packetSensor_t  *packetSensor);
extern uint8_t setSensorCap(packetSensor_t  *packetSensor, sensorObj_t * sensorObj);
extern uint8_t isSensorCapSupport(sensorObj_t * sensorObj, packetSensor_t  *packetSensor);
extern uint8_t warnSensorBad();
extern uint16_t captureAndPacketSensorTemp(sensorObj_t * sensorObj);

#endif  //__SENSOR_NODE_H__
