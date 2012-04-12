/**
 * \file
 *         sensor relative functions
 * \author
 *         nicolas chang <zhbsh.zhbsh@gmail.com>
 */

#include <stdio.h> /* For printf() */
#include "sensornode.h"

/*---------------------------------------------------------------------------*/
uint8_t getSensorCap(sensorObj_t * sensorObj, packetSensor_t  *packetSensor)
{
    if(!sensorObj || !packetSensor)  return ERR_FAIL;

    sensorObj->id = packetSensor->buf[0];
    sensorObj->cap = packetSensor->buf[1];
    sensorObj->interval = packetSensor->buf[2];
    sensorObj->state = packetSensor->buf[3];

    return ERR_OK;
}

uint8_t setSensorCap(packetSensor_t  *packetSensor, sensorObj_t * sensorObj)
{
    if(!packetSensor || !sensorObj)  return ERR_FAIL;
	
    packetSensor->buf[0] = sensorObj->id;
    packetSensor->buf[1] = sensorObj->cap;
    packetSensor->buf[2] = sensorObj->interval;
    packetSensor->buf[3] = sensorObj->state;	
    packetSensor->len = 4;

    return ERR_OK;	
}

uint8_t isSensorCapSupport(sensorObj_t * sensorObj, packetSensor_t  *packetSensor)
{
    if(!sensorObj || !packetSensor)  return ERR_FAIL;

    //here to check whther client sensor can support server's requeseted capability

	
    //if it is ok, reset the client sensor's capbility
   return getSensorCap(sensorObj, packetSensor);
}

uint8_t warnSensorBad()
{
    printf("****Warning!!!, Sensor is broken!!!****");
    return ERR_OK;
}

uint16_t captureAndPacketSensorTemp(sensorObj_t * sensorObj)
{
    if(!sensorObj)  return ERR_FAIL;
	
    //uint16_t temp = ds18b20_get_temp();
    uint16_t temp = 0x22EE;
    sensorObj->packet.buf[0] = sensorObj->id;
    sensorObj->packet.buf[1] = temp >> 8;   //high temp
    sensorObj->packet.buf[2] = temp;  //low temp	
    sensorObj->packet.len = 3;

    return ERR_OK;
}
/*---------------------------------------------------------------------------*/
