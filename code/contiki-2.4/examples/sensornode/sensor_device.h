#ifndef  __SENSOR_DEVICE_H__
#define  __SENSOR_DEVICE_H__

typedef struct sensorAttr
{
    /* type */
    uint8_t type;
    /* identifier */
    uint8_t id;
    /* state */
    uint8_t state;
    /* power */
    uint8_t power;   
    /* loop interval */
    uint8_t interval;    
}sensorAttr_t;

typedef struct sensorDev
{
    /* attributes */
    sensorAttr_t* attrs;
    /* pivate data */
    void* platform_data;
    
    /* packet buffer*/
    uint8_t* packet_buf;
    /* packet len */
    uint16_t packet_len;

    /* set attributes */
    int (* setAttrs)(const void *attrs);
    /* get attributes */
    int (* getAttrs)(void *attrs);
    
}sensorDev_t;

typedef enum sensorState
{
    STOPED = 0,
    STARTED,
}sensorState_t;

typedef enum sensorType
{
    TEMPERATURE = 0,
    HUMIDITY, 
    MAXTYPE,    
}sensorType_t;

#endif  //__SENSOR_DEVICE_H__
