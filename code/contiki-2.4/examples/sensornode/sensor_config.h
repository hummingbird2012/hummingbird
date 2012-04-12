#ifndef _SENSOR_CONFIG_H_
#define _SENSOR_CONFIG_H_

/**
 * \file
 *         sensor config
 * \author
 *         Nicolas Chang <zhbsh.zhbsh@gmail.com>
 */

#include "net/rime.h"
#include "sensor_device.h"

#define TRUE		1
#define FALSE		0

#define BUFFER_LENGTH  64

  /* global buffer for data or cmd */
  extern uint8_t g_cmd_buffer[BUFFER_LENGTH];

  /* global varients */
  extern sensorAttr_t g_sensor_attrs;
  extern sensorDev_t g_sensor;
  extern process_event_t g_data_rec_event;
  extern struct etimer et;
  extern struct broadcast_conn bc;

#endif // _SENSOR_CONFIG_H_