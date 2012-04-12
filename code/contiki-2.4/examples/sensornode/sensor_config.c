
#include "sensor_config.h"

  uint8_t g_cmd_buffer[BUFFER_LENGTH];
  sensorDev_t g_sensor;
  sensorAttr_t g_sensor_attrs;
  process_event_t g_data_rec_event;
  struct etimer et;
  struct broadcast_conn bc;
  