#include "contiki-net.h"
#include "contiki.h"
#include "cpu.h"
#include "sys/etimer.h"
#include "sys/rtimer.h"
#include "dev/leds.h"
#include "dev/uart.h"
#include "dev/watchdog.h"
#include "dev/serial-line.h"
#include "dev/sys-ctrl.h"
#include "net/rime/broadcast.h"
#include "net/ip/uip.h"
#include "net/ip/uip-udp-packet.h"
#include "net/ip/uiplib.h"
#include "net/ipv6/uip-ds6-route.h"
#include "net/ipv6/uip-ds6-nbr.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "rest-engine.h"

#include <time.h>

#include "spi-arch.h"
#include "spi.h"
#include "i2c.h"
#include "gpio.h"
#include "mpu9250.h"
#include "lps331ap.h"
#include "si1147.h"
#include "si7021.h"
#include "amn41122.h"
#include "adc121c021.h"
#include "rv3049.h"


#define SW_VERSION "1.0"
#define HW_VERSION "A"
#define SAMPLE "sample"

#define ADD_CHAR_IF_POSSIBLE(char) \
  if(strpos >= *offset && bufpos < preferred_size) { \
    buffer[bufpos++] = char; \
  } \
  ++strpos

#define ADD_STRING_IF_POSSIBLE(string, op) \
  tmplen = strlen(string); \
  if(strpos + tmplen > *offset) { \
    bufpos += snprintf((char *)buffer + bufpos, \
                       preferred_size - bufpos + 1, \
                       "%s", \
                       string \
                       + (*offset - (int32_t)strpos > 0 ? \
                          *offset - (int32_t)strpos : 0)); \
    if(bufpos op preferred_size) { \
      printf("res: BREAK at %s (%p)\n", string, resource); \
      break; \
    } \
  } \
  strpos += tmplen

static uint32_t MOTION_THRESHOLD = 60;

struct etimer my_etimer;
static time_t last_pir_time;

void pir_callback(uint8_t port, uint8_t pin);



PROCESS(app, "CoAP LEDs app");
AUTOSTART_PROCESSES(&app);

void
float_to_str(float value_f, char* buf) {
    int value_i = value_f; // get the integer part;
    float dec_f = value_f - value_i; // get the decimal part;
    int dec_i = (int)(dec_f * 100); // get two decimal places out of decimal part;
    
    //char *buf = (char *)malloc(sizeof(char)*6);
    snprintf(buf, 6, "%d.%02d", value_i, dec_i);
    
    return;
}

int
coap_parse_bool (void* request)
{
  int length;
  const char* payload = NULL;

  length = REST.get_request_payload(request, (const uint8_t**) &payload);
  if (length > 0) {
    if (strncmp(payload, "true", length) == 0) {
      return 1;
    } else if (strncmp(payload, "false", length) == 0) {
      return 0;
    } else {
      return -1;
    }
  } else {
    return -1;
  }
}


/*******************************************************************************
 * led0/Power
 ******************************************************************************/

static void
led0_power_get_handler(void *request,
                       void *response,
                       uint8_t *buffer,
                       uint16_t preferred_size,
                       int32_t *offset) {
  int length;

  length = snprintf((char*) buffer, REST_MAX_CHUNK_SIZE, "%s",
    ((leds_get()&LEDS_GREEN)==LEDS_GREEN)?"true":"false");

  REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  REST.set_response_payload(response, buffer, length);
}

static void
led0_power_post_handler(void *request,
                  void *response,
                  uint8_t *buffer,
                  uint16_t preferred_size,
                  int32_t *offset) {

  uint8_t b = coap_parse_bool(request);
  if (b == 1) {
    leds_on(LEDS_GREEN);
  } else if (b == 0) {
    leds_off(LEDS_GREEN);
  } else {
    REST.set_response_status(response, REST.status.BAD_REQUEST);
  }
}

/* A simple actuator example. Toggles the red led */
RESOURCE(coap_led0_power,
         "title=\"Green LED\";rt=\"Control\"",
         led0_power_get_handler,
         led0_power_post_handler,
         led0_power_post_handler,
         NULL);


/*******************************************************************************
 * led1/Power
 ******************************************************************************/

static void
led1_power_get_handler(void *request,
                       void *response,
                       uint8_t *buffer,
                       uint16_t preferred_size,
                       int32_t *offset) {
  int length;

  length = snprintf((char*) buffer, REST_MAX_CHUNK_SIZE, "%s",
    ((leds_get()&LEDS_RED)==LEDS_RED)?"true":"false");

  REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  REST.set_response_payload(response, buffer, length);
}

static void
led1_power_post_handler(void *request,
                  void *response,
                  uint8_t *buffer,
                  uint16_t preferred_size,
                  int32_t *offset) {

  uint8_t b = coap_parse_bool(request);
  if (b == 1) {
    leds_on(LEDS_RED);
  } else if (b == 0) {
    leds_off(LEDS_RED);
  } else {
    REST.set_response_status(response, REST.status.BAD_REQUEST);
  }
}

/* A simple actuator example. Toggles the red led */
RESOURCE(coap_led1_power,
         "title=\"Red LED\";rt=\"Control\"",
         led1_power_get_handler,
         led1_power_post_handler,
         led1_power_post_handler,
         NULL);


/*******************************************************************************
 * led2/Power
 ******************************************************************************/

static void
led2_power_get_handler(void *request,
                       void *response,
                       uint8_t *buffer,
                       uint16_t preferred_size,
                       int32_t *offset) {
  int length;

  length = snprintf((char*) buffer, REST_MAX_CHUNK_SIZE, "%s",
    ((leds_get()&LEDS_BLUE)==LEDS_BLUE)?"true":"false");

  REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  REST.set_response_payload(response, buffer, length);
}

static void
led2_power_post_handler(void *request,
                  void *response,
                  uint8_t *buffer,
                  uint16_t preferred_size,
                  int32_t *offset) {

  uint8_t b = coap_parse_bool(request);
  if (b == 1) {
    leds_on(LEDS_BLUE);
  } else if (b == 0) {
    leds_off(LEDS_BLUE);
  } else {
    REST.set_response_status(response, REST.status.BAD_REQUEST);
  }
}

/* A simple actuator example. Toggles the red led */
RESOURCE(coap_led2_power,
         "title=\"Blue LED\";rt=\"Control\"",
         led2_power_get_handler,
         led2_power_post_handler,
         led2_power_post_handler,
         NULL);


/*******************************************************************************
 * device/software/Version
 ******************************************************************************/

static void
device_software_version_get_handler(void *request,
                  void *response,
                  uint8_t *buffer,
                  uint16_t preferred_size,
                  int32_t *offset) {
  int length;
  char res[] = "%s";

  length = snprintf((char*) buffer, REST_MAX_CHUNK_SIZE, res, SW_VERSION);

  REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  REST.set_response_payload(response, buffer, length);
}

RESOURCE(coap_device_software_version,
         "title=\"device/software/Version\";rt=\"sw\"",
         device_software_version_get_handler,
         NULL,
         NULL,
         NULL);


/*******************************************************************************
 * device/hardware/Version
 ******************************************************************************/

static void
device_hardware_version_get_handler(void *request,
                  void *response,
                  uint8_t *buffer,
                  uint16_t preferred_size,
                  int32_t *offset) {
  int length;
  char res[] = "%s";

  length = snprintf((char*) buffer, REST_MAX_CHUNK_SIZE, res, HW_VERSION);

  REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  REST.set_response_payload(response, buffer, length);
}

RESOURCE(coap_device_hardware_version,
         "title=\"device/hardware/Version\";rt=\"hw\"",
         device_hardware_version_get_handler,
         NULL,
         NULL,
         NULL);


/*******************************************************************************
 * sensor/Accel/X
 ******************************************************************************/

static void
sensor_accel_get_handler(void *request,
                  void *response,
                  uint8_t *buffer,
                  uint16_t preferred_size,
                  int32_t *offset,
                  unsigned char axis) {
  int length;
  int16_t accel_axis;
  char buf[6];

  switch(axis) {
    case 'X':
        accel_axis = mpu9250_readSensor(MPU9250_ACCEL_XOUT_L, MPU9250_ACCEL_XOUT_H);
        break;
    case 'Y':
        accel_axis = mpu9250_readSensor(MPU9250_ACCEL_YOUT_L, MPU9250_ACCEL_YOUT_H);
        break;
    case 'Z':
        accel_axis = mpu9250_readSensor(MPU9250_ACCEL_ZOUT_L, MPU9250_ACCEL_ZOUT_H);
        break;
    default:
        accel_axis = mpu9250_readSensor(MPU9250_ACCEL_XOUT_L, MPU9250_ACCEL_XOUT_H);
  }

  float_to_str(accel_axis/2048.0, buf);
  length = snprintf((char*) buffer, REST_MAX_CHUNK_SIZE, "%s", buf);

  REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  REST.set_response_payload(response, buffer, length);

  return;
}

static void
sensor_accel_x_get_handler(void *request,
                  void *response,
                  uint8_t *buffer,
                  uint16_t preferred_size,
                  int32_t *offset) {
 sensor_accel_get_handler(request, response, buffer, preferred_size, offset, 'X');
}

RESOURCE(coap_sensor_accel_x,
         "title=\"sensor/Accel/X\";rt=\"sensor\"",
         sensor_accel_x_get_handler,
         NULL,
         NULL,
         NULL);


/*******************************************************************************
 * sensor/Accel/Y
 ******************************************************************************/

static void
sensor_accel_y_get_handler(void *request,
                  void *response,
                  uint8_t *buffer,
                  uint16_t preferred_size,
                  int32_t *offset) {
 sensor_accel_get_handler(request, response, buffer, preferred_size, offset, 'Y');
}

RESOURCE(coap_sensor_accel_y,
         "title=\"sensor/Accel/Y\";rt=\"sensor\"",
         sensor_accel_y_get_handler,
         NULL,
         NULL,
         NULL);


/*******************************************************************************
 * sensor/Accel/Z
 ******************************************************************************/

static void
sensor_accel_z_get_handler(void *request,
                  void *response,
                  uint8_t *buffer,
                  uint16_t preferred_size,
                  int32_t *offset) {
 sensor_accel_get_handler(request, response, buffer, preferred_size, offset, 'Z');
}

RESOURCE(coap_sensor_accel_z,
         "title=\"sensor/Accel/Z\";rt=\"sensor\"",
         sensor_accel_z_get_handler,
         NULL,
         NULL,
         NULL);


/*******************************************************************************
 * sensor/Pressure
 ******************************************************************************/

static void
sensor_pressure_get_handler(void *request,
                  void *response,
                  uint8_t *buffer,
                  uint16_t preferred_size,
                  int32_t *offset) {
  int length;
  uint32_t press = lps331ap_one_shot();
  char buf[6];
 
  float_to_str(press/4096.0, buf);
  length = snprintf((char*) buffer, REST_MAX_CHUNK_SIZE, "%s", buf);

  REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  REST.set_response_payload(response, buffer, length);

  return;
}

RESOURCE(coap_sensor_pressure,
         "title=\"sensor/Pressure\";rt=\"hw\"",
         sensor_pressure_get_handler,
         NULL,
         NULL,
         NULL);


/*******************************************************************************
 * sensor/Temperature
 ******************************************************************************/

static void
sensor_temperature_get_handler(void *request,
                  void *response,
                  uint8_t *buffer,
                  uint16_t preferred_size,
                  int32_t *offset) {
  int length;
  uint16_t temp = si7021_readTemp(TEMP_NOHOLD);
  char buf[6];
 
  float_to_str( ((float)(temp * 175.72)) / 65536 - 46.85, buf);
  length = snprintf((char*) buffer, REST_MAX_CHUNK_SIZE, "%s", buf);

  REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  REST.set_response_payload(response, buffer, length);

  return;
}

static void sensor_temperature_trigger(void);
EVENT_RESOURCE(coap_sensor_temperature, 
               "title=\"sensor/Temperature\";rt=\"hw\"",
               sensor_temperature_get_handler,
               NULL, 
               NULL, 
               NULL, 
               sensor_temperature_trigger);

static void 
sensor_temperature_trigger(void) {
  // do nothing
  leds_toggle(LEDS_GREEN);
  REST.notify_subscribers(&coap_sensor_temperature);
  return;
}
// RESOURCE(coap_sensor_temperature,
//          "title=\"sensor/Temperature\";rt=\"hw\"",
//          sensor_temperature_get_handler,
//          NULL,
//          NULL,
//          NULL);


/*******************************************************************************
 * sensor/Humidity
 ******************************************************************************/

static void
sensor_humidity_get_handler(void *request,
                  void *response,
                  uint8_t *buffer,
                  uint16_t preferred_size,
                  int32_t *offset) {
  int length;
  uint16_t humid = si7021_readHumd(RH_NOHOLD);
  char buf[6];
 
  float_to_str( ((float)(humid * 125)) / 65536 - 6, buf);
  length = snprintf((char*) buffer, REST_MAX_CHUNK_SIZE, "%s", buf);

  REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  REST.set_response_payload(response, buffer, length);

  return;
}

RESOURCE(coap_sensor_humidity,
         "title=\"sensor/Humidity\";rt=\"hw\"",
         sensor_humidity_get_handler,
         NULL,
         NULL,
         NULL);


/*******************************************************************************
 * sensor/Light
 ******************************************************************************/

static void
sensor_light_get_handler(void *request,
                  void *response,
                  uint8_t *buffer,
                  uint16_t preferred_size,
                  int32_t *offset) {
  int length;
  si1147_als_data_t als_data;
  si1147_als_force_read(&als_data);
  uint16_t light = als_data.vis.val;
  uint16_t ir = als_data.ir.val;

  //char buf[6];
  //float_to_str((float) light, buf);
  //float_to_str( ((float)(light * .47)) - 125, buf);
  length = snprintf((char*) buffer, REST_MAX_CHUNK_SIZE, "%u, %u", light, ir);

  REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  REST.set_response_payload(response, buffer, length);

  return;
}

RESOURCE(coap_sensor_light,
         "title=\"sensor/Light\";rt=\"hw\"",
         sensor_light_get_handler,
         NULL,
         NULL,
         NULL);


/*******************************************************************************
 * sensor/Motion
 ******************************************************************************/

static void
sensor_motion_get_handler(void *request,
                  void *response,
                  uint8_t *buffer,
                  uint16_t preferred_size,
                  int32_t *offset) {
  
  int length;
  char buf[100];
  char* output;
  struct tm timedate;
  time_t cur_time_epoch;
  rv3049_time_t current_time;

  rv3049_read_time(&current_time);

  sprintf(buf, "%u %u %u %u:%u:%u", current_time.days, current_time.month, current_time.year, current_time.hours, current_time.minutes, current_time.seconds);
  strptime(buf, "%d %m %Y %H:%M:%S", &timedate);
  timedate.tm_isdst = -1;
  cur_time_epoch = mktime(&timedate);

  printf("buf: %s\n", buf);
  printf("cur:%u, last:%u\n", (long)cur_time_epoch, (long)last_pir_time);

  if ( ((long)cur_time_epoch - (long)last_pir_time) > MOTION_THRESHOLD ) {
    output = "No motion within time window: ";
  } else {
    output = "Motion within time window: ";
  }

  length = snprintf((char*) buffer, REST_MAX_CHUNK_SIZE, "%s%u", output, MOTION_THRESHOLD);

  REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  REST.set_response_payload(response, buffer, length);

  return;
}

RESOURCE(coap_sensor_motion,
         "title=\"sensor/Motion\";rt=\"hw\"",
         sensor_motion_get_handler,
         NULL,
         NULL,
         NULL);





PROCESS_THREAD(app, ev, data) {
  PROCESS_BEGIN();

  i2c_init(GPIO_C_NUM, 5, // SDA
           GPIO_C_NUM, 4, // SCL
           I2C_SCL_FAST_BUS_SPEED);
  
  si1147_init(SI1147_FORCED_CONVERSION, SI1147_ALS_ENABLE);
  mpu9250_init();
  lps331ap_init();
  amn41122_init();
  amn41122_irq_enable(pir_callback);
  adc121c021_config();
  


  leds_on(LEDS_ALL);

  // CoAP + REST
  rest_init_engine();


  rest_activate_resource(&coap_led0_power,         "led0/Power");
  rest_activate_resource(&coap_led1_power,         "led1/Power");
  rest_activate_resource(&coap_led2_power,         "led2/Power");

  rest_activate_resource(&coap_device_software_version, "device/software/Version");
  rest_activate_resource(&coap_device_hardware_version, "device/hardware/Version");

  rest_activate_resource(&coap_sensor_accel_x, "sensor/Accel/X");
  rest_activate_resource(&coap_sensor_accel_y, "sensor/Accel/Y");
  rest_activate_resource(&coap_sensor_accel_z, "sensor/Accel/Z");

  rest_activate_resource(&coap_sensor_pressure,     "sensor/Pressure");
  rest_activate_resource(&coap_sensor_temperature,  "sensor/Temperature");
  rest_activate_resource(&coap_sensor_humidity,     "sensor/Humidity");
  rest_activate_resource(&coap_sensor_light,        "sensor/Light");
  rest_activate_resource(&coap_sensor_motion,       "sensor/Motion");


  while (1) {
    //etimer_set(&my_etimer, 5*CLOCK_SECOND);
    PROCESS_WAIT_EVENT();
    //sensor_temperature_trigger();
  }

  PROCESS_END();
}


void pir_callback(uint8_t port, uint8_t pin) {
  INTERRUPTS_DISABLE();

  char buf[100];
  struct tm time_date;
  rv3049_time_t pir_time;

  rv3049_read_time(&pir_time);
  
  sprintf(buf, "%u %u %u %u:%u:%u", pir_time.days, pir_time.month, pir_time.year, pir_time.hours, pir_time.minutes, pir_time.seconds);
  
  strptime(buf, "%d %m %Y %H:%M:%S", &time_date);
  
  time_date.tm_isdst = -1;

  last_pir_time = mktime(&time_date);

  INTERRUPTS_ENABLE();

  return;
}