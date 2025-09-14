#ifndef dmxGadget_boards_h
#define dmxGadget_boards_h

#include "Arduino.h"

typedef struct {
  int16_t battery_voltage {-1};
  int16_t status_led {-1};
  int16_t neopixel_power {-1};
  int16_t neopixel_data {-1};
} dmxgadget_board_pin_t;

typedef struct {
  char display_name[128];
  dmxgadget_board_pin_t pins;

} dmxgadget_board_t;

extern dmxgadget_board_t DMXGADGET_BOARD_HUZZAH32_PROPMAKER_RF24;
extern dmxgadget_board_t DMXGADGET_BOARD_HUZZAH32_RF24;
extern dmxgadget_board_t DMXGADGET_BOARD_XIAO_ESP32S3;
extern dmxgadget_board_t DMXGADGET_BOARD_XIAO_ESP32S3_NO_STATUS_LED;

#endif