#include "dmxGadget-boards.h"

dmxgadget_board_t DMXGADGET_BOARD_HUZZAH32_PROPMAKER_RF24 = {
  .display_name = "Adafruit Huzzah32 with PropMaker",
  .pins = {
    .battery_voltage = 35,   // A13
    .status_led      = 13,
    .neopixel_power  = 33,
    .neopixel_data   = 14
  }
};

dmxgadget_board_t DMXGADGET_BOARD_XIAO_ESP32S3 = {
  .display_name = "XIAO ESP32-S3",
  .pins = {
    .status_led      = 21
  }
};

dmxgadget_board_t DMXGADGET_BOARD_XIAO_ESP32S3_NO_STATUS_LED = {
  .display_name = "XIAO ESP32-S3 without Status LED",
  .pins = {
  }
};
