#include "dmxGadget-boards.h"

dmxgadget_board_t DMXGADGET_BOARD_HUZZAH32_PROPMAKER_RF24 = {
  .display_name = "Adafruit Huzzah32 with PropMaker",
  .pins = {
    .battery_voltage = 35,   // A13
    .status_led      = 13,
    .neopixel_power  = 33,
    .neopixel_data   = 14,
  }
};

dmxgadget_board_t DMXGADGET_BOARD_HUZZAH32_RF24 = {
  .display_name = "Adafruit Huzzah32 with RF24",
  .pins = {
    .battery_voltage = 35,   // A13
  }
};

dmxgadget_board_t DMXGADGET_BOARD_HUZZAH32 = {
  .display_name = "Adafruit Huzzah32",
  .pins = {
    .battery_voltage = 35,   // A13
    .status_led      = 13,
  }
};

dmxgadget_board_t DMXGADGET_BOARD_HUZZAH32_NO_STATUS = {
  .display_name = "Adafruit Huzzah32 with status LED disabled",
  .pins = {
    .battery_voltage = 35,   // A13
  }
};

dmxgadget_board_t DMXGADGET_BOARD_XIAO_ESP32S3 = {
  .display_name = "XIAO ESP32-S3",
  .pins = {
    .status_led      = 21,
  }
};

dmxgadget_board_t DMXGADGET_BOARD_XIAO_ESP32S3_NO_STATUS_LED = {
  .display_name = "XIAO ESP32-S3 without Status LED",
  .pins = {
  }
};

dmxgadget_board_t DMXGADGET_BOARD_SPARKLE_MOTION_MINI = {
  .display_name = "Adafruit Sparkle Motion Mini",
  .pins = {
    .status_led          = 12,
    .neopixel_status_led = 18,
    .neopixel_data       = 32,
    .button              = 0,
  }
};

dmxgadget_board_t DMXGADGET_BOARD_SPARKLE_MOTION = {
  .display_name = "Adafruit Sparkle Motion",
  .pins = {
    .status_led          = 4,
    .neopixel_status_led = 2,
    .neopixel_data       = 19,
    .button              = 0,
  }
};