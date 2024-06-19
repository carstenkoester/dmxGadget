#ifndef dmxGadget_h
#define dmxGadget_h

#include <esp_task_wdt.h>

#include <Adafruit_NeoPixel.h>
#include <WirelessDMXReceiver.h>
#include <BLEConfig.h>
#include <Battery18650Stats.h>

#define WDT_TIMEOUT                             60   // 60 seconds watchdog timeout

#define RF24_PIN_CE                             25   // GPIO connected to nRF24L01 CE pin (module pin 3)
#define RF24_PIN_CSN                             4   // GPIO connected to nRF24L01 CSN pin (module pin 4)

#define NEOPIXEL_LED_POWER                      33   // Power pin that needs to be pulled HIGH. Applicable to Arduino PropMaker.
#define NEOPIXEL_LED_PIN                        14   // GPIO connected to Neopixel LED Data
#define NEOPIXEL_LED_CONFIG   NEO_RGB + NEO_KHZ800   // NEO_GRB / NEO_RGB / NEO_RGBW

#define BAT_VOLT_PIN                           A13   // Battery voltage measure pin
#define STATUS_LED_PIN                          13   // Status indicator LED pin

class dmxGadget
{
  public:
    dmxGadget(char* name, unsigned int led_count, unsigned int defaultDmxAddress=1);
    void setup();
    void loop();

    WirelessDMXReceiver receiver;
    BLEConfig config;
    Adafruit_NeoPixel strip;
    Battery18650Stats battery;

    BLEUIntConfigItem dmxAddress;

    unsigned int bleConfigDisableSeconds = 300; // 5 Minutes
    unsigned int statusSeconds = 5;

  private:
    unsigned long outputLoopCount = 0;
    unsigned long previousMillis = 0;
    unsigned long previousOutputLoopCount = 0;
    unsigned long previousRxCount = 0;
    unsigned long previousErrors = 0;
};
#endif