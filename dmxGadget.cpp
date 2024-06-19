#include <dmxGadget.h>

dmxGadget::dmxGadget(char* name, unsigned int led_count, unsigned int defaultDmxAddress):
  receiver(RF24_PIN_CE, RF24_PIN_CSN, STATUS_LED_PIN),
  config(name),
  strip(led_count, NEOPIXEL_LED_PIN, NEOPIXEL_LED_CONFIG),
  battery(BAT_VOLT_PIN),
  dmxAddress("dmx_address", defaultDmxAddress)
{
};

void dmxGadget::setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting up!");

  // Enable WDT
#if ESP_ARDUINO_VERSION_MAJOR == 3
  esp_task_wdt_config_t twdt_config = {
      .timeout_ms = WDT_TIMEOUT * 1000,
      .idle_core_mask = 0x03,    // Bitmask of all cores
      .trigger_panic = true,
  };
  esp_task_wdt_init(&twdt_config);
#else
  esp_task_wdt_init(WDT_TIMEOUT, true);
#endif

  // Bring up the status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);

  // Start config
  config.begin();
  config.addItem(dmxAddress);
  //config.advertise();

  // Start the receiver
  receiver.debug = true;
  receiver.begin();

  // Start the strip
  if (strip.numPixels() > 0) {
    strip.begin();
    strip.show();
  } else {
    Serial.println("NeoPixel strip is zero length, not initializing");
  }

  // Power PropMaker wing NeoPixel circuit
  pinMode(NEOPIXEL_LED_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_LED_POWER, HIGH);
}

void dmxGadget::loop() {
  outputLoopCount++;

  unsigned long currentMillis = millis();

  if (statusSeconds > 0) {
    if (currentMillis > (previousMillis + (statusSeconds * 1000))) {
      unsigned int rxCount = receiver.rxCount();
      unsigned int rxErrors = receiver.rxErrors();

      unsigned int deltaMillis = currentMillis-previousMillis;

      Serial.printf("Addr %d, Uptime: %d, RxCount: %d (+%d, %.2f/sec), errCount %d (+%d, %.2f/sec, %.2f%%), loop iterations %d (+%d, %.2f/sec), Bat Voltage: %.2fV (%d%%), BLE active %d, connected: %d\n",
        dmxAddress.value(),
        currentMillis/1000,
        rxCount, rxCount-previousRxCount, ((float)(rxCount-previousRxCount)/deltaMillis*1000),
        rxErrors, rxErrors-previousErrors, ((float)(rxErrors-previousErrors)/deltaMillis*1000), ((((float)(rxErrors-previousErrors)/(rxCount-previousRxCount))*100)),
        outputLoopCount, outputLoopCount-previousOutputLoopCount, ((float)(outputLoopCount-previousOutputLoopCount)/deltaMillis*1000),
        battery.getBatteryVolts(), battery.getBatteryChargeLevel(),
        config.active(), BLE.connected()
      );
      previousMillis = currentMillis;
      previousRxCount = rxCount;
      previousErrors = rxErrors;
      previousOutputLoopCount = outputLoopCount;
    }
  }

  if ((currentMillis > (bleConfigDisableSeconds * 1000)) && (config.active())) {
    Serial.printf("Disabling Bluetooth configuration\n");
    config.end();
  }
}