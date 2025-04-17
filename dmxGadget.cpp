#include <dmxGadget.h>

BLEConfig dmxGadget::config;
unsigned int dmxGadget::_statusLEDPin;
unsigned char dmxGadget::_statusLEDLevel = 0;
unsigned int dmxGadget::_statusLEDMillis = 0;
unsigned int dmxGadget::_scanMillis = 0;

dmxGadget::dmxGadget(char* name, unsigned int led_count, unsigned int defaultDmxAddress, unsigned int statusLEDPin):
  strip(led_count, NEOPIXEL_LED_PIN, NEOPIXEL_LED_CONFIG),
  battery(BAT_VOLT_PIN),
  dmxAddress("DMX Address", defaultDmxAddress)
{
  config.setAppName(name, true);
  _statusLEDPin = statusLEDPin;
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
  esp_task_wdt_reconfigure(&twdt_config);
#else
  esp_task_wdt_init(WDT_TIMEOUT, true);
#endif

  // Bring up the status LED
  pinMode(_statusLEDPin, OUTPUT);
  digitalWrite(_statusLEDPin, HIGH);

  // Start config
  config.begin();
  config.addItem(dmxAddress);
  config.advertise();

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

  config.loop();
  if (config.connected()) {  
    Serial.printf("Configuration is connected\n");
    config.handleConnected();
    Serial.printf("Configuration is disconnected\n");
  }

  unsigned long currentMillis = millis();

  if ((currentMillis > (bleConfigDisableSeconds * 1000)) && (config.active())) {
    Serial.printf("Disabling Bluetooth configuration\n");
    config.end();
  }
}

void dmxGadget::onReceive() {
  if (_statusLEDPin != 0) {
    // Pulse status LED when we're receiving
    unsigned long currentMillis = millis();

    if (currentMillis > _statusLEDMillis + 2000) {
      if (_statusLEDLevel == 0) {
        _statusLEDLevel = 255;
      } else {
        _statusLEDLevel = 0;
      }
      digitalWrite(_statusLEDPin, _statusLEDLevel);
      _statusLEDMillis = currentMillis;
    }
  }
}

void dmxGadget::onScan()
{
  if (_statusLEDPin != 0) {
    // Pulse status LED when we're receiving
    unsigned long currentMillis = millis();

    if (currentMillis > _scanMillis + 500) {
      digitalWrite(_statusLEDPin, !digitalRead(_statusLEDPin));
    }
    _scanMillis = currentMillis;
  }

  config.pollAndHandleConnected();
}

rf24DmxGadget::rf24DmxGadget(char* name, unsigned int led_count, wdmxID_t defaultWdmxID, unsigned int defaultDmxAddress, unsigned int statusLEDPin):
  dmxGadget(name, led_count, defaultDmxAddress, statusLEDPin),
  receiver(RF24_PIN_CE, RF24_PIN_CSN),
  wdmxID("Radio ID", defaultWdmxID)
{
};

void rf24DmxGadget::setup()
{
  dmxGadget::setup();
  config.addItem(wdmxID); // FIXME - Can we add this after we've called advertise()??

  // Start the receiver
  receiver.debug = true;
  receiver.begin((wdmxID_t)wdmxID.value(), this->onScan);
};

void rf24DmxGadget::loop()
{
  dmxGadget::loop();
  unsigned long currentMillis = millis();

  if (statusSeconds > 0) {
    if (currentMillis > (previousMillis + (statusSeconds * 1000))) {
      unsigned int rxCount = receiver.rxCount();
      unsigned int rxInvalid = receiver.rxInvalid();
      unsigned int overruns = receiver.rxOverruns();
      unsigned int seqErrors = receiver.rxSeqErrors();

      unsigned int deltaMillis = currentMillis-previousMillis;

      Serial.printf("WDMX ID %d (cfg %d), chan %d, Addr %d, Uptime: %d, RxCount: %d (+%d, %.2f/sec), invalid %d (+%d, %.2f/sec, %.2f%%), overruns %d (+%d, %.2f/sec, %.2f%%), seq err %d (+%d, %.2f/sec, %.2f%%), loop %d (+%d, %.2f/sec), bat %.2fV (%d%%), BLE act %d\n",
        receiver.getId(),
        (wdmxID_t)wdmxID.value(),
        receiver.getChannel(),
        dmxAddress.value(),
        currentMillis/1000,
        rxCount, rxCount-previousRxCount, ((float)(rxCount-previousRxCount)/deltaMillis*1000),
        rxInvalid, rxInvalid-previousInvalid, ((float)(rxInvalid-previousInvalid)/deltaMillis*1000), ((((float)(rxInvalid-previousInvalid)/(rxCount-previousRxCount))*100)),
        overruns, overruns-previousOverruns, ((float)(overruns-previousOverruns)/deltaMillis*1000), ((((float)(overruns-previousOverruns)/(rxCount-previousRxCount))*100)),
        seqErrors, seqErrors-previousSeqErrors, ((float)(seqErrors-previousSeqErrors)/deltaMillis*1000), ((((float)(seqErrors-previousSeqErrors)/(rxCount-previousRxCount))*100)),
        outputLoopCount, outputLoopCount-previousOutputLoopCount, ((float)(outputLoopCount-previousOutputLoopCount)/deltaMillis*1000),
        battery.getBatteryVolts(), battery.getBatteryChargeLevel(),
        config.active()
      );
      previousMillis = currentMillis;
      previousRxCount = rxCount;
      previousInvalid = rxInvalid;
      previousOverruns = overruns;
      previousSeqErrors = seqErrors;
      previousOutputLoopCount = outputLoopCount;
    }
  }
}



DmxNowDmxGadget::DmxNowDmxGadget(char* name, unsigned int led_count, uint8_t defaultChannel, unsigned int defaultDmxAddress, unsigned int statusLEDPin):
  dmxGadget(name, led_count, defaultDmxAddress, statusLEDPin),
  receiver(),
  channel("Wifi Channel", defaultChannel)
{
};

void DmxNowDmxGadget::setup()
{
  dmxGadget::setup();
  config.addItem(channel); // FIXME - Can we add this after we've called advertise()??

  // Start the receiver
  receiver.debug = true;
  receiver.begin(channel.value(), onReceive);
};

void DmxNowDmxGadget::loop()
{
  dmxGadget::loop();
  unsigned long currentMillis = millis();

  if (statusSeconds > 0) {
    if (currentMillis > (previousMillis + (statusSeconds * 1000))) {
      unsigned int rxCount = receiver.rxCount();
      unsigned int rxInvalid = receiver.rxInvalid();
      unsigned int overruns = receiver.rxOverruns();
      unsigned int seqErrors = receiver.rxSeqErrors();

      unsigned int deltaMillis = currentMillis-previousMillis;

      Serial.printf("DMXNow Addr %d, Uptime: %d, RxCount: %d (+%d, %.2f/sec), invalid %d (+%d, %.2f/sec, %.2f%%), overruns %d (+%d, %.2f/sec, %.2f%%), seq err %d (+%d, %.2f/sec, %.2f%%), loop %d (+%d, %.2f/sec), bat %.2fV (%d%%), BLE act %d\n",
        dmxAddress.value(),
        currentMillis/1000,
        rxCount, rxCount-previousRxCount, ((float)(rxCount-previousRxCount)/deltaMillis*1000),
        rxInvalid, rxInvalid-previousInvalid, ((float)(rxInvalid-previousInvalid)/deltaMillis*1000), ((((float)(rxInvalid-previousInvalid)/(rxCount-previousRxCount))*100)),
        overruns, overruns-previousOverruns, ((float)(overruns-previousOverruns)/deltaMillis*1000), ((((float)(overruns-previousOverruns)/(rxCount-previousRxCount))*100)),
        seqErrors, seqErrors-previousSeqErrors, ((float)(seqErrors-previousSeqErrors)/deltaMillis*1000), ((((float)(seqErrors-previousSeqErrors)/(rxCount-previousRxCount))*100)),
        outputLoopCount, outputLoopCount-previousOutputLoopCount, ((float)(outputLoopCount-previousOutputLoopCount)/deltaMillis*1000),
        battery.getBatteryVolts(), battery.getBatteryChargeLevel(),
        config.active()
      );
      previousMillis = currentMillis;
      previousRxCount = rxCount;
      previousInvalid = rxInvalid;
      previousOverruns = overruns;
      previousSeqErrors = seqErrors;
      previousOutputLoopCount = outputLoopCount;
    }
  }
}