#include <dmxGadget.h>

BLEConfig dmxGadget::config;
StatusLED dmxGadget::_statusLED;

unsigned long DmxNowDmxGadget::_last_receive_millis = 0;
unsigned long sACNDmxGadget::_last_receive_millis = 0;
uint8_t* sACNDmxGadget::_dmxBuffer = nullptr;

WiFiUDP sACNDmxGadget::sacn;
Receiver sACNDmxGadget::recv(sacn);

unsigned long sACNDmxGadget::rxCount = 0;
unsigned long sACNDmxGadget::previousRxCount = 0;

dmxGadget::dmxGadget(char* name, dmxgadget_board_t board, unsigned int led_count):
  strip(led_count, -1, NEOPIXEL_LED_CONFIG),
  dmxAddress("DMX Address", DEFAULT_DMX_SLOT)
{
  _ledCount = led_count;
  _appName = name;
  _board = board;

  if (_board.pins.battery_voltage >= 0) {
    battery = new Battery18650Stats(_board.pins.battery_voltage);
  }

  if (_board.pins.neopixel_data >= 0) {
    strip.setPin(_board.pins.neopixel_data);
  }
};

void dmxGadget::_reconfigureWDT(unsigned int timeout, bool panic) {
  #if ESP_ARDUINO_VERSION_MAJOR == 3
  esp_task_wdt_config_t twdt_config = {
      .timeout_ms = timeout * 1000,
      .idle_core_mask = 0x03,    // Bitmask of all cores
      .trigger_panic = panic,
  };
  esp_task_wdt_reconfigure(&twdt_config);
#else
  if (panic) {
    esp_task_wdt_init(timeout, true);
  } else {
    esp_task_wdt_deinit();
  }
#endif  
}

void dmxGadget::setup(std::vector<BLEConfigItem*> additionalBLEConfigItems)
{
  Serial.begin(115200);
  delay(500);
  Serial.printf("Starting up!\n");

  // Enable WDT and startus LED
  _reconfigureWDT(DEFAULT_SCAN_WDT_TIMEOUT);
  _statusLED.setup(_board.pins.status_led);

  // Start BLE Configuration
  config.begin(_appName.c_str());
  config.addItem(dmxAddress);

  for(typename std::vector<BLEConfigItem*>::iterator iter = additionalBLEConfigItems.begin(); iter != additionalBLEConfigItems.end(); ++iter) {
    config.addItem(**iter);
  }
  config.advertise();

  // Start the strip
  if ((_ledCount > 0) && (_board.pins.neopixel_data >= 0)) {
    strip.begin();
    strip.show();
  } else {
    Serial.printf("NeoPixel strip is zero length or has no pin, not initializing. Len %d pin %d\n", _ledCount, _board.pins.neopixel_data);
  }

  // Power the NeoPixel circuit, if it requires an amplifier circuit to be enabled (eg. Adafruit PropMaker).
  if (_board.pins.neopixel_power) {
    pinMode(_board.pins.neopixel_power, OUTPUT);
    digitalWrite(_board.pins.neopixel_power, HIGH);
  }

  Serial.printf("Board type: %s\n", _board.display_name);
  Serial.printf("BLE name:   %s\n", config.getName().c_str());
}

void dmxGadget::loop() {
  config.loop();

  outputLoopCount++;
  unsigned long currentMillis = millis();

  if ((currentMillis > (bleConfigDisableSeconds * 1000)) && (config.active() && !(config.connected()))) {
    Serial.printf("Disabling Bluetooth configuration\n");
    config.end();
  }
}

void dmxGadget::fatalError(unsigned int error_code) {
  _reconfigureWDT(1, false);

  Serial.printf("FATAL ERROR: %d\n", error_code);
  _statusLED.error_blink(error_code);
  Serial.printf("Restarting...\n");
  ESP.restart();
}

void dmxGadget::onScan()
{
  _statusLED.blink(200);
}

rf24DmxGadget::rf24DmxGadget(char* name, dmxgadget_board_t board, unsigned int led_count):
  dmxGadget(name, board, led_count),
  receiver(RF24_PIN_CE, RF24_PIN_CSN),
  wdmxID("Radio ID", DEFAULT_RF24_WDMX_ID)
{
};

void rf24DmxGadget::setup(std::vector<BLEConfigItem*> userConfigItems)
{
  std::vector<BLEConfigItem*> extraConfigItems{
    &wdmxID
  };
  extraConfigItems.insert(extraConfigItems.end(), userConfigItems.begin(), userConfigItems.end());
  dmxGadget::setup(extraConfigItems);

  // Start the receiver
  receiver.debug = true;
  receiver.begin((wdmxID_t)wdmxID.value(), this->onScan);

  _reconfigureWDT(DEFAULT_RECEIVE_LOOP_WDT_TIMEOUT);
};

void rf24DmxGadget::loop()
{
  dmxGadget::loop();
  unsigned long currentMillis = millis();

  // If we have received data in the last 1.5 seconds, blink happily. Otherwise, blink fast.
  if (currentMillis < receiver.lastRxMillis()+1500) {
    _statusLED.breathe(1500);
  } else {
    _statusLED.blink(80);
  }

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
        _getBatteryVolts(), _getBatteryChargeLevel(),
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

/*
 * DMXNow
 */
DmxNowDmxGadget::DmxNowDmxGadget(char* name, dmxgadget_board_t board, unsigned int led_count):
  dmxGadget(name, board, led_count),
  receiver(),
  universe("DMX Universe", DEFAULT_DMXNOW_UNIVERSE),
  channel("Wifi Channel", DEFAULT_DMXNOW_WIFI_CHANNEL)
{
};

void DmxNowDmxGadget::onReceive()
{
  _last_receive_millis = millis();
}

void DmxNowDmxGadget::setup(std::vector<BLEConfigItem*> userConfigItems)
{
  std::vector<BLEConfigItem*> extraConfigItems{
    &channel,
    &universe
  };
  extraConfigItems.insert(extraConfigItems.end(), userConfigItems.begin(), userConfigItems.end());
  dmxGadget::setup(extraConfigItems);

  // Start the receiver
  receiver.debug = true;
  receiver.begin(channel.value(), universe.value(), onReceive);

  Serial.printf("Waiting for receiver to lock on a transmitter\n");
  while (!receiver.isLocked()) {
    _statusLED.blink(200);
    delay(25);
  }
  Serial.printf("Found transmitter\n");

  _reconfigureWDT(DEFAULT_RECEIVE_LOOP_WDT_TIMEOUT);
};

void DmxNowDmxGadget::loop()
{
  dmxGadget::loop();
  unsigned long currentMillis = millis();

  // If we have received data in the last 3.5 seconds, blink happily. Otherwise, blink fast.
  if (currentMillis < _last_receive_millis+3500) {
    _statusLED.breathe(1500);
  } else {
    _statusLED.blink(80);
  }

  if (statusSeconds > 0) {
    if (currentMillis > (previousMillis + (statusSeconds * 1000))) {
      unsigned long rxCount = receiver.rxCount();
      unsigned long rxInvalid = receiver.rxInvalid();
      unsigned long overruns = receiver.rxOverruns();
      unsigned long seqErrors = receiver.rxSeqErrors();
      unsigned long wrongUniverse = receiver.rxWrongUniverse();

      unsigned int deltaMillis = currentMillis-previousMillis;
      const uint8_t* xmtr_mac = receiver.getXmtr();

      Serial.printf("DMXNow Channel %d, Univ %d, Addr %d, Lock %d, Xmtr %02x:%02x:%02x:%02x:%02x:%02x, RSSI %d, Uptime: %d, RxCount: %d (+%d, %.2f/sec), invalid %d (+%d, %.2f/sec, %.2f%%), seq err %d (+%d, %.2f/sec, %.2f%%), wrong univ %d (+%d, %.2f/sec, %.2f%%), loop %d (+%d, %.2f/sec), bat %.2fV (%d%%), BLE act %d\n",
        channel.value(),
        universe.value(),
        dmxAddress.value(),
        receiver.isLocked(),
        xmtr_mac[0], xmtr_mac[1], xmtr_mac[2], xmtr_mac[3], xmtr_mac[4], xmtr_mac[5],
        WiFi.RSSI(),
        currentMillis/1000,
        rxCount, rxCount-previousRxCount, ((float)(rxCount-previousRxCount)/deltaMillis*1000),
        rxInvalid, rxInvalid-previousInvalid, ((float)(rxInvalid-previousInvalid)/deltaMillis*1000), ((((float)(rxInvalid-previousInvalid)/(rxCount-previousRxCount))*100)),
        seqErrors, seqErrors-previousSeqErrors, ((float)(seqErrors-previousSeqErrors)/deltaMillis*1000), ((((float)(seqErrors-previousSeqErrors)/(rxCount-previousRxCount))*100)),
        wrongUniverse, wrongUniverse-previousWrongUniverse, ((float)(wrongUniverse-previousWrongUniverse)/deltaMillis*1000), ((((float)(wrongUniverse-previousWrongUniverse)/(rxCount-previousRxCount))*100)),
        outputLoopCount, outputLoopCount-previousOutputLoopCount, ((float)(outputLoopCount-previousOutputLoopCount)/deltaMillis*1000),
        _getBatteryVolts(), _getBatteryChargeLevel(),
        config.active()
      );
      previousMillis = currentMillis;
      previousRxCount = rxCount;
      previousInvalid = rxInvalid;
      previousOverruns = overruns;
      previousSeqErrors = seqErrors;
      previousWrongUniverse = wrongUniverse;
      previousOutputLoopCount = outputLoopCount;
    }
  }
}

/*
 * sACN
 */
sACNDmxGadget::sACNDmxGadget(char* name, dmxgadget_board_t board, unsigned int led_count):
  dmxGadget(name, board, led_count),
  universe("DMX Universe", DEFAULT_SACN_UNIVERSE),
  ssid("Wifi SSID", DEFAULT_SACN_SSID, SACN_SSID_LENGTH),
  passphrase("Wifi Passphrase", DEFAULT_SACN_PASSPHRASE, SACN_PASSPHRASE_LENGTH)
{
};

void sACNDmxGadget::setup(std::vector<BLEConfigItem*> userConfigItems)
{
  std::vector<BLEConfigItem*> extraConfigItems{
    &ssid,
    &passphrase,
    &universe
  };
  extraConfigItems.insert(extraConfigItems.end(), userConfigItems.begin(), userConfigItems.end());
  dmxGadget::setup(extraConfigItems);

  // Start the receiver
  WiFi.mode(WIFI_STA);

  while(WiFi.status() != WL_CONNECTED) {
    unsigned int waitCount = 0;
    Serial.printf("Connecting to WiFi SSID \"%s\"", ssid.value().c_str());
    WiFi.begin(ssid.value().c_str(), passphrase.value().c_str());

    while ((WiFi.status() != WL_CONNECTED) && (waitCount < 600)) {
      _statusLED.blink(100);
      delay(50);
      waitCount++;
    }
    Serial.printf("\n");

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("Connected to WiFi SSID \"%s\" with IP address %s\n", ssid.value().c_str(), WiFi.localIP().toString().c_str());
    } else {
      Serial.printf("Timed out; starting over\n");
    }
  }
  
  _dmxBuffer = recv.dmx();

  recv.begin(universe.value());

  Serial.printf("Listening for data on DMX universe %d\n", universe.value());

  _reconfigureWDT(DEFAULT_RECEIVE_LOOP_WDT_TIMEOUT);

  // Start the output task
  xTaskCreatePinnedToCore(
    _startSACNReceiveThread, /* Function to implement the task */
    "sACN Receive Thread",   /* Name of the task */
    10000,                   /* Stack size in words */
    this,                    /* Task input parameter */
    0,                       /* Priority of the task */
    &_sACNReceiveTask,       /* Task handle. */
    0                        /* Core where the task should run */
  );
  esp_task_wdt_add_user("sACN_Receiver", &wdt_handle);

  Serial.printf("Waiting for source to lock and first DMX packets to arrive\n");
  while (!isLocked()) {
    _statusLED.blink(200);
    delay(25);
  }
};

void sACNDmxGadget::loop()
{
  dmxGadget::loop();
  unsigned long currentMillis = millis();

  if (isLocked()) {
    _statusLED.breathe(1500);
  } else {
    _statusLED.blink(80);
  }

  if (statusSeconds > 0) {
    if (currentMillis > (previousMillis + (statusSeconds * 1000))) {
      unsigned int deltaMillis = currentMillis-previousMillis;

      Serial.printf("SSID \"%s\", sACN Univ %d, Addr %d, Source \"%s\", Locked %d, Rate %d, Uptime: %d, RxCount: %d (+%d, %.2f/sec), loop %d (+%d, %.2f/sec), bat %.2fV (%d%%), BLE act %d\n",
        ssid.value().c_str(),
        universe.value(),
        dmxAddress.value(),
        recv.name(),
        isLocked(),
        recv.framerate(),
        currentMillis/1000,
        rxCount, rxCount-previousRxCount, ((float)(rxCount-previousRxCount)/deltaMillis*1000),
        outputLoopCount, outputLoopCount-previousOutputLoopCount, ((float)(outputLoopCount-previousOutputLoopCount)/deltaMillis*1000),
        _getBatteryVolts(), _getBatteryChargeLevel(),
        config.active()
      );
      previousMillis = currentMillis;
      previousRxCount = rxCount;
      previousOutputLoopCount = outputLoopCount;
    }
  }
}

void sACNDmxGadget::_startSACNReceiveThread(void* _this)
{
  ((sACNDmxGadget*)_this)->_sACNReceiveLoop();
}

void sACNDmxGadget::_sACNReceiveLoop()
{
  while(true) {
    if (recv.update()) {
      _last_receive_millis = millis();
      rxCount++;
      _dmxBuffer = recv.dmx(); // Not sure if this pointer ever changes or if we could do that one in setup and be done with it?
      esp_task_wdt_reset_user(wdt_handle);
    }
  }
}
