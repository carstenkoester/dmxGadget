#ifndef dmxGadget_h
#define dmxGadget_h

/*
Using board 'featheresp32' from platform in folder: C:\Users\carst\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.2.0
Using core 'esp32' from platform in folder: C:\Users\carst\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.2.0

Using library dmxGadget in folder: C:\Users\carst\OneDrive\Documents\Arduino\libraries\dmxGadget (legacy)
Using library Adafruit NeoPixel at version 1.12.5 in folder: C:\Users\carst\OneDrive\Documents\Arduino\libraries\Adafruit_NeoPixel 
Using library BLEConfig in folder: C:\Users\carst\OneDrive\Documents\Arduino\libraries\BLEConfig (legacy)
Using library Preferences at version 3.2.0 in folder: C:\Users\carst\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.2.0\libraries\Preferences 
Using library BLE at version 3.2.0 in folder: C:\Users\carst\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.2.0\libraries\BLE 
Using library Battery_18650_Stats at version 1.0.0 in folder: C:\Users\carst\OneDrive\Documents\Arduino\libraries\Battery_18650_Stats 
Using library WirelessDMXReceiver in folder: C:\Users\carst\OneDrive\Documents\Arduino\libraries\WirelessDMXReceiver (legacy)
Using library RF24 at version 1.4.11 in folder: C:\Users\carst\OneDrive\Documents\Arduino\libraries\RF24 
Using library SPI at version 3.2.0 in folder: C:\Users\carst\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.2.0\libraries\SPI 
Using library RingBuffer at version 1.0.5 in folder: C:\Users\carst\OneDrive\Documents\Arduino\libraries\RingBuffer 
Using library DMXNow in folder: C:\Users\carst\OneDrive\Documents\Arduino\libraries\DMXNow (legacy)
Using library ESP_NOW at version 3.2.0 in folder: C:\Users\carst\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.2.0\libraries\ESP_NOW 
Using library sACN at version 1.1.0 in folder: C:\Users\carst\OneDrive\Documents\Arduino\libraries\sACN 
Using library WiFi at version 3.2.0 in folder: C:\Users\carst\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.2.0\libraries\WiFi 
Using library Networking at version 3.2.0 in folder: C:\Users\carst\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.2.0\libraries\Network 
*/

#include <string>
#include <esp_task_wdt.h>

#include <Adafruit_NeoPixel.h>
#include <BLEConfig.h>
#include <Battery18650Stats.h>

#include <WirelessDMXReceiver.h>
#include <DMXNow_Receiver.h>
#include <sACN.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "StatusLED.h"

#include "dmxGadget-boards.h"


#define DEFAULT_SCAN_WDT_TIMEOUT               120   // 2 Minute watchdog timeout during scan phase
#define DEFAULT_RECEIVE_LOOP_WDT_TIMEOUT        20   // 30-second watchdog timeout after we've locked on a receiver

#define DEFAULT_BLE_CONFIG_DISABLE_SECONDS      60   // Disable BLE configuration after one minute uptime
#define DEFAULT_STATUS_SECONDS                   5   // Print status update every n seconds. Set to zero to disable.
#define DEFAULT_DMX_SLOT                         1

#define DEFAULT_RF24_WDMX_ID                  AUTO   // Auto

#define DEFAULT_DMXNOW_WIFI_CHANNEL              1
#define DEFAULT_DMXNOW_UNIVERSE                  1

#define DEFAULT_SACN_UNIVERSE                    1
#define DEFAULT_SACN_SSID              "Change Me"
#define DEFAULT_SACN_PASSPHRASE        "Change Me"

#define SACN_SSID_LENGTH                        32
#define SACN_PASSPHRASE_LENGTH                  32

#define RF24_PIN_CE                             25   // GPIO connected to nRF24L01 CE pin (module pin 3)
#define RF24_PIN_CSN                             4   // GPIO connected to nRF24L01 CSN pin (module pin 4)

#define NEOPIXEL_LED_CONFIG   NEO_RGB + NEO_KHZ800   // NEO_GRB / NEO_RGB / NEO_RGBW


class dmxGadget
{
  public:
    dmxGadget(char* name, dmxgadget_board_t board, unsigned int led_count = 0);
    void setup(std::vector<BLEConfigItem*> additionalBLEConfigItems);
    void loop();

    virtual uint8_t getValue(unsigned int address) const = 0;
    virtual void getValues(unsigned int startAddress, unsigned int length, void* buffer) const = 0;

    void fatalError(unsigned int error_code);

    static BLEConfig config;
    Adafruit_NeoPixel strip;
    Battery18650Stats* battery = nullptr;

    BLEUIntConfigItem dmxAddress;

    unsigned int bleConfigDisableSeconds = DEFAULT_BLE_CONFIG_DISABLE_SECONDS;
    unsigned int statusSeconds = DEFAULT_STATUS_SECONDS;

    dmxgadget_board_t _board;

  protected:
    static void onScan();                                         // Callback to be called during network search or channel scan
    void _reconfigureWDT(unsigned int timeout, bool panic = true);  

    unsigned int _ledCount;

    static StatusLED _statusLED;

    unsigned long outputLoopCount = 0;
    unsigned long previousMillis = 0;
    unsigned long previousOutputLoopCount = 0;

    float _getBatteryVolts() { return (battery ? battery->getBatteryVolts() : 0.0); };
    unsigned int _getBatteryChargeLevel() { return(battery ? battery->getBatteryChargeLevel() : 0); };

    std::string _appName;
};

class rf24DmxGadget : public dmxGadget {
  public:
    rf24DmxGadget(char* name, dmxgadget_board_t board, unsigned int led_count = 0);

    void setup(std::vector<BLEConfigItem*> userConfigItems = {});
    void loop();

    uint8_t getValue(unsigned int address) const { return receiver.getValue(address); };
    void getValues(unsigned int startAddress, unsigned int length, void* buffer) const { receiver.getValues(startAddress, length, buffer); };

    WirelessDMXReceiver receiver;
    BLEUIntConfigItem wdmxID;

  protected:
    unsigned long previousRxCount = 0;
    unsigned long previousInvalid = 0;
    unsigned long previousOverruns = 0;
    unsigned long previousSeqErrors = 0;
};

class DmxNowDmxGadget : public dmxGadget {
  public:
    DmxNowDmxGadget(char* name, dmxgadget_board_t board, unsigned int led_count = 0);

    void setup(std::vector<BLEConfigItem*> userConfigItems = {});
    void loop();

    uint8_t getValue(unsigned int address) const { return receiver.getValue(address); };
    void getValues(unsigned int startAddress, unsigned int length, void* buffer) const { receiver.getValues(startAddress, length, buffer); };

    static void onReceive();

    DMXNow_Receiver receiver;
    BLEUIntConfigItem universe;
    BLEUIntConfigItem channel;

  protected:
    unsigned long previousRxCount = 0;
    unsigned long previousInvalid = 0;
    unsigned long previousOverruns = 0;
    unsigned long previousSeqErrors = 0;
    unsigned long previousWrongUniverse = 0;

    static unsigned long _last_receive_millis;
};

class sACNDmxGadget : public dmxGadget {
  public:
    sACNDmxGadget(char* name, dmxgadget_board_t board, unsigned int led_count);

    void setup(std::vector<BLEConfigItem*> userConfigItems = {});
    void loop();

    uint8_t getValue(unsigned int address) const { return (_dmxBuffer[address - 1]); };
    void getValues(unsigned int startAddress, unsigned int length, void* buffer) const { memcpy(buffer, (void *)(_dmxBuffer + startAddress - 1), length); };

    const bool isLocked() const { return((recv.name()[0] != '\0') && (rxCount > 3)); }; // Wait for at least three valid packets before declaring victory

    BLEUIntConfigItem universe;
    BLEStringConfigItem ssid;
    BLEStringConfigItem passphrase;

    static WiFiUDP sacn;
    static Receiver recv;

  protected:
    static unsigned long rxCount;
    static unsigned long previousRxCount;

    static unsigned long _last_receive_millis;
    static uint8_t* _dmxBuffer;

    TaskHandle_t _sACNReceiveTask;
    esp_task_wdt_user_handle_t wdt_handle;
    static void _startSACNReceiveThread(void*);
    void _sACNReceiveLoop();    
};
#endif