# To-Do

  - OTA updates using https://github.com/Raghav117/bluetooth_ota_firmware_update
  - When Bluetooth configuration exits, board should reboot
  - Can we make LED count BT configurable as well? It's currently passed to the constructor -- what would change?
  - Provide methods to:
      disableButton()
      setButton(int)
      disableStatusLED()
      setStatusLED(int)
    which, when called after constructor but before begin()/setup(), override using these elements. This way we don't need a separate board type for non-button/non-status-led versions.
      - Set pin during constructor
      - Allow overriding pin before setup()
      - in setup(), use local variable rather than board.* reference

  - Sparkle Motion (and others):
      - Use button for configuration mode rather than power-on timer. If button pressed for 3 seconds, enter configuration mode (on button release). If button pressed during configuration mode, reset.
      - Use NeoPixel status LED
          - Green (pulsing): Receiving
          - Red: Scanning / searching (possibly also pulse/blink while doing so)
          - Yellow: Configuration button pressed for 3 sec
          - Purple: In configuration mode