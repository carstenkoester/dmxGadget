#ifndef _STATUS_LED_H
#define _STATUS_LED_H

class StatusLED
{
  public:
    StatusLED();
    void setup(const unsigned int pin);
    void on();
    void off();
    void blink(const unsigned int cycle_time);
    void breathe(const unsigned int cycle_time);

    // Blink a number of times to indicate a specific error. Note that unlike other functions in ths class, this one will block.
    void error_blink(const unsigned int num_blinks);

    unsigned int error_blink_cycle_time = 300;
    unsigned int error_blink_low_time = 2000;
    unsigned int error_blink_repeat_count = 2;

  protected:
    unsigned int _pin;
    unsigned char _level = 0;
    enum breathe_direction_t {
      DIRECTION_UP,
      DIRECTION_DOWN
    } _breathe_direction = DIRECTION_UP;
    unsigned long _last_millis = 0;
};

#endif