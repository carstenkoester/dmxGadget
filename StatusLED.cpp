#include <Arduino.h>
#include "StatusLED.h"

#define RETURN_IF_NO_STATUS_LED \
  if (_pin == -1) { return; }

StatusLED::StatusLED()
{
}

void StatusLED::setup(const int16_t pin)
{
  _pin = pin;

  RETURN_IF_NO_STATUS_LED;

  pinMode(_pin, OUTPUT);
  off();
}

void StatusLED::on()
{
  RETURN_IF_NO_STATUS_LED;

  digitalWrite(_pin, 255);
}

void StatusLED::off()
{
  RETURN_IF_NO_STATUS_LED;

  digitalWrite(_pin, 0);
}

void StatusLED::blink(const unsigned int cycle_time)
{
  RETURN_IF_NO_STATUS_LED;
  unsigned long currentMillis = millis();

  if (currentMillis < _last_millis + cycle_time) {
    return;
  }

  if (_level == 0) {
    _level = 255;
  } else {
    _level = 0;
  }
  analogWrite(_pin, _level);
  _last_millis = currentMillis;
}

void StatusLED::breathe(const unsigned int cycle_time)
{
  RETURN_IF_NO_STATUS_LED;

  unsigned long currentMillis = millis();
  unsigned long deltaMillis = currentMillis-_last_millis;

  unsigned int delta = (deltaMillis * 256) / cycle_time; 
  if (delta == 0) {
    return;
  }

  if (_breathe_direction == DIRECTION_UP) {
    // Incrementing
    if (_level + delta > 255) {
      _level = 255;
      _breathe_direction = DIRECTION_DOWN;
    } else {
      _level += delta;
    }
  } else { 
    // Decrementing
    if (delta > _level) {
      _level = 0;
      _breathe_direction = DIRECTION_UP;
    } else {
      _level -= delta;
    }
  }

  analogWrite(_pin, _level);
  _last_millis = currentMillis;
}

void StatusLED::error_blink(const unsigned int num_blinks)
{
  RETURN_IF_NO_STATUS_LED;

  off();
  for (int j=0; j<error_blink_repeat_count; j++) {
    delay(error_blink_low_time);

    for (int i=0; i<num_blinks; i++) {
      on();
      delay(error_blink_cycle_time);
      off();
      delay(error_blink_cycle_time);
    }
  }
}