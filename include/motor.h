#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
  // Initialisation of the Motor class
  Motor(const uint8_t pin1, const uint8_t pin2);

  // Get the pwm sent to the H bridge
  int16_t getPwm() const;

  // Set the pwm sent to the H bridge
  void setPwm(const int16_t pwm);

private:
  const uint8_t _pin1, _pin2;
  int16_t _pwm;
};

#endif
