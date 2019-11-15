#include "motor.h"

Motor::Motor(const uint8_t pin1, const uint8_t pin2) :
_pin1(pin1), _pin2(pin2) , _pwm(0) {
    analogWrite(_pin1, 0);
    analogWrite(_pin2, 0);
}

int16_t Motor::get_pwm() const {
    return _pwm;
}

void Motor::set_pwm(const int16_t pwm) {
    _pwm = constrain(pwm, -255, 255);
    if (pwm > 0) {
        analogWrite(_pin1, 0);
        analogWrite(_pin2, pwm);
    }
    else {
        analogWrite(_pin1, -pwm);
        analogWrite(_pin2, 0);
    }
}
