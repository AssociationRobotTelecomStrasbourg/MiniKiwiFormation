#include "ramp.h"

Ramp::Ramp(float max_speed, float d_speed) : _target_speed(0.), _speed(0.), _d_speed(d_speed) {}

void Ramp::setTargetSpeed(float target_speed) {
    _target_speed = target_speed;
}

float Ramp::computeSpeed() {
    // Limit acceleration
    if (_target_speed - _speed > _d_speed)
        _speed = _speed + _d_speed;
    else if (_target_speed - _speed < -_d_speed)
        _speed = _speed - _d_speed;

    // Saturate if speed go above limits
    if (_speed > _max_speed)
        _speed = _max_speed;
    else if (_speed < -_max_speed)
        _speed = -_max_speed;

    return _speed;
}
