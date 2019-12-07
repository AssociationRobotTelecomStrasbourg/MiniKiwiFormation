#ifndef RAMP_H
#define RAMP_H

#include <Arduino.h>

class Ramp {
public:
    // Initialisation of the Ramp class
    Ramp(float max_speed, float d_speed);

    // Set the target speed
    void setTargetSpeed(float target_speed);

    // Compute the corrected speed
    float computeSpeed();

private:
    float _target_speed;
    float _speed;
    float _max_speed;
    float _d_speed;
};

#endif
