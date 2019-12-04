#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <Arduino.h>
#include <Encoder.h>
#include <pid.h>
#include "motor.h"
#include "board.h"

const float step_per_turn = 1200;

typedef struct {
    float step;
    float last_step;
    float speed;
    float target_speed;
} control_t;

class Locomotion {
public:
    // Initialisation of the Locomotion class
    Locomotion(const float sample_time);

    // Run the locomotion
    void run();

    // Règle la vitesse des moteurs
    void setSpeeds(const float speed1, const float speed2);

private:
    Motor _motor1, _motor2;
    Encoder _encoder1, _encoder2;
    PID _speed_pid1, _speed_pid2;
    control_t _control1, _control2;
    const float _sample_time;
};

#endif
