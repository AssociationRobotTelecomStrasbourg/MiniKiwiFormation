#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <Arduino.h>
#include <Encoder.h>
#include <pid.h>
#include "motor.h"
#include "ramp.h"
#include "board.h"

// Encoder
const float step_per_turn = 1200;

// Odometry parameters
const float step_ratio = 1; // Ratio of the two wheel perimeter wheel1/wheel2
const float wheel_perimeter = 120; // in mm
const float center_distance = 120; // in mm

// Speed ramps
const float max_speed = 100; // in mm
const float acceleration = 100; // in mm

typedef struct {
    float step;
    float last_step;
    float speed;
    float target_speed;
    float pwm;
} control_t;

typedef struct {
    float x;
    float y;
    float theta;
} position_t;

class Locomotion {
public:
    // Initialisation of the Locomotion class
    Locomotion(const float sample_time);

    // Run the locomotion
    void run();

    // Set motor speed
    void setSpeeds(const float translation_speed, const float rotation_speed);

    // Get position
    const position_t* getPosition() const;

private:
    Motor _motor1, _motor2;
    position_t _position;
    const float _sample_time;
    float _d_step1, _d_step2;
    float _d_translation, _d_rotation;
};

#endif
