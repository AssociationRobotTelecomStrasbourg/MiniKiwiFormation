#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <Arduino.h>
#include <Encoder.h>
#include <pid.h>
#include "motor.h"
#include "board.h"

const float step_per_turn = 1200;

// Odometry parameters
const float step_ratio = 1; // Ratio of the two wheel perimeter wheel1/wheel2
const float wheel_perimeter = 120; // in mm
const float center_distance = 120; // in mm

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
    void setSpeeds(const float speed1, const float speed2);

    // Compute odometry
    void computeOdometry();

    // Get position
    const position_t* getPosition() const;

private:
    Motor _motor1, _motor2;
    Encoder _encoder1, _encoder2;
    PID _speed_pid1, _speed_pid2;
    control_t _control1, _control2;
    position_t _position;
    const float _sample_time;
};

#endif
