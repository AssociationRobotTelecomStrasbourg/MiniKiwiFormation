#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <Arduino.h>
#include <pid.h>
#include "motor.h"
#include "board.h"

// Odometry parameters
const float step_ratio = 1; // Ratio of the two wheel perimeter wheel1/wheel2
const float wheel_perimeter = 120; // in mm
const float center_distance = 120; // in mm

// Precision parameters
const float translation_precision = 1;
const float rotation_precision = 0.005;

typedef struct {
    float x;
    float y;
    float theta;
} position_t;

typedef enum {STOP, STOPPING, ROTATE, TRANSLATE} state_t;

class Locomotion {
public:
    // Initialisation of the Locomotion class
    Locomotion(const float sample_time);

    // Rotate from d theta
    void rotateFrom(const float d_theta);

    // Translate for distance
    void translateFrom(const float distance);

    // Stop
    void stop();

    // Run the locomotion
    state_t run();

    // Set motor speed
    void setSpeeds(const float translation_speed, const float rotation_speed);

    // Get position
    const position_t* getPosition() const;

private:
    Motor _motor1, _motor2;
    PID _translation_pid, _rotation_pid;
    position_t _position, _target_position;
    state_t _state;
    const float _sample_time;
    float _d_position1, _d_position2;
    float _d_translation, _d_rotation;
    float _d_x, _d_y, _distance, _theta;
};

#endif
