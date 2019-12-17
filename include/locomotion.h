#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <Arduino.h>
#include <pid.h>
#include "motor.h"
#include "board.h"

// Odometry parameters
const float wheel_ratio = 1; // Ratio of the two wheel perimeter wheel1/wheel2
const float wheel_perimeter = 120; // in mm
const float center_distance = 120; // in mm

// Precision parameters
const float translation_precision = 5; // in mm
const float rotation_precision = 0.05; // in rad

// Speeds
const float max_rotation_speed = M_PI/10; // in rad/s
const float max_translation_speed = 500/10; // in mm

typedef struct {
    float x;
    float y;
    float theta;
} position_t;

typedef enum {STOP, ROTATE, TRANSLATE} state_t;

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
    position_t _position, _target_position;
    state_t _state;
    const float _sample_time;
    float _d_position1, _d_position2;
    float _d_translation, _d_rotation;
    float _distance, _theta;
    float _translation_speed, _rotation_speed;
};

float pi_modulo(float angle);

#endif
