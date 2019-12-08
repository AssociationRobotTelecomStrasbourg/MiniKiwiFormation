#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Encoder.h>
#include <pid.h>

// Encoder
const float step_per_turn = 1200.;

// PID
const float kp = 60.;
const float ki = 10.;
const float kd = 0.;

// Speed ramps
const float max_speed = 5.; // in turn/s
const float acceleration = 5.; // in turn/s^2

typedef struct {
    float position;
    float d_position;
    float speed;
    float ramp_speed;
    float target_speed;
    float pwm;
} control_t;

class Motor {
public:
    // Initialisation of the Motor class
    Motor(const uint8_t motor_pin1, const uint8_t motor_pin2, const uint8_t encoder_pin1, const uint8_t encoder_pin2, const float sample_time);

    // Set the pwm sent to the H bridge
    void setPwm(const float pwm);

    // Set the target speed in turn/s
    void setTargetSpeed(const float target_speed);

    // Get the position in turn
    float getDPosition();

    // Run the speed control every sample time
    void run();

private:
    Encoder _encoder;
    PID _pid;
    control_t _c;
    const uint8_t _motor_pin1, _motor_pin2;
    const float _sample_time;
};

#endif
