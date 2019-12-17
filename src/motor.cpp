#include "motor.h"

Motor::Motor(const uint8_t motor_pin1, const uint8_t motor_pin2, const uint8_t encoder_pin1, const uint8_t encoder_pin2, const float sample_time) : _encoder(encoder_pin1, encoder_pin2), _pid(kp, ki, kd), _c({0., 0., 0., 0., 0.}), _motor_pin1(motor_pin1), _motor_pin2(motor_pin2), _sample_time(sample_time) {
    setPwm(0);
    _pid.setOutputLimits(-limit, limit);
    _pid.setMode(true);
}

void Motor::setPwm(const float pwm) {
    _c.pwm = constrain(pwm, -limit, limit);
    if (pwm > 0) {
        analogWrite(_motor_pin1, 0);
        analogWrite(_motor_pin2, pwm);
    }
    else {
        analogWrite(_motor_pin1, -pwm);
        analogWrite(_motor_pin2, 0);
    }
}

void Motor::setTargetSpeed(const float target_speed) {
    _c.target_speed = target_speed;
}

float Motor::getDPosition() {
    return _c.d_position;
}

void Motor::run() {
    // Update position
    _c.d_position = -_c.position;
    _c.position = _encoder.read()/step_per_turn;
    _c.d_position += _c.position;

    // Compute speed
    _c.speed = _c.d_position/_sample_time;

    // Set speed PID inputs
    _pid.setInput(_c.speed);

    // Update speed PID setpoint
    _pid.setSetpoint(_c.target_speed);

    // Compute PID
    _pid.compute();

    // Get PWM
    _c.pwm = _pid.getOutput();

    // Apply PWM
    setPwm(_c.pwm);
}
