#include "locomotion.h"

Locomotion::Locomotion(float sample_time) : _motor1(IN1_1, IN2_1), _motor2(IN1_2, IN2_2), _encoder1(A_1, B_1), _encoder2(B_2, A_2), _speed_pid1(60, 10, 0), _speed_pid2(60, 10, 0), _control1({0, 0, 0, 0}), _control2({0, 0, 0, 0}), _sample_time(sample_time) {
    _speed_pid1.setMode(true);
    _speed_pid2.setMode(true);
}

void Locomotion::run() {
    // Update position
    _control1.last_step = _control1.step;
    _control2.last_step = _control2.step;
    _control1.step = _encoder1.read();
    _control2.step = _encoder2.read();

    // Compute speed
    _control1.speed = (_control1.step-_control1.last_step)*1000/step_per_turn/_sample_time;
    _control2.speed = (_control2.step-_control2.last_step)*1000/step_per_turn/_sample_time;

    // Set speed PID inputs
    _speed_pid1.setInput(_control1.speed);
    _speed_pid2.setInput(_control2.speed);

    // Compute PID
    _speed_pid1.compute();
    _speed_pid2.compute();

    // Apply PWM
    _motor1.setPwm(_speed_pid1.getOutput());
    _motor2.setPwm(_speed_pid2.getOutput());
}

void Locomotion::setSpeeds(const float speed1, const float speed2) {
    _control1.target_speed = speed1;
    _control2.target_speed = speed2;
    _speed_pid1.setSetpoint(_control1.target_speed);
    _speed_pid2.setSetpoint(_control2.target_speed);
}

// void Locomotion::computeOdometry() {
//
// }
