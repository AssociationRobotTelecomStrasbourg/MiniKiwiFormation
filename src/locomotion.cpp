#include "locomotion.h"

Locomotion::Locomotion(float sample_time) : _motor1(IN1_1, IN2_1), _motor2(IN1_2, IN2_2), _encoder1(A_1, B_1), _encoder2(B_2, A_2), _speed_pid1(60, 10, 0), _speed_pid2(60, 10, 0), _control1({0, 0, 0, 0, 0}), _control2({0, 0, 0, 0, 0}), _position({0, 0, 0}), _sample_time(sample_time/1000) {
    // Activate PID
    _speed_pid1.setMode(true);
    _speed_pid2.setMode(true);
}

void Locomotion::run() {
    // Update step
    _control1.last_step = _control1.step;
    _control2.last_step = _control2.step;
    _control1.step = _encoder1.read();
    _control2.step = _encoder2.read();

    // Compute position
    computeOdometry();

    // Compute speed
    _control1.speed = (_control1.step-_control1.last_step)/step_per_turn/_sample_time;
    _control2.speed = (_control2.step-_control2.last_step)/step_per_turn/_sample_time;

    // Set speed PID inputs
    _speed_pid1.setInput(_control1.speed);
    _speed_pid2.setInput(_control2.speed);

    // Compute PID
    _speed_pid1.compute();
    _speed_pid2.compute();

    // Get PWM
    _control1.pwm = _speed_pid1.getOutput();
    _control2.pwm = _speed_pid2.getOutput();

    // Apply PWM
    _motor1.setPwm(_control1.pwm);
    _motor2.setPwm(_control2.pwm);
}

void Locomotion::setSpeeds(const float speed1, const float speed2) {
    // Set speeds
    _control1.target_speed = speed1;
    _control2.target_speed = speed2;

    // Apply speeds
    _speed_pid1.setSetpoint(_control1.target_speed);
    _speed_pid2.setSetpoint(_control2.target_speed);
}

void Locomotion::computeOdometry() {
    // Calculate step moved during sample time
    float step1 = _control1.step - _control1.last_step;
    float step2 = _control2.step - _control2.last_step;

    // Calculate the equivalent translation and rotation moved during sample time
    float translation = (step1+step2)/2/step_per_turn*wheel_perimeter;
    float rotation = (-step1+step2)/step_per_turn*wheel_perimeter/center_distance;

    // Update position
    _position.x += translation*cos(_position.theta);
    _position.y += translation*sin(_position.theta);
    _position.theta += rotation;
}

const position_t* Locomotion::getPosition() const {
    return &_position;
}
