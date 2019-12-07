#include "locomotion.h"

Locomotion::Locomotion(float sample_time) : _motor1(IN1_1, IN2_1), _motor2(IN1_2, IN2_2), _encoder1(A_1, B_1), _encoder2(B_2, A_2), _pid1(60, 10, 0), _pid2(60, 10, 0), _ramp1(max_speed, acceleration*sample_time), _ramp2(max_speed, acceleration*sample_time), _control1({0, 0, 0, 0, 0}), _control2({0, 0, 0, 0, 0}), _position({0, 0, 0}), _sample_time(sample_time/1000) {
    // Activate PID
    _pid1.setMode(true);
    _pid2.setMode(true);
}

void Locomotion::run() {
    // Update step
    _control1.last_step = _control1.step;
    _control2.last_step = _control2.step;
    _control1.step = _encoder1.read();
    _control2.step = _encoder2.read();

    // Calculate step moved during sample time
    _d_step1 = _control1.step - _control1.last_step;
    _d_step2 = _control2.step - _control2.last_step;

    // Compute speed
    _control1.speed = _d_step1/step_per_turn/_sample_time;
    _control2.speed = _d_step2/step_per_turn/_sample_time;

    // Set speed PID inputs
    _pid1.setInput(_control1.speed);
    _pid2.setInput(_control2.speed);

    // Update speed PID setpoint
    _pid1.setSetpoint(_ramp1.computeSpeed());
    _pid2.setSetpoint(_ramp2.computeSpeed());

    // Compute PID
    _pid1.compute();
    _pid2.compute();

    // Get PWM
    _control1.pwm = _pid1.getOutput();
    _control2.pwm = _pid2.getOutput();

    // Apply PWM
    _motor1.setPwm(_control1.pwm);
    _motor2.setPwm(_control2.pwm);

    // Calculate the equivalent translation and rotation moved during sample time
    _d_translation = (_d_step1+_d_step2)/2/step_per_turn*wheel_perimeter;
    _d_rotation = (-_d_step1+_d_step2)/step_per_turn*wheel_perimeter/center_distance;

    // Update position
    _position.x += _d_translation*cos(_position.theta);
    _position.y += _d_translation*sin(_position.theta);
    _position.theta += _d_rotation;
}

void Locomotion::setSpeeds(const float translation_speed, const float rotation_speed) {
    // Set speeds
    _control1.target_speed = (translation_speed - rotation_speed*center_distance/2)/wheel_perimeter;
    _control2.target_speed = (translation_speed + rotation_speed*center_distance/2)/wheel_perimeter;

    // Set the target speeds for the ramp
    _ramp1.setTargetSpeed(_control1.target_speed);
    _ramp2.setTargetSpeed(_control2.target_speed);
}

const position_t* Locomotion::getPosition() const {
    return &_position;
}
