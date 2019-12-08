#include "locomotion.h"

Locomotion::Locomotion(float sample_time) : _motor1(IN1_1, IN2_1, A_1, B_1, sample_time), _motor2(IN1_2, IN2_2, B_2, A_2, sample_time), _position({0., 0., 0.}), _sample_time(sample_time) {}

void Locomotion::run() {
    // Run speed control
    _motor1.run();
    _motor2.run();

    _d_position1 = _motor1.getDPosition();
    _d_position2 = _motor2.getDPosition();

    // Calculate the equivalent translation and rotation moved during sample time
    _d_translation = (_d_position1+_d_position2)/2/wheel_perimeter;
    _d_rotation = (-_d_position1+_d_position2)/wheel_perimeter/center_distance;

    // Update position
    _position.x += _d_translation*cos(_position.theta);
    _position.y += _d_translation*sin(_position.theta);
    _position.theta += _d_rotation;
}

void Locomotion::setSpeeds(const float translation_speed, const float rotation_speed) {
    // Set the target speeds
    _motor1.setTargetSpeed(translation_speed - rotation_speed*center_distance/2/wheel_perimeter);
    _motor2.setTargetSpeed(translation_speed + rotation_speed*center_distance/2/wheel_perimeter);
}

const position_t* Locomotion::getPosition() const {
    return &_position;
}
