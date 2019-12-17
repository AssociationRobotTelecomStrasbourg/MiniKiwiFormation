#include "locomotion.h"

Locomotion::Locomotion(float sample_time) : _motor1(IN1_1, IN2_1, A_1, B_1, sample_time), _motor2(IN1_2, IN2_2, B_2, A_2, sample_time), _position({0., 0., 0.}), _target_position({0., 0., 0.}), _state(STOP), _sample_time(sample_time) {
    // Start in STOP state
    stop();
    setSpeeds(0., 0.);
}

void Locomotion::rotateFrom(const float d_theta) {
    _theta = d_theta;
    _state = ROTATE;
}

void Locomotion::translateFrom(const float distance) {
    _distance = distance;
    _state = TRANSLATE;
}

void Locomotion::stop() {
    _state = STOP;
}

state_t Locomotion::run() {
    // Compute rotation speed according to the state
    switch(_state) {
        case STOP:
            _translation_speed = 0;
            _rotation_speed = 0;
            break;
        case ROTATE:
            _translation_speed = 0;
            if (_theta > rotation_precision)
                _rotation_speed = max_rotation_speed;
            else if (_theta < -rotation_precision)
                _rotation_speed = -max_rotation_speed;
            else {
                _rotation_speed = 0;
                stop();
            }
            _theta -= _rotation_speed * _sample_time;
            break;
        case TRANSLATE:
            _rotation_speed = 0;
            if (_distance > translation_precision)
                _translation_speed = max_translation_speed;
            else if (_distance < -translation_precision)
                _translation_speed = -max_translation_speed;
            else {
                _translation_speed = 0;
                stop();
            }
            _distance -= _translation_speed * _sample_time;
            break;
    }

    // Apply rotation speed
    setSpeeds(_translation_speed, _rotation_speed);

    // Run speed control
    _motor1.run();
    _motor2.run();

    _d_position1 = _motor1.getDPosition();
    _d_position2 = _motor2.getDPosition()*wheel_ratio;

    // Calculate the equivalent translation and rotation moved during sample time
    _d_translation = (_d_position1+_d_position2)*wheel_perimeter/2;
    _d_rotation = (-_d_position1+_d_position2)*wheel_perimeter/center_distance;

    // Update position
    _position.x += _d_translation*cos(_position.theta);
    _position.y += _d_translation*sin(_position.theta);
    _position.theta += _d_rotation;

    return _state;
}

void Locomotion::setSpeeds(const float translation_speed, const float rotation_speed) {
    // Set the target speeds
    _motor1.setTargetSpeed((translation_speed - rotation_speed*center_distance/2)/wheel_perimeter);
    _motor2.setTargetSpeed((translation_speed + rotation_speed*center_distance/2)/wheel_perimeter*wheel_ratio);
}

const position_t* Locomotion::getPosition() const {
    return &_position;
}

float pi_modulo(float angle) {
    angle = fmodf(angle, 2 * M_PI);
    if (angle > M_PI) {
        return angle - 2 * M_PI;
    }
    if (angle < -M_PI) {
        return angle + 2 * M_PI;
    }
    return angle;
}
