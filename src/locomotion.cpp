#include "locomotion.h"

Locomotion::Locomotion(float sample_time) : _motor1(IN1_1, IN2_1, A_1, B_1, sample_time), _motor2(IN1_2, IN2_2, B_2, A_2, sample_time), _translation_pid(3., 0., 30.), _rotation_pid(4., 0., 30.), _position({0., 0., 0.}), _target_position({0., 0., 0.}), _state(STOP), _sample_time(sample_time) {
    // Start in STOP state
    stop();
    setSpeeds(0., 0.);

    // Set the speeds limits
    _rotation_pid.setOutputLimits(-2*M_PI, 2*M_PI);
    _translation_pid.setOutputLimits(-500, 500);
}

void Locomotion::rotateFrom(const float d_theta) {
    // Update target orientation
    _target_position.theta += d_theta;

    // Enable only the rotation PID
    _rotation_pid.setMode(true);
    _translation_pid.setMode(false);

    _state = ROTATE;
}

void Locomotion::translateFrom(const float distance) {
    // Update target position
    _target_position.x += distance * cos(_target_position.theta);
    _target_position.y += distance * sin(_target_position.theta);

    // Enable the PIDs
    _translation_pid.setMode(true);
    _rotation_pid.setMode(true);

    _state = TRANSLATE;
}

void Locomotion::stop() {
    // Disable the PIDs
    _translation_pid.setMode(false);
    _rotation_pid.setMode(false);

    // Set outputs to 0
    _translation_pid.setOutput(0.);
    _rotation_pid.setOutput(0.);
    _translation_pid.setIntegral(0.);
    _rotation_pid.setIntegral(0.);

    if (_state == TRANSLATE)
        _state = STOPPING_TRANSLATE;
    else if (_state == ROTATE)
        _state = STOPPING_ROTATE;
    else
        _state = STOP;
}

void Locomotion::stop_translate() {
    _d_x = _target_position.x - _position.x;
    _d_y = _target_position.y - _position.y;
    _distance = sqrtf(_d_x*_d_x + _d_y*_d_y);

    if (fabsf(_distance) > translation_precision)
        translateFrom(_distance);
    else
        stop();
}

void Locomotion::stop_rotate() {
    _theta = _target_position.theta - _position.theta;

    if (fabsf(_theta) > rotation_precision)
        rotateFrom(_theta);
    else
        stop();
}

state_t Locomotion::run() {
    // Compute rotation speed according to the state
    switch(_state) {
        case STOP:
            break;
        case STOPPING_TRANSLATE:
            stop_translate();
            break;
        case STOPPING_ROTATE:
            stop_rotate();
            break;
        case ROTATE:
            // Stop if arrived
            if (fabsf(_target_position.theta - _position.theta) < rotation_precision) {
                _target_position.x = _position.x;
                _target_position.y = _position.y;
                stop_rotate();
            }
            // Update the PIDs
            else {
                _rotation_pid.setInput(_position.theta);
                _rotation_pid.setSetpoint(_target_position.theta);
            }
            break;
        case TRANSLATE:
            // Calculate distance and theta to move
            _d_x = _target_position.x - _position.x;
            _d_y = _target_position.y - _position.y;
            _distance = sqrtf(_d_x*_d_x + _d_y*_d_y);
            _theta = pi_modulo(atan2f(_d_y, _d_x) - _position.theta);
            // Go backward if it is faster
            if (fabsf(_theta) > M_PI_2) {
                _distance = -_distance;
                _theta = pi_modulo(_theta + M_PI);
            }

            // Stop if arrived
            if (fabsf(_distance) < translation_precision) {
                _target_position.theta = _position.theta;
                stop_translate();
            }
            // Update the PIDs
            else {
                _translation_pid.setInput(-_distance);
                _translation_pid.setSetpoint(0.);

                _rotation_pid.setInput(-_theta);
                _rotation_pid.setSetpoint(0.);
            }
            break;
    }
    _translation_pid.compute();
    _rotation_pid.compute();

    // Apply rotation speed
    setSpeeds(_translation_pid.getOutput(), _rotation_pid.getOutput());

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
