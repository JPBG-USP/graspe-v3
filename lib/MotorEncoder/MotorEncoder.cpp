#include "MotorEncoder.h"

/**
 * @brief Construct a new Motor Encoder:: Motor Encoder object
 * @param potPin The analog pin connected to the potentiometer
 * @param pos_init The ADC value corresponding to 0 radians
 * @param pos_end The ADC value corresponding to PI radians
 * @param std The standard deviation of the measurement noise for the Kalman filter
 * @param Q The process noise covariance for the Kalman filter
 * @param P_init The initial estimation error covariance for the Kalman filter
 */
MotorEncoder::MotorEncoder(uint8_t potPin, uint16_t pos_init, uint16_t pos_end, float std, float Q, float P_init)
    : _potPin(potPin), _pos_init(pos_init), _pos_end(pos_end),
      _std(std), _r(std*std), _q(Q), _x(0.0), _p(P_init)
{
    pinMode(_potPin, INPUT);
}

/**
 * @brief Read the raw ADC value from the potentiometer
 * @return uint16_t The raw ADC value
 */
uint16_t MotorEncoder::readPot() const {
    return analogRead(_potPin);
}

/**
 * @brief Reads the potentiometer value and converts it to a radians.
 *
 * This function reads the raw analog input from the potentiometer pin,
 * applies calibration constants, and returns a floating-point value.
 *
 * @return A float between 0.0 and PI representing the potentiometer angle in radians.
 */
float MotorEncoder::getAngle() const {
    uint16_t pot_read = readPot();
    float angle;

    if (_pos_init < _pos_end) {
        if (pot_read < _pos_init) {
            angle = 0.0f;
        } else if (pot_read > _pos_end) {
            angle = PI;
        } else {
            angle = (float)(pot_read - _pos_init) / (float)(_pos_end - _pos_init) * PI;
        }
    } else {
        if (pot_read > _pos_init) {
            angle = 0.0f;
        } else if (pot_read < _pos_end) {
            angle = PI;
        } else {
            angle = (float)(_pos_init - pot_read) / (float)(_pos_init - _pos_end) * PI;
        }
    }

    return angle;
}

/**
 * @brief Returns the filtered angle using a Kalman filter
 * @return float The filtered angle in radians
 */
float MotorEncoder::getFilteredAngle() {
    float z = getAngle();
    
    // Initialize Kalman
    if (!_initialized)
    {
        _x = z;
        _initialized = true;
    }

    // Prediction update
    _p = _p + _q;

    // Measurement update
    float k = _p / (_p + _r);
    _x = _x + k * (z - _x);
    _p = (1 - k) * _p;

    return _x;
}

