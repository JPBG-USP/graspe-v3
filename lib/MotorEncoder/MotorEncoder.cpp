#include "MotorEncoder.h"

/**
 * @brief Construct a new Motor Encoder:: Motor Encoder object
 * @param potPin The analog pin connected to the potentiometer
 * @param pot_init The ADC value corresponding to 0 radians
 * @param pot_end The ADC value corresponding to PI radians
 * @param std The standard deviation of the measurement noise for the Kalman filter
 * @param Q The process noise covariance for the Kalman filter
 * @param P_init The initial estimation error covariance for the Kalman filter
 */
MotorEncoder::MotorEncoder(uint8_t potPin, uint16_t pot_init, uint16_t pot_end, float angle_init, float angle_end, float std, float Q, float P_init)
    : _potPin(potPin), _pot_init(pot_init), _pot_end(pot_end), _angle_init(angle_init), _angle_end(angle_end),
      _std(std), _r(std*std), _q(Q), _x(0.0), _p(P_init)
{
    pinMode(_potPin, INPUT);
    _slope = (_angle_end-_angle_init)/(_pot_end-_pot_init);
    _displacement = _angle_init;
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
float MotorEncoder::getAngle() const{
    uint16_t pot_read = readPot();
    float angle;

    angle = ((pot_read-_pot_init)*_slope)+_displacement;

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

