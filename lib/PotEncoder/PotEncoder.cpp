#include <PotEncoder.h>

PotEncoder::PotEncoder(uint8_t potPin, uint16_t pos_init, uint16_t pos_end)
: _potPin(potPin), _pos_init(pos_init), _pos_end(pos_end)
{
    pinMode(_potPin, INPUT);   
}

/**
 * @brief Returns the pure measurement of the potenciometer
 */
uint16_t PotEncoder::read() {
    return analogRead(_potPin); 
}

/**
 * @brief Returns the angle of the servo in radians
 */
float PotEncoder::getAngle() {
    uint16_t pot_read = read();
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