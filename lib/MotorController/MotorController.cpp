#include <MotorController.h>
#include <Arduino.h>

MotorController::MotorController(uint8_t intA, uint8_t intB)
: _intA(intA), _intB(intB)
{
    pinMode(_intA, OUTPUT);
    pinMode(_intB, OUTPUT);
}

void MotorController::action(int action) {
    // Limit check (PWM on Arduino Uno is 0–255)
    if (abs(action) > 255) {
        //Serial.println("Error: action out of range (must be between -255 and 255)");
        action = constrain(action, -255, 255);  // clamp to valid range
    }
    //Serial.print("Sending action: ");
    //Serial.println(action);
    if (action == 0) {
        // Motor off
        digitalWrite(_intA, LOW);
        digitalWrite(_intB, LOW);
    }
    else if (action > 0) {
        // Forward
        analogWrite(_intA, action);
        digitalWrite(_intB, LOW);
    }
    else { // action < 0
        // Reverse
        digitalWrite(_intA, LOW);
        analogWrite(_intB, abs(action));
    }
}
