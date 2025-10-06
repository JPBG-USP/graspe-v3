#include <MotorController.h>
#include <Arduino.h>

MotorController::MotorController(uint8_t intA, uint8_t intB)
: _intA(intA), _intB(intB)
{
    pinMode(_intA, OUTPUT);
    pinMode(_intB, OUTPUT);

    const uint32_t PWM_FREQ = 1000;   // 1 kHz 
    const uint8_t PWM_RES = 8;        // 8 bits resolution (0-255)

    ledcSetup(0, PWM_FREQ, PWM_RES);  // channel 0 -> _intA
    ledcAttachPin(_intA, 0);

    ledcSetup(1, PWM_FREQ, PWM_RES);  // channel 1 -> _intB
    ledcAttachPin(_intB, 1);
}

void MotorController::action(int action) {
    // Limita a ação para 0–255
    action = constrain(action, -255, 255);

    if (action == 0) {
        // Stop
        ledcWrite(0, 0);
        ledcWrite(1, 0);
    }
    else if (action > 0) {
        // Forward
        ledcWrite(0, action);     
        ledcWrite(1, 0);          
    }
    else { // action < 0
        // Reverse
        ledcWrite(0, 0);                
        ledcWrite(1, abs(action));      
    }
}
