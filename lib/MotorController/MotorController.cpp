#include <MotorController.h>
#include <Arduino.h>

/**
 * @brief Constructor of the MotorController class
 * 
 * This class is responsible for sending commands to the motor driver.
 * The compatible motor driver model is <model>.
 * 
 * @param intA The GPIO pin connected to input A of the motor driver
 * @param intB The GPIO pin connected to input B of the motor driver
 */
MotorController::MotorController(uint8_t intA, uint8_t intB)
: _intA(intA), _intB(intB)
{
    pinMode(_intA, OUTPUT);
    pinMode(_intB, OUTPUT);
}

/**
 * @brief Sends a motion command to the motor driver
 * 
 * This function outputs two analog/PWM signals to the motor driver.
 * The action parameter is constrained internally to the range -255 to 255.
 * 
 * - action = 0 - Motor stopped  
 * 
 * - action > 0 - Forward rotation 
 *  
 * - action < 0 - Reverse rotation  
 * 
 * The magnitude of the action determines the PWM duty cycle.
 * 
 * @param action The desired motor command, ranging from -255 to 255
 */
void MotorController::action(int action) {
    action = constrain(action, -255, 255);

    if (action == 0) {
        // Stop
        analogWrite(_intA, 0);
        analogWrite(_intB, 0);
    }
    else if (action > 0) {
        // Forward
        analogWrite(_intA, abs(action));     
        analogWrite(_intB, 0);          
    }
    else { // action < 0
        // Reverse
        analogWrite(_intA, 0);                
        analogWrite(_intB, abs(action));      
    }
}
