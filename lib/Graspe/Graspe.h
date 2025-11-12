#ifndef _GRASPE_GPIO_H_
#define _GRASPE_GPIO_H_

// GPIO Pin definitions
#define GRASPE_GPIO_VERSION "3.1.0"

// LEDs pins
#define LED_ERROR_PIN 27
#define LED_WARNING_PIN 12
#define LED_SUCCESS_PIN 13

// Motor control pins
#define MOTOR1_PIN_A 32
#define MOTOR1_PIN_B 33
#define MOTOR2_PIN_A 26
#define MOTOR2_PIN_B 25
#define MOTOR3_PIN_A 4
#define MOTOR3_PIN_B 18
#define MOTOR4_PIN_A 19
#define MOTOR4_PIN_B 23

// Motor Encoder pins
#define MOTOR1_ENCODER_PIN 36
#define MOTOR2_ENCODER_PIN 39
#define MOTOR3_ENCODER_PIN 34
#define MOTOR4_ENCODER_PIN 35

#include <Arduino.h>

namespace GraspeGPIO {
    // Function to initialize GPIO pins
    inline void initGPIO(){
        pinMode(LED_ERROR_PIN, OUTPUT);
        pinMode(LED_WARNING_PIN, OUTPUT);
        pinMode(LED_SUCCESS_PIN, OUTPUT);
    };

    inline void indicateStatus(bool error, bool warning, bool success){
        digitalWrite(LED_ERROR_PIN, error ? HIGH : LOW);
        digitalWrite(LED_WARNING_PIN, warning ? HIGH : LOW);
        digitalWrite(LED_SUCCESS_PIN, success ? HIGH : LOW);
    };

    inline const char* getVersion() {
        return GRASPE_GPIO_VERSION;
    };
}

namespace Graspe{
    struct ControllerGains
    {
        float Kp;
        float Kd;
        float Ki;
    };

    struct RobotState 
    {
        float jointSetpoint[4];
        float jointPosition[4];
        bool updateController = false;
        ControllerGains controllerGains[4];
    };

    void updateRobotStateSerialLoop(RobotState &localRobotState, RobotState &globalRobotState);
    void updateRobotStateControlLoop(RobotState &localRobotState, RobotState &globalRobotState);
}

#endif // _GRASPE_GPIO_H_