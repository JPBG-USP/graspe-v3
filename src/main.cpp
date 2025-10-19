#include <Arduino.h>
#include <PIDcontroller.h>
#include <MotorController.h>
#include <PotEncoder.h>
#include <KalmanFilter1D.h>

// Motor pins
#define PIN_A   27
#define PIN_B   26
#define PIN_POT 33

// PID gains
#define KP 2.5
#define KI 0.5
#define KD 0.2
#define DT 0.005 // 5 ms, 200Hz

// Potenciomenter data
#define POT_BEGIN 3934.1767
#define POT_END 269.249

// Kalman filter data
#define POT_STD 13.552
#define POT_INIT_X 0.0
#define PROCESS_NOISE_VARIANCE 5.0
#define POT_INIT_UNCERTANTY 1.0e-3

// Objects
KalmanFilter1D kalman_filter(POT_STD, PROCESS_NOISE_VARIANCE, POT_INIT_X, POT_INIT_UNCERTANTY);
MotorController motor1(PIN_A, PIN_B);
PIDcontroller pid_controller(KP, KD, KI, DT);
PotEncoder pot_encoder(PIN_POT, POT_BEGIN, POT_END); 

// Global variables
float setpoint = 1.0;
unsigned long loop_time = millis();
// Read Encoder 
int pot_read;
float angle;
// Apply kalman filter and get error
float x;
float error;
// Compute PID action
float u;

void setup() {
  Serial.begin(115200);
  Serial.println("System starting...");  
  Serial.println("Type a new setpoint (0-4095) and press Enter:");
}

void loop() {
  // --- Check if user sent a new setpoint ---
  if (Serial.available() > 0) {
    float val = Serial.parseFloat();   
    if ((val > 0.0) && (val < 3.1415)) {
      setpoint = val;
    }
    while (Serial.available() > 0) Serial.read();
  }

  if ( DT <= (loop_time - millis()))
  {
    // Read Encoder 
    pot_read = pot_encoder.read();
    angle = pot_encoder.getAngle();
    
    // Apply kalman filter and get error
    x = kalman_filter.update(angle);
    error = ((float)setpoint - x);
    
    // Compute PID action
    u = (int)(pid_controller.action(error) * 255);
    // Drive motor
    motor1.action(u);
    loop_time = millis();

    // Debug logging
    Serial.print("SP: ");
    Serial.print(setpoint);
    Serial.print(" | Pot: ");
    Serial.print(pot_read);
    Serial.print(" | Angle: ");
    Serial.print(angle);
    Serial.print(" | Kalman: ");
    Serial.print(x);
    Serial.print(" | Error: ");
    Serial.print(error, 4);
    Serial.print(" | Control: ");
    Serial.println(u);
  }
}