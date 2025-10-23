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
#define KI 2.0
#define KD 0.12
#define DT 0.005 // 5 ms, 200Hz

// Potentiometer data
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
float setpoint = 1.0f;
unsigned long loop_time = millis();
// Read Encoder 
int pot_read;
float angle;
// Apply Kalman filter and get error
float x;
float error;
// Compute PID action
float u;
// Buffer to store messages
String buffer = "";
bool receiving = false;

// timeout for handshake in milliseconds
const unsigned long HANDSHAKE_TIMEOUT_MS = 30000;

bool handshake_ok = false;

// Function to read complete messages from PC
String readSerialMessage() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '<') {
      buffer = "";
      receiving = true;
    } else if (c == '>') {
      receiving = false;
      String msg = buffer;
      buffer = "";
      return msg;
    } else if (receiving) {
      buffer += c;
    }
  }
  return "";
}

void setup() {
  Serial.begin(115200);

  Serial.println("<ESP_READY>");

  unsigned long start = millis();

  // Try handshake within a limited time
  while (millis() - start < HANDSHAKE_TIMEOUT_MS) {
    String msg = readSerialMessage();
    if (msg == "HELLO") {
      Serial.println("<HELLO_ACK>");
      handshake_ok = true;
      break;
    }
  }

  if (!handshake_ok) {
    // If handshake fails, restart after 5 seconds
    Serial.println("<HANDSHAKE_FAILED>");
    while (true) {
      Serial.println("<NO_CONNECTION_RESTARTING>");
      delay(5000);
      ESP.restart();  // Restart the microcontroller
    }
  }

  Serial.println("<HANDSHAKE_SUCCESS>");
  Serial.println("<SYSTEM_READY>");
}

void loop() {
  String msg = readSerialMessage();

  if (msg.startsWith("SETQ1")) {
    int spaceIndex = msg.indexOf(' ');
    if (spaceIndex > 0) {
      String valueStr = msg.substring(spaceIndex + 1);
      float newSetpoint = valueStr.toFloat();
      setpoint = newSetpoint;
      Serial.print("<ACK SETQ1 ");
      Serial.print(setpoint, 3);
      Serial.println(">");
    }
  }

  // Simulate reading the motor's position
  if (millis() - loop_time >= DT) {
    pot_read = pot_encoder.read();
    angle = pot_encoder.getAngle();
    
    // Apply Kalman filter and get error
    x = kalman_filter.update(angle);
    error = setpoint - x;
    
    // Compute PID action
    u = (int)(pid_controller.action(error) * 255);
    // Drive motor
    motor1.action(u);
    loop_time = millis();

    // Send motor position feedback
    Serial.print("<posq1 ");
    Serial.print(x * 180.0/3.1415, 3);  // Send the position with 3 decimal places
    Serial.println(">");
  }
}