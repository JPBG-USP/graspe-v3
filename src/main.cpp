#include <Arduino.h>
#include <GraspeGPIO.h>
#include <SerialBridge.h>

#define HANDSHAKE_TIMEOUT_MS 10000 // 10 seconds timeout for handshake

SerialBridge serialBridge(Serial, HANDSHAKE_TIMEOUT_MS); 

void setup() {
  GraspeGPIO::initGPIO();
  Serial.begin(115200);
  while (serialBridge.performHandshake() == false)
  {
    digitalWrite(LED_ERROR_PIN, HIGH);
    digitalWrite(LED_WARNING_PIN, LOW);
    digitalWrite(LED_SUCCESS_PIN, LOW);
    Serial.println("<NO_CONNECTION_TRYING_AGAIN>");
    delay(2000);
    digitalWrite(LED_ERROR_PIN, LOW);
  }
  Serial.println("<SYSTEM_READY>");
}

void loop() {
  SerialBridgeCommands::Command command = serialBridge.readCommand();
  if (command.type == SerialBridgeCommands::SET_JOINT_POSITION)
  {
    Serial.println("Received SET_JOINT_POSITION command");
  }
  else if (command.type == SerialBridgeCommands::SET_ALL_JOINT_POSITIONS)
  {
    Serial.println("Received SET_ALL_JOINT_POSITIONS command");
  }
  serialBridge.sendCommandAck(command);
  delay(1000);
}