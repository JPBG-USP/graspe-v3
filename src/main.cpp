#include <Arduino.h>
#include <GraspeGPIO.h>
#include <SerialBridge.h>

#define HANDSHAKE_TIMEOUT_MS 10000 // 10 seconds timeout for handshake
#define CONTROL_LOOP_DELAY_MS 20   // 50 hz control loop
#define SERIAL_LOOP_DELAY_MS 100   // 10 hz serial communication loop

// Multi Treading Task Handles
TaskHandle_t controlLoopTaskHandle;
TaskHandle_t serialBridgeTaskHandle;

SerialBridge serialBridge(Serial, HANDSHAKE_TIMEOUT_MS); 

void controlLoopTask(void * parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t dt = pdMS_TO_TICKS(CONTROL_LOOP_DELAY_MS);

  for(;;) {
    /*     TODO     */
    /* CONTROL LOOP */
    vTaskDelayUntil(&lastWakeTime, dt);
  }
}

void serialBridgeTask(void * parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t dt = pdMS_TO_TICKS(SERIAL_LOOP_DELAY_MS);

  for(;;) {
    SerialBridgeCommands::Command command = serialBridge.readCommand();
    if (command.type != SerialBridgeCommands::NO_COMMAND) {
      /*          TODO            */
      /* UPDATE COMMAND VARIABLES */
      serialBridge.sendCommandAck(command);
    }
    vTaskDelayUntil(&lastWakeTime, dt);
  }
}

void setup() {
  GraspeGPIO::initGPIO();
  
  // Create control loop task pinned to core 0, start immediately
  xTaskCreatePinnedToCore(
        controlLoopTask, "ControlTask", 4096, NULL, 2, &controlLoopTaskHandle, 0
    );
  
  // Initialize Serial and perform handshake
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

  // Create tasks for control loop and serial bridge
  xTaskCreatePinnedToCore(
      serialBridgeTask, "CommTask", 4096, NULL, 1, &serialBridgeTaskHandle, 1
  );
}

void loop() {
  // Empty. Tasks are running independently.
}