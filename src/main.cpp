#include <Arduino.h>
#include <Graspe.h>
#include <SerialBridge.h>
#include <MotorEncoder.h>

#define HANDSHAKE_TIMEOUT_MS 10000    // 10 seconds timeout for handshake
#define CONTROL_LOOP_DELAY_MS 20      // 50 hz control loop
#define SERIAL_LOOP_DELAY_MS 100      // 10 hz serial communication loop
#define HANDSHAKE_RETRY_DELAY_MS 2000 // 2 seconds delay before retrying handshake

// Multi Treading Task Handles
TaskHandle_t controlLoopTaskHandle;
TaskHandle_t serialBridgeTaskHandle;

SerialBridge serialBridge(Serial, HANDSHAKE_TIMEOUT_MS);
SemaphoreHandle_t stateMutex;
Graspe::RobotState currentRobotState;

void controlLoopTask(void * parameter) {

  // TODO: Add Motor Control Initialization Here
  MotorEncoder m1_encoder(MOTOR1_ENCODER_PIN, 3934.16, 269.249, 13.552, 5.0, 1e-3);
  MotorEncoder m2_encoder(MOTOR2_ENCODER_PIN, 3934.16, 269.249, 13.552, 5.0, 1e-3);
  MotorEncoder m3_encoder(MOTOR3_ENCODER_PIN, 3934.16, 269.249, 13.552, 5.0, 1e-3);
  MotorEncoder m4_encoder(MOTOR4_ENCODER_PIN, 3934.16, 269.249, 13.552, 5.0, 1e-3);

  // TODO: Implement the controler to move to the startposition of the manipulator

  float sp[4], pos[4];

  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t dt = pdMS_TO_TICKS(CONTROL_LOOP_DELAY_MS);
  for(;;) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
      memcpy(sp, currentRobotState.jointSetpoint, sizeof(sp));
    xSemaphoreGive(stateMutex);

    // Read current positions from encoders
    pos[0] = m1_encoder.getFilteredAngle();
    pos[1] = m2_encoder.getFilteredAngle();
    pos[2] = m3_encoder.getFilteredAngle();
    pos[3] = m4_encoder.getFilteredAngle();

    xSemaphoreTake(stateMutex, portMAX_DELAY);
      for (int i = 0; i < 4; i++) {
          currentRobotState.jointPosition[i] = pos[i];
      }
    xSemaphoreGive(stateMutex);

    /*     TODO     */
    /* CONTROL LOOP */
    
    vTaskDelayUntil(&lastWakeTime, dt);
  }
}

void serialBridgeTask(void * parameter) {

  // Initialize Serial and perform handshake
  Serial.begin(115200);
  while (serialBridge.performHandshake() == false)
  {
    GraspeGPIO::indicateStatus(true, false, false);
    Serial.println("<NO_CONNECTION_TRYING_AGAIN>");
    Serial.flush();
    vTaskDelay(pdMS_TO_TICKS(HANDSHAKE_RETRY_DELAY_MS));;
    GraspeGPIO::indicateStatus(false, false, false);
  }
  Serial.println("<SYSTEM_READY>");

  float receivedSetpoint[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // TODO: Add initialization based on current state
  float currentPosition[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  // Begin serial communication loop
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t dt = pdMS_TO_TICKS(SERIAL_LOOP_DELAY_MS);

  for(;;) {
    SerialBridgeCommands::Command command = serialBridge.readCommand();
    if (command.type != SerialBridgeCommands::NO_COMMAND) {
      serialBridge.sendCommandAck(command);
      
      // Parse received command<
      if (command.type == SerialBridgeCommands::SET_JOINT_POSITION) {
        uint8_t idx = command.data.joint.joint_idx;
        if (idx < 4) {
          receivedSetpoint[idx] = command.data.joint.pos;
        }
      } else if (command.type == SerialBridgeCommands::SET_ALL_JOINT_POSITIONS) {
        receivedSetpoint[0] = command.data.manipulator.q1;
        receivedSetpoint[1] = command.data.manipulator.q2;
        receivedSetpoint[2] = command.data.manipulator.q3;
        receivedSetpoint[3] = command.data.manipulator.q4;
      }

      // Update desired setpoints based on received command
      xSemaphoreTake(stateMutex, portMAX_DELAY);
        for (int i = 0; i < 4; i++) {
          currentRobotState.jointSetpoint[i] = receivedSetpoint[i];
        }
      xSemaphoreGive(stateMutex);  
    }
    xSemaphoreTake(stateMutex, portMAX_DELAY);
      serialBridge.sendFeedbackPositions(
          currentRobotState.jointPosition[0],
          currentRobotState.jointPosition[1],
          currentRobotState.jointPosition[2],
          currentRobotState.jointPosition[3]
      );
    xSemaphoreGive(stateMutex);
    vTaskDelayUntil(&lastWakeTime, dt);
  }
}

void setup() {
  GraspeGPIO::initGPIO();

  // Create mutex for command access
  stateMutex = xSemaphoreCreateMutex();
  if (stateMutex == NULL) {
    Serial.println("<CRITICAL_FAILURE>");
    Serial.println("<ERROR_CREATING_MUTEX>");
    for (int i = 0; i < 10; i++)
    {
      GraspeGPIO::indicateStatus(!digitalRead(LED_ERROR_PIN), false, false);
      delay(500);
    }
    Serial.println("<SYSTEM_RESTARTING>");
    Serial.flush();
    ESP.restart();
  }
  
  // Create control loop task pinned to core 0, start immediately
  xTaskCreatePinnedToCore(
        controlLoopTask, "ControlTask", 4096, NULL, 2, &controlLoopTaskHandle, 0
    );
  
  // Create tasks for control loop and serial bridge
  xTaskCreatePinnedToCore(
      serialBridgeTask, "CommTask", 4096, NULL, 1, &serialBridgeTaskHandle, 1
  );
}

void loop() {
  // Empty. Tasks are running independently.
}