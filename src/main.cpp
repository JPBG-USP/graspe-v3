#include <Arduino.h>
#include <Graspe.h>
#include <SerialBridge.h>
#include <MotorEncoder.h>
#include <MotorController.h>
#include <PIDcontroller.h>

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

  MotorController m1_driver(MOTOR1_PIN_A, MOTOR1_PIN_B);
  MotorController m2_driver(MOTOR2_PIN_A, MOTOR2_PIN_B);
  MotorController m3_driver(MOTOR3_PIN_A, MOTOR3_PIN_B);
  MotorController m4_driver(MOTOR4_PIN_A, MOTOR4_PIN_B);

  MotorEncoder m1_encoder(MOTOR1_ENCODER_PIN, 3831, 104, 0.0, PI, 0.008743444454, 5.0e-3, 0.1);
  MotorEncoder m2_encoder(MOTOR2_ENCODER_PIN, 731, 2830, 0, 2.02263, 0.01083128044, 5.0e-3, 0.1);
  MotorEncoder m3_encoder(MOTOR3_ENCODER_PIN, 434, 2885, PI/2, -PI/2, 0, 5.0e-3, 0.1);      // TODO: Find better Kalman Parameter
  MotorEncoder m4_encoder(MOTOR4_ENCODER_PIN, 378, 4049, -PI/2, PI/2, 0.1677506961, 5.0e-3, 0.1); // TODO: Find better Kalman Parameter

  PIDcontroller m1_controller(0.9, 0.1, 0.0, CONTROL_LOOP_DELAY_MS/1000.0f);
  PIDcontroller m2_controller(0.0, 0.0, 0.0, CONTROL_LOOP_DELAY_MS/1000.0f);
  PIDcontroller m3_controller(0.0, 0.0, 0.0, CONTROL_LOOP_DELAY_MS/1000.0f);
  PIDcontroller m4_controller(0.0, 0.0, 0.0, CONTROL_LOOP_DELAY_MS/1000.0f);

  float sp[4], pos[4], u[4];

  // TODO: Do the startup control to set the manipulator on start position
  sp[0] = PI/2;

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

    u[0] = 255 * m1_controller.action(sp[0] - pos[0]);
    // u[1] = 255 * m2_controller.action(sp[1] - pos[1]);
    // u[2] = 255 * m3_controller.action(sp[2] - pos[2]);
    // u[3] = 255 * m4_controller.action(sp[3] - pos[3]);

    m1_driver.action(u[0]);
    // m2_driver.action(u[1]);
    // m3_driver.action(u[2]);
    // m4_driver.action(u[3]);

    // DEBUG CODE
    // Serial.print("Motor Action: ");
    // Serial.print(u[0]);
    // Serial.print(" Motor pose: ");
    // Serial.print(pos[0]);
    // Serial.print(" Set point: ");
    // Serial.println(sp[0]);
    // Serial.print(" LASTSet point: ");
    // Serial.println(last_sp[0]);
    
    vTaskDelayUntil(&lastWakeTime, dt);
  }
}

void serialBridgeTask(void * parameter) {

  // Initialize Serial and perform handshake
  Serial.begin(115200);
  while (serialBridge.performHandshake() == false)
  {
    GraspeGPIO::indicateStatus(true, false, false);
    // Serial.println(ADRIANO_ASCII);
    Serial.println("<NO_CONNECTION_TRYING_AGAIN>");
    Serial.flush();
    vTaskDelay(pdMS_TO_TICKS(HANDSHAKE_RETRY_DELAY_MS));;
    GraspeGPIO::indicateStatus(false, false, false);
  }
  Serial.println("<SYSTEM_READY>");

  float receivedSetpoint[4] = {PI/2, 0.0f, 0.0f, 0.0f}; // TODO: Add initialization based on current state

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