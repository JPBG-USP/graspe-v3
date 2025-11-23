#include <Arduino.h>
#include <Graspe.h>
#include <SerialBridge.h>
#include <MotorEncoder.h>
#include <MotorController.h>
#include <PIDcontroller.h>
#include <ESP32Servo.h>

#define DEBUG_CODE false
#define HANDSHAKE_TIMEOUT_MS 10000    // 10 seconds timeout for handshake
#define CONTROL_LOOP_DELAY_MS 10      // 100 hz control loop
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

  MotorEncoder m1_encoder(MOTOR1_ENCODER_PIN, 3831, 104, 0.0, PI, 0.005941603939, 2.0e-3, 0.1);
  MotorEncoder m2_encoder(MOTOR2_ENCODER_PIN, 731, 2830, 0, 2.02263, 0.007445738567, 2.0e-3, 0.1);
  MotorEncoder m3_encoder(MOTOR3_ENCODER_PIN, 419, 2902, -PI/2, PI/2, 0.01074133152, 2.0e-3, 0.1);
  MotorEncoder m4_encoder(MOTOR4_ENCODER_PIN, 234, 1586, -PI/2, 0.0, 0.01189513862, 2.0e-3, 0.1);

  PIDcontroller m1_controller(1.8, 0.1, 0.1, CONTROL_LOOP_DELAY_MS/1000.0f);
  PIDcontroller m2_controller(1.7, 0.0, 0.5, CONTROL_LOOP_DELAY_MS/1000.0f);
  PIDcontroller m3_controller(1.4, 0.0, 0.5, CONTROL_LOOP_DELAY_MS/1000.0f);
  PIDcontroller m4_controller(1.4, 0.08, 0.0, CONTROL_LOOP_DELAY_MS/1000.0f);

  Servo gripper;
  gripper.setPeriodHertz(50);
  gripper.attach(GRIPPER_PIN,700,2350);
  gripper.write(GRIPPER_OPEN);
  
  // TODO: Do the startup control to set the manipulator on start position
  
  Graspe::RobotState localRobotState;

  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t dt = pdMS_TO_TICKS(CONTROL_LOOP_DELAY_MS);
  for(;;) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
      Graspe::updateRobotStateControlLoop(localRobotState, currentRobotState);
    xSemaphoreGive(stateMutex);

    if (localRobotState.updateController)
    {
      localRobotState.updateController = false;
      switch (localRobotState.controllerGains.joint_idx)
      {
        case 1: // Motor 1
          m1_controller.updateGains(localRobotState.controllerGains);
          break;
        case 2: // Motor 2
          m2_controller.updateGains(localRobotState.controllerGains);
          break;
        case 3: // Motor 3
          m3_controller.updateGains(localRobotState.controllerGains);
          break;
        case 4: // Motor 4
          m4_controller.updateGains(localRobotState.controllerGains);
          break;
      default:
          break;
      }
    }

    // Read current positions from encoders
    localRobotState.jointPosition[0] = m1_encoder.getFilteredAngle();
    localRobotState.jointPosition[1] = m2_encoder.getFilteredAngle();
    localRobotState.jointPosition[2] = m3_encoder.getFilteredAngle();
    localRobotState.jointPosition[3] = m4_encoder.getFilteredAngle();
    
    float e[4];
    for (int i = 0; i < 4; i++)
    {
      e[i] = localRobotState.jointSetpoint[i] - localRobotState.jointPosition[i];
    }

    float u[4] = {0};

    if(localRobotState.motorPower){
      u[0] = 255 * m1_controller.action(e[0]);
      u[1] = 255 * m2_controller.action(e[1]);
      u[2] = 255 * m3_controller.action(e[2]);
      u[3] = 255 * m4_controller.action(e[3]);
    }

    m1_driver.action(u[0]);
    m2_driver.action(u[1]);
    m3_driver.action(u[2]);
    m4_driver.action(u[3]);

    gripper.write(localRobotState.gripperOn ?  GRIPPER_CLOSED : GRIPPER_OPEN);
    
    #if DEBUG_CODE
    Serial.print("Motor Action: ");
    Serial.print(u[1]);
    Serial.print(" Motor pose: ");
    Serial.print(pos[1]);
    Serial.print(" Set point: ");
    Serial.println(sp[1]);
    #endif

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

  Graspe::RobotState localRobotState;

  // Joint Start position
  localRobotState.jointSetpoint[0] = PI/2;
  localRobotState.jointSetpoint[1] = PI/4;
  localRobotState.jointSetpoint[2] = PI/2;
  localRobotState.jointSetpoint[3] = -PI/4;

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
          localRobotState.jointSetpoint[idx] = command.data.joint.pos;
        }
      }
      if (command.type == SerialBridgeCommands::SET_ALL_JOINT_POSITIONS) {
        localRobotState.jointSetpoint[0] = command.data.manipulator.q1;
        localRobotState.jointSetpoint[1] = command.data.manipulator.q2;
        localRobotState.jointSetpoint[2] = command.data.manipulator.q3;
        localRobotState.jointSetpoint[3] = command.data.manipulator.q4;
      }
      if (command.type == SerialBridgeCommands::CHANGE_CONTROLLER_GAINS)
      {
        localRobotState.updateController = true;
        localRobotState.controllerGains.joint_idx = command.data.controller.joint_idx;
        localRobotState.controllerGains.Kp = command.data.controller.Kp;
        localRobotState.controllerGains.Kd = command.data.controller.Kd;
        localRobotState.controllerGains.Ki = command.data.controller.Ki;
      }
      if (command.type == SerialBridgeCommands::CHANGE_MOTOR_POWER_STATE){
        localRobotState.motorPower = command.data.motors_power.state;
      }
      if (command.type == SerialBridgeCommands::CHANGE_GRIPPER_STATE){
        localRobotState.gripperOn = command.data.gripper.state;
      }
      
    }
    xSemaphoreTake(stateMutex, portMAX_DELAY);
      Graspe::updateRobotStateSerialLoop(localRobotState, currentRobotState);
    xSemaphoreGive(stateMutex);

    localRobotState.updateController = false;

    serialBridge.sendFeedbackPositions(
        localRobotState.jointPosition[0],
        localRobotState.jointPosition[1],
        localRobotState.jointPosition[2],
        localRobotState.jointPosition[3]
    );
    
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