#include "SerialBridge.h"
#include <GraspeGPIO.h>


SerialBridge::SerialBridge(HardwareSerial& serialPort, unsigned long handshakeTimeoutMs)
    : serial(serialPort), handshakeTimeout(handshakeTimeoutMs) {}

/**
 * @brief Performs a handshake over the serial port with the connected device.
 * 
 * This function initiates a handshake procedure by sending the "<ESP_READY>" message
 * over the configured serial port. It waits for a response message "<HELLO>" from the
 * device within the specified timeout (`handshakeTimeout`). 
 * 
 * While waiting, the warning LED (`LED_WARNING_PIN`) blinks to indicate the handshake
 * is in progress. If a successful handshake is received:
 * - The function sends back "<HELLO_ACK>".
 * - Turns off the warning and error LEDs.
 * - Blinks the success LED (`LED_SUCCESS_PIN`) in a short pattern to indicate success.
 * 
 * If no valid response is received within the timeout, the function sends
 * "<HANDSHAKE_FAILED>" and returns false.
 * 
 * @return true if handshake is successful.
 * @return false if handshake times out or fails.
 */
bool SerialBridge::performHandshake() {

    unsigned long start = millis();
    unsigned long blink_time = start;

    while (millis() - start < handshakeTimeout) {
        String msg = readMessage();
        if (msg == "HELLO") {
            // Acknowledge handshake
            serial.println("<HELLO_ACK>");

            // update LED states
            digitalWrite(LED_WARNING_PIN, LOW);
            digitalWrite(LED_ERROR_PIN, LOW);
            
            // Blink success LED to indicate successful handshake
            bool success_state = true; 
            for (int i = 0; i < 9; i++)
            {
                success_state = !success_state;
                digitalWrite(LED_SUCCESS_PIN, success_state);
                delay(250/(0.5*i+1));
            }
            digitalWrite(LED_SUCCESS_PIN, HIGH);
            serial.println("<HANDSHAKE_SUCCESS>");
            return true;
        }
        // blink warning LED while waiting for handshake
        if (millis() - blink_time > 250)
        {
            blink_time = millis();
            static bool warning_state = false;
            warning_state = !warning_state;
            digitalWrite(LED_WARNING_PIN, warning_state);
        }
    }
    // Handshake failed
    serial.println("<HANDSHAKE_FAILED>");
    return false;
}

/**
 * @brief Reads a complete message from the serial port.
 * 
 * This function reads incoming characters from the serial buffer and constructs a message
 * delimited by '<' and '>'. Only characters between these delimiters are returned.
 * 
 * Example:
 * - Serial input: "<HELLO>"
 * - Returned string: "HELLO"
 * 
 * If no complete message is available, an empty string is returned.
 * 
 * @return String The message content between the '<' and '>' delimiters, excluding the delimiters themselves.
 */
String SerialBridge::readMessage() {

    String msg = "";
    bool receiving = false;

    while (serial.available())
    {
        char c = serial.read();
        if (c == '<'){
            receiving = true;
        } else if (c == '>'){
            receiving = false;
            return msg;
        } 
        else if (receiving)
        {
            msg += c;
        }
    }
    return "";
}

/**
 * @brief Reads a command from the serial interface and parses it into a structured format.
 * 
 * This function reads a complete message from the serial buffer (using `readMessage()`) and
 * interprets it as a command for the manipulator. It supports both single joint commands
 * (SETQ) and full manipulator commands (SETALLQ).
 * 
 * Supported command formats:
 * - Single joint command: "SETQ<joint_index> <position>"
 *   - Example: "SETQ1 45.0" sets joint 1 to position 45.0
 * - Full manipulator command: "SETALLQ <q1> <q2> <q3> <q4>"
 *   - Example: "SETALLQ 10.0 20.0 30.0 40.0" sets all four joints
 * 
 * @return SerialBridgeCommands::Command A struct representing the parsed command,
 *         including the command type and associated joint or manipulator positions.
 */
SerialBridgeCommands::Command SerialBridge::readCommand() {
    SerialBridgeCommands::Command cmd;
    String msg = readMessage();

    // If command is to set a single joint position
    if(msg.startsWith("SETQ")){
        cmd.type = SerialBridgeCommands::SET_JOINT_POSITION;

        // Extract joint index
        cmd.data.joint.joint_idx = msg.substring(4,5).toInt();

        // Extract position value
        int spaceIndex = msg.indexOf(' ');
        cmd.data.joint.pos = msg.substring(spaceIndex + 1).toFloat();
        return cmd;
    }

    if(msg.startsWith("SETALLQ")){
        cmd.type = SerialBridgeCommands::SET_ALL_JOINT_POSITIONS;

        // Extract all joint positions
        int firstSpace = msg.indexOf(' ');
        int secondSpace = msg.indexOf(' ', firstSpace + 1);
        int thirdSpace = msg.indexOf(' ', secondSpace + 1);
        int fourthSpace = msg.indexOf(' ', thirdSpace + 1);

        cmd.data.manipulator.q1 = msg.substring(firstSpace + 1, secondSpace).toFloat();
        cmd.data.manipulator.q2 = msg.substring(secondSpace + 1, thirdSpace).toFloat();
        cmd.data.manipulator.q3 = msg.substring(thirdSpace + 1, fourthSpace).toFloat();
        cmd.data.manipulator.q4 = msg.substring(fourthSpace + 1).toFloat();
        return cmd;
    }
}