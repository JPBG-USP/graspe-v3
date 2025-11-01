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

    serial.println("<ESP_READY>");

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
String SerialBridge::readMessage(){

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
SerialBridgeCommands::Command SerialBridge::readCommand(){
    SerialBridgeCommands::Command cmd;
    String msg = readMessage();

    // If command is to set a single joint position
    if(msg.startsWith("SETQ")){
        cmd.type = SerialBridgeCommands::SET_JOINT_POSITION;

        // Extract joint index
        String jointStr = msg.substring(4,5);
        if (jointStr.length() == 0 || jointStr.toInt() < 0) {
            cmd.type = SerialBridgeCommands::NO_COMMAND;
            return cmd;
        }
        cmd.data.joint.joint_idx = jointStr.toInt();

        // Extract position value
        int spaceIndex = msg.indexOf(' ');
        String posStr = msg.substring(spaceIndex + 1);
        if (posStr.length() == 0 || (posStr.length() == msg.length())) {
            cmd.type = SerialBridgeCommands::NO_COMMAND;
            return cmd;
        }
        cmd.data.joint.pos = posStr.toFloat();

        return cmd;
    }

    // If command is to set all joint positions
    if (msg.startsWith("SETALLQ")) {
        cmd.type = SerialBridgeCommands::SET_ALL_JOINT_POSITIONS;

        float values[4] = {0};
        int start = msg.indexOf(' ') + 1;
        bool valid = true;

        for (int i = 0; i < 4; i++) {
            if (start <= 0) {
                valid = false;
                break;
            }

            int end = msg.indexOf(' ', start);
            String part;
            if (end == -1) {
                part = msg.substring(start);
                start = -1;
            } else {
                part = msg.substring(start, end);
                start = end + 1;
            }

            if (part.length() == 0) {
                valid = false;
                break;
            }

            values[i] = part.toFloat();
        }

        if (!valid || start != -1) {
            cmd.type = SerialBridgeCommands::NO_COMMAND;
            return cmd;
        }

        cmd.data.manipulator.q1 = values[0];
        cmd.data.manipulator.q2 = values[1];
        cmd.data.manipulator.q3 = values[2];
        cmd.data.manipulator.q4 = values[3];

        return cmd;
    }
    cmd.type = SerialBridgeCommands::NO_COMMAND;
    return cmd;
}

/**
 * @brief Sends an acknowledgment message for a received command over the serial interface.
 * 
 * This function constructs and sends an acknowledgment message based on the type of command
 * received. It supports acknowledgments for both single joint commands (SETQ) and full manipulator
 * commands (SETALLQ).
 * 
 * Acknowledgment message formats:
 * - For single joint command: "<ACK_SETQ<joint_index> <position>>"
 *   - Example: "<ACK_SETQ1 45.0>"
 * - For full manipulator command: "<ACK_SETALLQ <q1> <q2> <q3> <q4>>"
 *   - Example: "<ACK_SETALLQ 10.0 20.0 30.0 40.0>"
 * 
 * @param cmd The command structure containing the command type and associated data.
 * @return true if the acknowledgment message was sent successfully.
 * @return false if the command type is unrecognized.
 */
bool SerialBridge::sendCommandAck(SerialBridgeCommands::Command cmd){
    String ack_msg = "ACK_";
    switch (cmd.type)
    {
    case SerialBridgeCommands::SET_JOINT_POSITION:
        ack_msg += "SETQ" + String(cmd.data.joint.joint_idx) + " " + String(cmd.data.joint.pos);
        sendMessage(ack_msg);
        return true;
    case SerialBridgeCommands::SET_ALL_JOINT_POSITIONS:
        ack_msg += "SETALLQ " + String(cmd.data.manipulator.q1) + " " + String(cmd.data.manipulator.q2)
                    + " " + String(cmd.data.manipulator.q3) + " " + String(cmd.data.manipulator.q4);
        sendMessage(ack_msg);
        return true;
    default:
        return false;;
    }
}

/**
 * @brief Sends a formatted message over the serial interface.
 * 
 * This function takes a message string, formats it by enclosing it within
 * angle brackets ('<' and '>'), and sends it over the configured serial port.
 * 
 * Example:
 * - Input message: "HELLO"
 * - Sent message: "<HELLO>"
 */
void SerialBridge::sendMessage(const String& msg){
    String formated_msg = "<" + msg + ">";
    serial.println(formated_msg);
}

/**
 * @brief Sends the current positions of all manipulator joints over the serial interface.
 * 
 * This function constructs a message containing the positions of all four joints
 * and sends it in the format: "<POSALL q1 q2 q3 q4>".
 * 
 * @param q1 Position of joint 1.
 * @param q2 Position of joint 2.
 * @param q3 Position of joint 3.
 * @param q4 Position of joint 4.
 * @return true if the message was sent successfully.
 */
bool SerialBridge::sendFeedbackPositions(float q1, float q2, float q3, float q4){
    String feedback_msg = "POSALL " + String(q1) + " " + String(q2) + " " + String(q3) + " " + String(q4);
    sendMessage(feedback_msg);
    return true;
}

/**
 * @brief Sends the position of a single joint over the serial interface.
 * 
 * This function constructs a message containing the position of a specified joint
 * and sends it in the format: "<POSQ<joint_index> position>".
 * 
 * @param joint_idx Index of the joint.
 * @param pos Position of the joint.
 * @return true if the message was sent successfully.
 */
bool SerialBridge::sendJointPosition(uint8_t joint_idx, float pos){
    String feedback_msg = "POSQ" + String(joint_idx) + " " + String(pos);
    sendMessage(feedback_msg);
    return true;
}