#ifndef _SERIAL_BRIDGE_H_
#define _SERIAL_BRIDGE_H_

#include <Arduino.h>
#include <Graspe.h>

namespace SerialBridgeCommands
{
    /// @brief List of command types supported by the SerialBridge.
    enum CommandType
    {
        NO_COMMAND,
        SET_JOINT_POSITION,
        SET_ALL_JOINT_POSITIONS,
        CHANGE_CONTROLLER_GAINS
    };

    /// @brief The command to set a single Graspe joint position. (in radians)
    struct JointPosCmd
    {
        uint8_t joint_idx;
        float pos;
    };

    /// @brief The command to set all Graspe joint positions at once. (in radians)
    struct GraspeJointPosCmd
    {
        float q1;
        float q2;
        float q3;
        float q4;
    };

    /// @brief The command to change a joint controller gains
    struct ChangeControllerGains
    {
        int joint_idx;
        float Kp;
        float Kd;
        float Ki;
    };
    

    /// @brief The general command structure.
    struct Command
    {
        CommandType type;
        union
        {
            JointPosCmd joint;
            GraspeJointPosCmd manipulator;
            ChangeControllerGains controller;
        } data;
    };
} // namespace SerialBridgeCommands

class SerialBridge
{
private:
    HardwareSerial& serial;
    unsigned long handshakeTimeout;
    String readMessage();
    void sendMessage(const String& message);

public:
    SerialBridge(HardwareSerial& serialPort, unsigned long handshakeTimeoutMs);
    bool performHandshake();
    SerialBridgeCommands::Command readCommand();
    bool sendCommandAck(SerialBridgeCommands::Command cmd);
    bool sendFeedbackPositions(float q1, float q2, float q3, float q4);
    bool sendJointPosition(uint8_t joint_idx, float pos);

};

#endif // _SERIAL_BRIDGE_H_
