#ifndef _SERIAL_BRIDGE_H_
#define _SERIAL_BRIDGE_H_

#include <Arduino.h>
#include <GraspeGPIO.h>

namespace SerialBridgeCommands
{
    /// @brief List of command types supported by the SerialBridge.
    enum CommandType
    {
        NO_COMMAND,
        SET_JOINT_POSITION,
        SET_ALL_JOINT_POSITIONS
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

    /// @brief The general command structure.
    struct Command
    {
        CommandType type;
        union
        {
            JointPosCmd joint;
            GraspeJointPosCmd manipulator;
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

};

#endif // _SERIAL_BRIDGE_H_
