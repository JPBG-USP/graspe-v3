#ifndef _SERIAL_BRIDGE_H_
#define _SERIAL_BRIDGE_H_

#include <Arduino.h>
#include <GraspeGPIO.h>

class SerialBridge {
    private:
        HardwareSerial& serial;
        unsigned long handshakeTimeout;
        String readMessage();

    public:
        SerialBridge(HardwareSerial& serialPort, unsigned long handshakeTimeoutMs);
        bool performHandshake();
        SerialBridgeCommands::Command readCommand();

};

namespace SerialBridgeCommands
{
    /**
     * List of command types supported by the SerialBridge.
     */
    enum CommandType
    {
        SET_JOINT_POSITION,
        SET_ALL_JOINT_POSITIONS
    };


    /**
     * The command to set a single Graspe joint position. (in radians)
     */
    typedef struct
    {
        u_int8_t joint_idx;
        float pos;
    } JointPosCmd;


    /**
     * The command to set all Graspe joint positions at once. (in radians)
     */
    typedef struct
    {
        float q1;
        float q2;
        float q3;
        float q4;
    } GraspeJointPosCmd;

    /**
     * The general command structure.
     */
     typedef struct Command {
        CommandType type;   // Indica se é junta ou manipulador
        union {
            JointPosCmd joint;
            GraspeJointPosCmd manipulator;
        } data;
    };


}

#endif // _SERIAL_BRIDGE_H_