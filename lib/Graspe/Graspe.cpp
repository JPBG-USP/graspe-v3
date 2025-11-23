#include <Graspe.h>


/**
 * @brief Update the global robot state from the Serial task.
 * 
 * This function is called inside the serial communication loop. It synchronizes
 * the local copy of the robot state used by the serial task with the global robot
 * state shared among all tasks.
 * 
 * - The global setpoints are updated with the new values received from the serial interface.
 * - The local joint positions are refreshed from the global state (measured by the control loop).
 * - If the flag `changeController` is set to true, the controller gains (Kp, Kd, Ki)
 *   are updated in the global state for each joint.
 * 
 * @param localRobotState  Reference to the local RobotState structure used by the serial loop.
 * @param globalRobotState Reference to the global RobotState structure shared between tasks.
 * 
 * @note After applying new controller gains, it is recommended to reset 
 *       `localRobotState.changeController` to false in the calling function.
 */
void Graspe::updateRobotStateSerialLoop(RobotState &localRobotState, RobotState &globalRobotState){

    globalRobotState.updateController = localRobotState.updateController;
    globalRobotState.motorPower = localRobotState.motorPower;
    globalRobotState.gripperOn = localRobotState.gripperOn;
    
    // Update controler gains if is true
    if (globalRobotState.updateController)
    {
        globalRobotState.controllerGains.joint_idx = localRobotState.controllerGains.joint_idx;
        globalRobotState.controllerGains.Kp = localRobotState.controllerGains.Kp;
        globalRobotState.controllerGains.Kd = localRobotState.controllerGains.Kd;
        globalRobotState.controllerGains.Ki = localRobotState.controllerGains.Ki;
    }        

    // For each joint
    for (int i = 0; i < 4; i++)
    {
        globalRobotState.jointSetpoint[i] = localRobotState.jointSetpoint[i];
        localRobotState.jointPosition[i] = globalRobotState.jointPosition[i];
        
    }
};


/**
 * @brief Update the local robot state from the Control task.
 * 
 * This function is called inside the control loop task. It synchronizes
 * the local copy of the robot state used by the control task with the global
 * robot state shared among all tasks.
 * 
 * - The local setpoints are updated from the global state (received via serial).
 * - The global joint positions are updated with the latest encoder readings.
 * - If the flag `changeController` is true, the local controller gains (Kp, Kd, Ki)
 *   are updated for each joint.
 * 
 * @param localRobotState  Reference to the local RobotState structure used by the control loop.
 * @param globalRobotState Reference to the global RobotState structure shared between tasks.
 */
void Graspe::updateRobotStateControlLoop(RobotState &localRobotState, RobotState &globalRobotState){

    localRobotState.updateController = globalRobotState.updateController;
    localRobotState.motorPower = globalRobotState.motorPower;
    localRobotState.gripperOn = globalRobotState.gripperOn;

    if (globalRobotState.updateController)
    {
        localRobotState.controllerGains.joint_idx = globalRobotState.controllerGains.joint_idx;
        localRobotState.controllerGains.Kp = globalRobotState.controllerGains.Kp;
        localRobotState.controllerGains.Kd = globalRobotState.controllerGains.Kd;
        localRobotState.controllerGains.Ki = globalRobotState.controllerGains.Ki;
    }        
    
    // For each joint
    for (int i = 0; i < 4; i++)
    {
        localRobotState.jointSetpoint[i] = globalRobotState.jointSetpoint[i];
        globalRobotState.jointPosition[i] = localRobotState.jointPosition[i];

        // Update controler gains if is true
    }
};
