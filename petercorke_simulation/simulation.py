from typing import Callable
import roboticstoolbox as rtb
from .robot_description import GRASPE_ROBOT
from matplotlib import pyplot as plt
import numpy as np
from tqdm import tqdm

robot = GRASPE_ROBOT

def run_simulation(
    time_s: float,
    dt: float,
    gain: np.ndarray,
    desired_q: Callable
):
    
    # control variables
    Kp = gain[0:4]
    Kd = gain[4:8]
    Ki = gain[8:12]

    def computeCoefficients(Kp, Kd, Ki):
        q0 = Kp + Ki * dt + (Kd / dt)
        q1 = -Kp - 2 * (Kd / dt)
        q2 = Kd / dt
        return q0, q1, q2
    
    q0, q1, q2 = np.zeros(4), np.zeros(4), np.zeros(4)
    for j in range(4):
        q0[j], q1[j], q2[j] = computeCoefficients(Kp[j], Kd[j], Ki[j])

    e0: np.ndarray = np.array([0.0, 0.0, 0.0, 0.0])
    e1: np.ndarray = np.array([0.0, 0.0, 0.0, 0.0])
    e2: np.ndarray = np.array([0.0, 0.0, 0.0, 0.0])

    last_action = np.array([0.0, 0.0, 0.0, 0.0])

    # robot description
    robot = GRASPE_ROBOT
    q = desired_q(0.0)
    qd = np.array([0.0, 0.0, 0.0, 0.0])
    tau_max = 1.5 # Nm

    # simulation setting
    num_steps = int(time_s/dt)
    gravity = np.array([0.0, 0.0, -9.81]) # m/s^2

    # recorder 
    log = {
        "timestamp": np.zeros(num_steps),
        "q": np.zeros((num_steps, 4)),
        "qd": np.zeros((num_steps, 4)),
        "qdd": np.zeros((num_steps, 4)),
        "action": np.zeros((num_steps, 4)),
        "tau": np.zeros((num_steps, 4)),
        "error": np.ones((num_steps, 4)) * 100, 
    }

    # running simulation
    for i in tqdm(range(num_steps)):
        t = i * dt

        # control law
        e2 = e1.copy()
        e1 = e0.copy()
        e0 = desired_q(t) - q.copy()

        action = last_action + q0 * e0 + q1 * e1 + q2 * e2
        last_action = action
        
        # scalling
        tau = action * 1e-3
        tau += robot.gravload(desired_q(t))

        # not limiting for now
        # tau = np.clip(tau, -tau_max, tau_max)

        # dynamic update
        qdd = robot.accel(q, qd, tau, gravity)
        if np.any(np.isnan(qdd)) or np.any(np.isinf(qdd)):
            print(f"Integration stopped at t = {t:.3f}s due to invalid qdd")
            return log

        # step simulation
        qd = qd + qdd * dt
        q = q + qd * dt

        # To lock joints just uncomment
        # q[0] = desired_q(t)[0]
        # q[1] = desired_q(t)[1]
        # q[2] = desired_q(t)[2]
        # q[3] = desired_q(t)[3]

        # record
        log["timestamp"][i] = t
        log["q"][i,:] = q
        log["qd"][i,:] = qd
        log["qdd"][i,:] = qdd
        log["action"][i,:] = action
        log["tau"][i,:] = tau
        log["error"][i, :] = e0

    return log


def desired_trajectory(t):
    return np.array([
        0.5*np.sin(0.5*t),
        0.5*np.sin(0.4*t),
        -1 + 0.5*np.sin(0.2*t),
        0.5*np.sin(0.3*t)
    ])

    

"""
    Test
"""
if __name__ == "__main__":

    gains = np.array([
        1800.0, 7000.0, 8000.0, 20000.0,
        50.0, 100.0, 50.0, 0.0,
        0.0, 0.0, 0.0, 9000.0
    ])

    log = run_simulation(15.0, 0.005, gains, desired_trajectory)

    # robot.plot(log["q"], dt=0.005)

    q_ref = np.array([desired_trajectory(t) for t in log["timestamp"]])

    for j in range(4):
        plt.figure()
        plt.plot(log["timestamp"], log["q"][:, j], label=f"q{j} simulado")
        plt.plot(log["timestamp"], q_ref[:, j], '--', label=f"q{j} desejado")
        plt.xlabel("Tempo (s)")
        plt.ylabel(f"Ângulo da junta {j} (rad)")
        plt.legend()
        plt.title(f"Comparação Junta {j}")
        plt.show()
