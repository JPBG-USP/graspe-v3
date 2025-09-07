from matplotlib import pyplot as plt
import roboticstoolbox as rtb
from typing import Tuple
import numpy as np

# Time discretization
DT = 0.005 # s

# PID gains of each joint
KP = np.array([4.0, 100.0, 4.0, 4.0])
KD = np.array([1.0, 1.0, 1.0, 1.0])
KI = np.array([0.1, 20.0, 0.1, 0.1])

def main(time_s: float, target_q: np.ndarray):
    num_steps = int(time_s / DT)

    # robot declaration
    mass = 1.0
    r = np.array([0, 0, 0])       # centro de massa no próprio frame do link
    I = np.eye(3)                  # momento de inércia unitário 3x3

    robot = rtb.DHRobot([
        rtb.RevoluteDH(d=10, alpha=np.pi/2, m=mass, r=r, I=I),
        rtb.RevoluteDH(a=10, m=mass, r=r, I=I),
        rtb.RevoluteDH(a=10, m=mass, r=r, I=I),
        rtb.RevoluteDH(a=10, m=mass, r=r, I=I)
    ], name='Graspe')

    # system states
    q  = np.zeros(4)   
    qd = np.zeros(4)   

    # erro 
    e0 = np.zeros(4)
    e1 = np.zeros(4)
    e2 = np.zeros(4)

    last_action = np.zeros(4)

    # Discrete PID coefficients
    def computeCoefficients(Kp, Kd, Ki):
        q0 = Kp + Ki*DT + (Kd/DT)
        q1 = -Kp - 2*(Kd/DT)
        q2 = Kd/DT
        return q0, q1, q2
    
    q0, q1, q2 = np.zeros(4), np.zeros(4), np.zeros(4)
    for j in range(4):
        q0[j], q1[j], q2[j] = computeCoefficients(KP[j], KD[j], KI[j])

    # logs
    q_log = []
    t_log = []

    # running simulation
    for i in range(num_steps):
        t = i*DT
        desired_q = target_q[i, :]

        # update error
        e2 = e1.copy()
        e1 = e0.copy()
        e0 = desired_q - q   

        # PID discreto (forma recursiva)
        action = last_action + q0*e0 + q1*e1 + q2*e2
        last_action = action

        # dinâmica do robô: qdd = accel(q, qd, tau)
        qdd = robot.accel(q, qd, action*100)

        # integração (Euler)
        qd = qd + qdd*DT
        q  = q + qd*DT

        q_log.append(q.copy())
        t_log.append(t)
    
    q_log_array = np.array(q_log)
    t_log_array = np.array(t_log)


     # ---- Subplots das juntas ----
    n_joints = q_log_array.shape[1]
    fig, axes = plt.subplots(n_joints, 1, figsize=(8, 2*n_joints), sharex=True)

    for j in range(n_joints):
        axes[j].plot(t_log_array, q_log_array[:, j], label=f'Joint {j+1} actual')
        axes[j].plot(t_log_array, target_q[:, j], '--', label=f'Joint {j+1} desired')
        axes[j].set_ylabel('Angle [rad]')
        axes[j].legend()
        axes[j].grid(True)

    axes[-1].set_xlabel('Time [s]')
    plt.tight_layout()
    plt.show()

    # robot.plot(np.array([0.1, 0.2, 0.3, 0.4]), block=True)
    # robot.plot(q_log_array, dt=DT)


if __name__ == "__main__":
    time_s = 20.0
    steps = int(time_s / DT)
    target_q = np.tile(np.array([0.1, 0.2, 0.3, 0.4]), (steps, 1))
    main(time_s, target_q)