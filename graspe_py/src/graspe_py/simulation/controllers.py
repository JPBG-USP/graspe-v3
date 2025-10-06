import numpy as np

"""
    CONTROLLER SETTINGS
"""
class ControllerPID:

    def __init__(self, gain, dt: float = 0.005):

        # Initialize error terms and coefficients
        self.e0 = np.zeros(4)
        self.e1 = np.zeros(4)
        self.e2 = np.zeros(4)

        self.q0 = np.zeros(4)
        self.q1 = np.zeros(4)
        self.q2 = np.zeros(4)

        self.last_action = np.zeros(4)
        self.dt = dt

        Kp = gain[0:4]
        Kd = gain[4:8]
        Ki = gain[8:12]

        def computeCoefficients(Kp, Kd, Ki):
            q0 = Kp + Ki * self.dt + (Kd / self.dt)
            q1 = -Kp - 2 * (Kd / self.dt)
            q2 = Kd / self.dt
            return q0, q1, q2
        
        for j in range(4):
            self.q0[j], self.q1[j], self.q2[j] = computeCoefficients(Kp[j], Kd[j], Ki[j])

    def act(self, error):
        self.e2 = self.e1.copy()
        self.e1 = self.e0.copy()
        self.e0 = error.copy()

        u = self.last_action + self.q0 * self.e0 + self.q1 * self.e1 + self.q2 * self.e2
        self.last_action = u.copy()
        return u