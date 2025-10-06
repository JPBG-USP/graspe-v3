import numpy as np
from scipy.integrate import RK45

"""
    Simpled DC Motor model configuration
"""
class DCMotorCfg:
    """
        DC Motor simulation config
    """

    Ra: float = 1.0    
    """ Armature resistance (ohms)"""

    La: float = 0.5    
    """ Armature inductance (henrys) """
    
    Ke: float = 0.01 
    """ Back EMF constant (V/(rad/s)) """

    Kt: float = 0.01   
    """ Torque constant (Nm/A) """

    J: float = 0.01   
    """ Rotor inertia (kg*m^2) """

    B: float = 0.1     
    """ Viscous friction coefficient (Nm/(rad/s)) """
    
    V_max: float = 6.0
    """ Maximum voltage (V) """

    fast_simulation: bool = True



class DCMotor:
    """
        DC Motor model
    """
    
    def __init__(self, cfg: DCMotorCfg = DCMotorCfg()):        
        # Loading config
        self._cfg = cfg
        # state: [theta, omega, ia, torque]
        self._state: np.ndarray = np.zeros(4)

    def reset(self, state: np.ndarray=None):
        """ Reset the motor state """
        if state is None:
            self._state = np.zeros(4)
        else:
            self._state = np.array(state, dtype=float)


    def step(self, V: float, dt:float, Tl:float = 0):

        theta, omega, ia, torque = self._state[0], self._state[1], self._state[2], self._state[3]

        V = np.clip(V, -self._cfg.V_max, self._cfg.V_max)

        torque_new = self._cfg.Kt * ia

        # derivatives
        def derivatives(t, y):
            theta, omega, ia = y
            dtheta = omega
            domega = (self._cfg.Kt * ia - self._cfg.B * omega - Tl) / self._cfg.J
            dia = (V - self._cfg.Ra * ia - self._cfg.Ke * omega) / self._cfg.La
            return [dtheta, domega, dia]
        
        # updating state
        if self._cfg.fast_simulation:
            # Euler integration
            dtheta, domega, dia = derivatives(0, [theta, omega, ia])
            theta_new = theta + dtheta * dt
            omega_new = omega + domega * dt
            ia_new = ia + dia * dt
        else:
            # RK45 integration  
            def rk4_step(f, y, t, dt):
                k1 = np.array(f(t, y))
                k2 = np.array(f(t + dt/2, y + dt/2 * k1))
                k3 = np.array(f(t + dt/2, y + dt/2 * k2))
                k4 = np.array(f(t + dt, y + dt * k3))
                return y + dt/6 * (k1 + 2*k2 + 2*k3 + k4)

            y_new = rk4_step(derivatives, np.array([theta, omega, ia]), 0, dt)
            theta_new, omega_new, ia_new = y_new

        # saving state
        self._state[0], self._state[1], self._state[2], self._state[3] = theta_new, omega_new, ia_new, torque_new

        return self._state
    

    @property
    def state(self):
        return self._state
    
    @property
    def cfg(self):
        return self._cfg
    
    def get_position(self):
        return self._state[0]

    def get_speed(self):
        return self._state[1]

    def get_current(self):
        return self._state[2]

    def get_torque(self):
        return self._state[3]


if __name__ == "__main__":
    motor = DCMotor()
    dt = 0.001
    T = 2.0
    time = np.arange(0, T, dt)

    voltages = []
    states = []
    for t in time:
        V = 6.0 if t < 1.0 else 0.0 
        voltages.append(V)
        state = motor.step(V, dt)
        states.append(state.copy())

    states = np.array(states)

    import matplotlib.pyplot as plt
    plt.plot(time, states[:,1], label="Angular Velocity (rad/s)")
    plt.plot(time, states[:,2], label="Current (A)")
    plt.plot(time, states[:,3], label="Torque (Nm)")
    plt.plot(time, voltages, label="Voltage (V)")
    plt.grid()
    plt.xlabel("Tempo (s)")
    plt.legend()
    plt.show()
