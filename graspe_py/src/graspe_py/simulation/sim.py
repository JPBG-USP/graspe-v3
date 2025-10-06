from typing import Callable, Dict
import roboticstoolbox as rtb
from .robot_description import GRASPE_ROBOT
from matplotlib import pyplot as plt
import numpy as np
from tqdm import tqdm
from roboticstoolbox.backends.swift import Swift
from scipy.integrate import RK45
from .controllers import ControllerPID

"""
    SIMULATION SETTINGS
"""
class SimulationCfg:
    """
        Simulation configuration class
    """

    dt: float = 0.005
    """ Time step for the physics simulation"""

    decimation: int = 1
    """ Number of simulation steps per control step"""

    robot: rtb.ERobot = GRASPE_ROBOT
    """ Robot model to be simulated"""

    gravity:np.ndarray = np.array([0.0, 0.0, -9.81])
    """ Gravity vector"""

    controller: ControllerPID = None
    """ Controller to be used """

    gravload_compensator: bool = True
    """ Enable/disable gravity load compensation """

    initial_q: np.ndarray =  None
    """ Initial joint positions (if None, zeros are used) """

    fast_simulation: bool = True
    """ Use fast simulation (explicit Euler) or accurate simulation (RK4) """


"""
    SIMULATION RUNNER
"""
class Simulation:

    _robot_state: Dict[str, np.ndarray] = {
        "q": np.zeros(4),
        "qd": np.zeros(4),
        "qdd": np.zeros(4),
    }

    _action: np.ndarray = np.zeros(4)

    _step: float = 0

    _decimation: int = 1

    _robot: rtb.ERobot = None

    _command: np.ndarray = np.zeros(4)

    _controller: ControllerPID = None
    
    _gravload_compensator: bool = True

    _q0: np.ndarray = None

    _fast_simulation = True 
    
    def __init__(self, cfg: SimulationCfg):
        self._robot: rtb.ERobot = cfg.robot
        self._dt: float = cfg.dt
        self._gravity: np.ndarray = cfg.gravity
        self._decimation: int = cfg.decimation
        self._controller: ControllerPID = cfg.controller
        self._gravload_compensator: bool = cfg.gravload_compensator
        self._fast_simulation: bool = cfg.fast_simulation

        if self._robot is None:
            print("No robot given by config")
            raise ValueError
        
        if cfg.initial_q is None:
            self._q0 = np.zeros(4)
        else:
            self._q0 = cfg.initial_q.copy()

        self._robot_state["q"] = self._q0

    def step(self, command: np.ndarray):

        if self._controller is None:
            raise ValueError("No controller defined")
        
        error = command - self._robot_state["q"]
        tau = self._controller.act(error) * 1e-3

        if self._gravload_compensator:
            tau += self._robot.gravload(self._robot_state["q"])

        q = self._robot_state["q"]
        qd = self._robot_state["qd"]

        qdd = self._robot.accel(q, qd, tau, self._gravity)
        if np.any(np.isnan(qdd)) or np.any(np.isinf(qdd)):
            print(f"Integration stopped at t = {self._dt*self._step:.3f}s due to invalid qdd")
            return self._robot_state

        # explicit Euler integration
        if self._fast_simulation is True:
            qd = qd + qdd * self._dt
            q = q + qd * self._dt
        else:
            # RUNGE-KUTTA 4 using scipy.integrate.RK45
            state = np.concatenate([q, qd])
            dt = self._dt

            integrator = RK45(
                lambda t, y: np.concatenate([
                    y[len(q):],  # dq/dt = qd
                    self._robot.accel(y[:len(q)], y[len(q):], tau, self._gravity)  # qdd
                ]),
                t0=0.0,
                y0=state,
                t_bound=np.inf,
                max_step=dt
            )

            integrator.step()
            state_next = integrator.y

            q = state_next[:len(q)]
            qd = state_next[len(q):]

        # update state
        self._robot_state["q"] = q
        self._robot_state["qd"] = qd
        self._robot_state["qdd"] = qdd

        self._step += 1

        return self._robot_state


    def action(self, action):
        raise NotImplementedError

    def command(self, command: np.ndarray):
        if self._controller is None:
            print("No controller defined")
            raise NotImplementedError
        
        error = command - self._robot_state["q"]
        self._action = self._controller.act(error) * 1e-3

        if self._gravload_compensator is True:
            self._action += self._robot.gravload(command)


if __name__ == "__main__":

    def desired_trajectory(t):
        return np.array([
            0.5*np.sin(0.5*t),
            0.5*np.sin(0.4*t),
            -1 + 0.5*np.sin(0.2*t),
            0.5*np.sin(0.3*t)
        ])

    gains = np.array([
        2.42546949e+03,  8.14028079e+03,  1.22690767e+04,  1.99268404e+04,
        7.13473580e+01,  1.76258937e+02,  4.77135702e+01, 5.55385357e-04,
        9.84752097e-04, 9.38973701e-04, 8.86627296e-04,  5.87867781e+03
    ])

    cfg = SimulationCfg()
    cfg.controller = ControllerPID(gains)
    cfg.initial_q = desired_trajectory(0.0)
    cfg.fast_simulation = False

    sim = Simulation(cfg)

    num_steps = int(2.0/cfg.dt)

    state = []
    pos_q = []
    robot = GRASPE_ROBOT

    for i in tqdm(range(num_steps)):
        t = i * cfg.dt
        q_des = desired_trajectory(t)
        
        state = sim.step(q_des)
        pos_q.append(state["q"])
    
    pos_q = np.array(pos_q)
    robot.plot(pos_q, dt=0.005)