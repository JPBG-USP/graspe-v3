import argparse


"""
    Parsing command line arguments
"""
parser = argparse.ArgumentParser(description="Running simulation of Graspe robot")

parser.add_argument(
    "--motor_model",
    action="store_true",
    help="Run simulation with motor model enabled"
)
parser.add_argument(
    "--time",
    type=float,
    default=2.0,
    help="Total simulation time in seconds"
)

args = parser.parse_args()


def run_simulation_without_motor_model():

    import numpy as np
    from tqdm import tqdm
    from graspe_py.simulation import Simulation, SimulationCfg
    from graspe_py.simulation import ControllerPID
    from graspe_py.simulation import GRASPE_ROBOT

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

    num_steps = int(args.time/cfg.dt)

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


def run_simulation_with_motor_model():
    raise NotImplementedError("Simulation with motor model is not implemented yet.")

if __name__ == "__main__":
    if args.motor_model:
        run_simulation_with_motor_model()
    else:
        run_simulation_without_motor_model()