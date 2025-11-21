import math
import numpy as np
import tkinter as tk
from matplotlib.figure import Figure
from graspe_py.simulation import GRASPE_ROBOT
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
# from .serialcontrolview import SerialControlFrame
from graspe_py.gui.view.serialcontrolview import SerialControlFrame
from graspe_py.gui.view.trajectoryview import TrajectoryView

from graspe_py.gui.view.slidersview import SlidersView


class ControllerView(tk.Frame):
    def __init__(self, parent, q_init: np.ndarray):
        super().__init__(parent)

        self.parent = parent

        # Setting frame config
        self.configure(
            bg="green",
            width=300,
            height=600
        )

        # State variables
        self.q_init = q_init
        self._q: np.ndarray = self.q_init
        self.gripper_state: bool = False
        self.iteration: int = 0

        # Views

        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)


        self.slider_view = SlidersView(self, q_init)
        self.slider_view.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        # self.slider_view.pack(fill="x", padx=10, pady=10)

        self.serial_frame = SerialControlFrame(self, q_init, None)
        self.serial_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)

        # self.serial_frame.pack(fill="x", padx=10, pady=10)

        self.trajectory_frame = TrajectoryView(self, None)
        self.trajectory_frame.grid(row=1, column=1, sticky="nsew", padx=10, pady=10)

        # self.trajectory_frame.pack(fill="x", padx=10, pady=10)

        # TODO: Graphs Frame

    @property
    def get_gripper_state(self):
        return self.serial_frame.get_gripper_state

    @property
    def get_sim_joint_pos(self):
        if self.trajectory_frame.sim_traj:
            self.iteration += 1
            if self.iteration >= self.trajectory_frame.jtraj.shape[0]:
                self.trajectory_frame.sim_traj = False
                self.iteration = 0
            return self.trajectory_frame.jtraj[self.iteration,:]
        return self.slider_view.get_joint_pos


if __name__ == "__main__":
    import tkinter as tk
    import numpy as np

    root = tk.Tk()
    root.geometry("1000x600")

    robot_view = ControllerView(root, q_init=np.array([0.0, 0.0, 0.0, 0.0]))
    robot_view.pack(fill="both", expand=True)

    root.mainloop()