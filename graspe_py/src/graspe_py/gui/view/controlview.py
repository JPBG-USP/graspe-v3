import numpy as np
import tkinter as tk
from graspe_py.gui.view.plotsview import PlotView
from graspe_py.gui.view.slidersview import SlidersView
from graspe_py.gui.view.trajectoryview import TrajectoryView
from graspe_py.gui.view.serialcontrolview import SerialControlFrame


class ControllerView(tk.Frame):
    def __init__(self, parent, q_init: np.ndarray, link = None):
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

        self.log = {
            'desired_pos': [],
            'real_pos': [],
        }

        # Views
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)

        self.slider_view = SlidersView(self, q_init)
        self.slider_view.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)

        self.serial_frame = SerialControlFrame(self, q_init, link)
        self.serial_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)

        self.trajectory_frame = TrajectoryView(self, link)
        self.trajectory_frame.grid(row=1, column=1, sticky="nsew", padx=10, pady=10)

        self.plot_frame = PlotView(self)
        self.plot_frame.grid(row=1, column=0, sticky="new", padx=10, pady=10)


    @property
    def get_gripper_state(self):
        return self.serial_frame.get_gripper_state


    @property
    def get_sim_joint_pos(self):

        # Simulated trajectory
        if self.trajectory_frame.sim_traj:
            self.iteration += 1
            if self.iteration >= self.trajectory_frame.jtraj.shape[0]:
                self.trajectory_frame.sim_traj = False
                self.iteration = 0
                self.trajectory_frame.btn_sim_traj.config(text="Simular Trajetória", bg="orange")
            return self.trajectory_frame.jtraj[self.iteration,:]
        return self.slider_view.get_joint_pos

    @property
    def get_sim_state(self):

        # Simulated trajectory
        if self.trajectory_frame.sim_traj:

            # process iteration
            self.iteration += 1
            if self.iteration >= self.trajectory_frame.jtraj.shape[0]:
                self.trajectory_frame.sim_traj = False
                self.iteration = 0
                self.trajectory_frame.btn_sim_traj.config(text="Simular Trajetória", bg="orange")
                return self.slider_view.get_joint_pos, self.serial_frame.get_gripper_state

            # Save Log
            self.log['desired_pos'].append(list(self.trajectory_frame.jtraj[self.iteration,:]))
            return self.trajectory_frame.jtraj[self.iteration,:], self.trajectory_frame.gripper_traj[self.iteration]
        
        # Real trajectory
        if self.serial_frame.real_traj and not self.trajectory_frame.sim_traj:

            # Process iteration
            self.iteration += 1
            if self.iteration >= self.trajectory_frame.jtraj.shape[0]:
                self.serial_frame.real_traj = False
                self.iteration = 0
                self.serial_frame.btn_send_traj.config(bg="lightgray")
                return self.slider_view.get_joint_pos, self.serial_frame.get_gripper_state

            self.serial_frame.send_position(self.trajectory_frame.jtraj[self.iteration,:], self.trajectory_frame.gripper_traj[self.iteration])

            # Save Log
            self.log['desired_pos'].append(list(self.trajectory_frame.jtraj[self.iteration,:]))
            real_q, _ = self.get_real_state
            if real_q is not None:
                self.log['real_pos'].append(list(real_q))
            return self.trajectory_frame.jtraj[self.iteration,:], self.trajectory_frame.gripper_traj[self.iteration]
        
        # Default - slider position
        return self.slider_view.get_joint_pos, self.serial_frame.get_gripper_state
    

    @property
    def get_real_state(self):
        return self.serial_frame.get_real_robot_joint(), self.serial_frame.get_gripper_state
    

if __name__ == "__main__":
    import tkinter as tk
    import numpy as np

    root = tk.Tk()
    root.geometry("1000x600")

    robot_view = ControllerView(root, q_init=np.array([0.0, 0.0, 0.0, 0.0]))
    robot_view.pack(fill="both", expand=True)

    root.mainloop()