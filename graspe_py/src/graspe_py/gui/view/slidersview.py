import math
import numpy as np
import tkinter as tk
from graspe_py.simulation import GRASPE_ROBOT

class SlidersView(tk.LabelFrame):
    def __init__(self, parent, q_init: np.ndarray):
        super().__init__(parent, text="Juntas (graus)", bg="lightgray", padx=10, pady=10)
        

        # Variables
        self._q: np.ndarray = q_init
        self._sliders = []
        self._joint_limits_deg = np.rad2deg(GRASPE_ROBOT.qlim)
        self.parent = parent

        # Seting up sliders
        def update_joints(idx: int, angle: float):
            self._q[idx] = math.radians(angle)
            self.parent.parent.sim_robot_frame.draw_robot(self._q, gripper=False)

        for i in range(4):
            lo, hi = float(self._joint_limits_deg[0, i]), float(self._joint_limits_deg[1, i]) 
            slider = tk.Scale(
                self,
                label=f"Junta {i+1}",
                from_=lo, to=hi,
                orient=tk.HORIZONTAL,
                resolution=0.1,
                length=250,
                bg="lightgray",
                command= lambda angle, j=i: update_joints(j, float(angle))
            )
            slider.set(math.degrees(q_init[i]))
            slider.pack(padx=10, pady=6, fill="x")
            self._sliders.append(slider)

    @property
    def get_joint_pos(self) -> np.ndarray:
        return  self._q
    
    @property
    def joint_limits(self, degree: bool = False) -> np.ndarray:
        if degree:
            return self._joint_limits_deg
        else:
            return np.deg2rad(self._joint_limits_deg)
    