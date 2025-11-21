import numpy as np
import tkinter as tk
from matplotlib.figure import Figure
from graspe_py.simulation import GRASPE_ROBOT
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class RobotView(tk.Frame):
    def __init__(self, parent, title: str = None):
        super().__init__(parent)

        self.configure(bg="white", width=700, height=300)

        self.q = np.zeros(4)
        self.gripper_status: bool = False

        # Setup title
        self.title_label = tk.Label(
            self,
            text=title or "Robot View",
            bg="white",
            font=("Arial", 14, "bold")
        )
        self.title_label.pack(side="top", fill="x", pady=5)
        self.title = title or "Robot View"

        # Setup matplotlib figure
        self.figure = Figure(figsize=(10,10))
        self.ax = self.figure.add_subplot(111, projection="3d")
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.set_zlabel("Z [m]")
        # self.ax.set_title(self.title)

        self.max_distance = np.sum(np.abs([1.672, 1.26, 1.26, 0.5]))*1e-1
        self.ax.set_xlim([-self.max_distance, self.max_distance])
        self.ax.set_ylim([-self.max_distance, self.max_distance])
        self.ax.set_zlim([0, self.max_distance])

        # Setup Tkinter canvas
        self.canvas = FigureCanvasTkAgg(self.figure, master=self)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side="top", fill="both", expand=True, padx=10, pady=10)


    def draw_robot(self, q: np.ndarray, gripper: bool = False):

        self.q = q
        self.gripper_status = gripper

        # Getting robot information
        Ts = GRASPE_ROBOT.fkine_all(q)
        xs, ys, zs = [0.0], [0.0], [0.0]
        for T in Ts:
            p = T.t
            xs.append(float(p[0]))
            ys.append(float(p[1]))
            zs.append(float(p[2]))

        # Setup label and limits
        self.ax.cla()
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.set_zlabel("Z [m]")
        # self.ax.set_title(self.title)
        self.ax.set_xlim([-self.max_distance, self.max_distance])
        self.ax.set_ylim([-self.max_distance, self.max_distance])
        self.ax.set_zlim([0, self.max_distance])

        # Plotting manipulator
        self.ax.plot(xs, ys, zs, marker="o", lw=4, ms=7, c="purple")
        self.ax.scatter(xs[-1], ys[-1], 0.0, c="red", s=40)
        if gripper:
            self.ax.scatter(xs[-1], ys[-1], zs[-1], c="green", s=42)
        else:
            self.ax.scatter(xs[-1], ys[-1], zs[-1], c="red", s=42)
        self.ax.view_init(elev=30, azim=90)

        self.canvas.draw_idle()

    def get_joint_pos(self) -> np.ndarray:
        return self.q
    
    def get_gripper_status(self) -> bool:
        return self.gripper_status


if __name__ == "__main__":
    import tkinter as tk
    import numpy as np

    root = tk.Tk()
    root.geometry("1000x600")

    robot_view = RobotView(root, title="Grasp-e Robot")
    robot_view.pack(fill="both", expand=True)

    q = np.array([0.1, 0.2, -0.1, 0.3])
    robot_view.draw_robot(q, gripper=True)

    root.mainloop()