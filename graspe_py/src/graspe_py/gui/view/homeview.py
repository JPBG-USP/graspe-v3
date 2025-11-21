import numpy as np
import tkinter as tk
from graspe_py.comms import SerialLink
from graspe_py.gui.view.robotview import RobotView
from graspe_py.gui.view.controlview import ControllerView

class HomeView(tk.Frame):
    def __init__(self, parent: tk.Tk = None, q_init: np.ndarray = np.array([0.0, 0.0, 0.0, 0.0]), link: SerialLink = None):
        super().__init__(parent)

        self.gripper_status: bool = False
        self.parent = parent

        self.link: SerialLink = link
        if self.link is None:
            print("[WARN] No SerialLink was provided in HomeView, impossible to comunicate")

        self.main_panned = tk.PanedWindow(
            self,
            orient=tk.HORIZONTAL,
            sashrelief="raised",
            bg="purple"
        )
        self.main_panned.pack(fill="both", expand=True)

        self.robot_view_panned = tk.PanedWindow(
            orient=tk.VERTICAL,
            sashrelief="raised",
            background="purple"
        )
        self.robot_view_panned.pack(fill="both", expand=True)

        self.real_robot_frame = RobotView(self.robot_view_panned, "Robô Real")
        self.sim_robot_frame = RobotView(self.robot_view_panned, "Robô Simulado")

        self.robot_view_panned.add(self.sim_robot_frame, minsize=300)
        self.robot_view_panned.add(self.real_robot_frame, minsize=300)

        self.control_view = ControllerView(self, q_init, link)

        self.main_panned.add(self.robot_view_panned)
        self.main_panned.add(self.control_view, minsize=700)

        self.parent.after(33, self.update_views)


    def update_views(self):
        self.update_sim_robot_view()
        self.update_real_robot_view()
        self.parent.after(33, self.update_views)


    def update_real_robot_view(self):
        if self.link is None:
            print("[ERROR] No SerialLink was provided, impossible to update real robot view")
            return
        if not self.link.is_connected():
            print("[ERROR] No serial connection was stabilhised, did you handshake?")
            return
        
        q = self.link.obter_posicao_motor()
        if q is not None:
            self.real_robot_frame.draw_robot(q, self.gripper_status)
            

    def update_sim_robot_view(self):
        q = self.control_view.get_sim_joint_pos
        gripper_state = self.control_view.get_gripper_state
        self.sim_robot_frame.draw_robot(q, gripper_state)


    def draw_robot(self, q):
        self.real_robot_frame.draw_robot(q)
        self.sim_robot_frame.draw_robot(q)


if __name__ == "__main__":
    import tkinter as tk
    import numpy as np

    root = tk.Tk()
    root.geometry("1000x600")

    robot_view = HomeView(root, np.array([0.0, 0.0, 0.0, 0.0]))
    robot_view.pack(fill="both", expand=True)

    root.mainloop()