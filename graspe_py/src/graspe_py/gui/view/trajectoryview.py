import numpy as np
import tkinter as tk
from graspe_py.trajectory.traj_planner import generate_traj

class TrajectoryView(tk.LabelFrame):
    def __init__(self, parent, link = None):
        super().__init__(parent, text="Planejador de Trajetória", bg="lightgray", padx=10, pady=10)
        
        if link is None:
            print("[WARN] No SerialLink was provided in SerialControlFrame")
        
        self.parent = parent
        self.link = link
        self.q_list = []
        self.gripper_list = []
        self.q_string: str = ""

        self.jtraj = None
        self.gripper_traj = None
        self.has_trajectory: bool = False
        self.sim_traj: bool = False

        # Buttons
        self.btn_sim_traj = tk.Button(
            self, text="Simular Trajetória",
            bg="orange", command=self.simulate_trajectory
        )
        self.btn_sim_traj.pack(fill="x", pady=(4, 8))

        tk.Label(self, text="Registrar Posições:", bg="lightgray", font=("Arial", 10, "bold")).pack(anchor="w")
        self.btn_reg_sim = tk.Button(
            self, text="Registrar Posição Simulada",
            command=self.register_sim_pos
        )
        self.btn_reg_sim.pack(fill="x", pady=3)
        self.btn_reg_sim = tk.Button(
            self, text="Registrar Posição Real",
            command=self.register_real_pos
        )
        self.btn_reg_sim.pack(fill="x", pady=3)

        self.btn_clean = tk.Button(
            self, text="Limpar trajetória",
            command=self.clean_traj
        )
        self.btn_clean.pack(fill="x", pady=3)

        tk.Label(self, text="Trajetória:", bg="lightgray", font=("Arial", 10, "bold")).pack(anchor="w")

        self.pos_label = tk.StringVar(value="Nenhuma posição fornecida")
        tk.Label(self, textvariable=self.pos_label, justify="left", bg="white", anchor="w").pack(fill="x")


    def register_sim_pos(self):
        # Get sim state
        q = self.parent.get_sim_joint_pos
        gripper = self.parent.get_gripper_state
        
        # Append state
        self.q_list.append(list(q))
        self.gripper_list.append(gripper)
        self.q_string += (
            f"Juntas={np.array2string(q, formatter={'float_kind':lambda x: f'{x:.2f}'})}, "
            f"Garra: {'Fechado' if gripper else 'Aberto'}\n"
        )
        # Update state
        self.pos_label.set(self.q_string)


    def register_real_pos(self):
        # Get real state
        q : np.array = np.asarray(self.parent.serial_frame.get_real_robot_joint())
        gripper = self.parent.get_gripper_state
        
        # Append state
        self.q_list.append(list(q))
        self.gripper_list.append(gripper)
        self.q_string += (
            f"Juntas={np.array2string(q, formatter={'float_kind':lambda x: f'{x:.2f}'})}, "
            f"Garra: {'Fechado' if gripper else 'Aberto'}\n"
        )
        # Update state
        self.pos_label.set(self.q_string)



    def clean_traj(self):
        self.q_string = ""
        self.q_list = []
        self.gripper_list = []
        self.pos_label.set("Nenhuma posição fornecida")
        self.has_trajectory = False


    def simulate_trajectory(self):

        # Seek for existing trajectory
        if not self.has_trajectory:
            self._create_trajectory()
            if self.has_trajectory:
                print("[INFO] Trajectory created successfully")
            else:
                print("[ERROR] Could not create trajectory, check if all positions are reachable")
                return
        
        self.btn_sim_traj.config(text="Simulando Trajetória", bg="darkgray")
        self.sim_traj = True


    def _create_trajectory(self):

        # Check if there are positions
        if len(self.q_list) == 0:
            return None, None
        
        # Generate trajectory
        q_array = np.array(self.q_list)
        jtraj, wtraj, gripper_traj = generate_traj(q_array, self.gripper_list)

        # Check if trajectory was created
        if jtraj is None or wtraj is None:
            return None, None
        
        self.jtraj = jtraj
        self.gripper_traj = gripper_traj
        self.has_trajectory = True
