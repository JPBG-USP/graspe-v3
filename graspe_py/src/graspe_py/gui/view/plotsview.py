import tkinter as tk
import matplotlib.pyplot as plt
import numpy as np

class PlotView(tk.LabelFrame):
    def __init__(self,parent):
        super().__init__(parent,text="Plots",bg="lightgray",padx=10,pady=10)

        self.parent = parent

        self.btn_q1 = tk.Button(
            self,text="Junta 1",
            command= lambda idx = 0: self.plot_joints(idx)
        )
        self.btn_q1.pack(fill="x", pady=3)

        self.btn_q2 = tk.Button(
            self,text="Junta 2",
            command= lambda idx = 1: self.plot_joints(idx)
        )
        self.btn_q2.pack(fill="x", pady=3)

        self.btn_q3 = tk.Button(
            self,text="Junta 3",
            command= lambda idx = 2: self.plot_joints(idx)
        )
        self.btn_q3.pack(fill="x", pady=3)

        self.btn_q4 = tk.Button(
            self,text="Junta 4",
            command= lambda idx = 3: self.plot_joints(idx)
        )
        self.btn_q4.pack(fill="x", pady=3)


    def plot_joints(self,idx):

        logs = self.parent.log
        traj_array = np.array(logs['desired_pos'])
        real_array = np.array(logs['real_pos'])
    

        if traj_array.shape[0] == 0 and real_array.shape[0] == 0:
            print("[ERROR] No log data available to plot")
            return

        if not real_array.shape[0] == 0:
            q_real  = real_array[:,idx]
            qd_real = np.diff(real_array[:,idx], n=1, prepend=real_array[0,idx], append=real_array[-1,idx])
            qdd_real = np.diff(real_array[:,idx], n=2, prepend=real_array[0,idx], append=real_array[-1,idx])
        else:
            q_real = np.array([])
            qd_real = np.array([])
            qdd_real =  np.array([])

        if not traj_array.shape[0] == 0:
            q_traj  = traj_array[:,idx]
            qd_traj = np.diff(traj_array[:,idx], n=1, prepend=traj_array[0,idx], append=traj_array[-1,idx])
            qdd_traj = np.diff(traj_array[:,idx], n=2, prepend=traj_array[0,idx], append=traj_array[-1,idx])
        else:
            q_traj = np.array([])
            qd_traj = np.array([])
            qdd_traj = np.array([])

        fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
        fig.suptitle(f"Junta {idx+1} - q, qd, qdd", fontsize=14)

        # --- q ---
        ax = axes[0]
        if q_traj.shape[0] > 1:
            ax.plot(q_traj, label="q traj", linewidth=2)
        if q_real.shape[0] > 1:
            ax.plot(q_real, label="q real", linestyle=":")
        ax.set_ylabel("q [rad]")
        ax.grid(True)
        ax.legend(loc="upper right")

        # --- qd ---
        ax = axes[1]
        if qd_traj.shape[0] > 1:
            ax.plot(qd_traj, label="qd traj", linewidth=2)
        if qd_real.shape[0] > 1:
            ax.plot(qd_real, label="qd real", linestyle=":")
        ax.set_ylabel("qd [rad/s]")
        ax.grid(True)
        ax.legend(loc="upper right")

        # --- qdd ---
        ax = axes[2]
        if qdd_traj.shape[0] > 1:
            ax.plot(qdd_traj, label="qdd traj", linewidth=2)
        if qdd_real.shape[0] > 1:
            ax.plot(qdd_real, label="qdd real", linestyle=":")
        ax.set_ylabel("qdd [rad/s²]")
        ax.set_xlabel("Tempo [s]")
        ax.grid(True)
        ax.legend(loc="upper right")

        fig.tight_layout()
        plt.show() 