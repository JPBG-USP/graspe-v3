import tkinter as tk
import matplotlib.pyplot as plt
import numpy as np

class PlotView(tk.LabelFrame):
    def __init__(self,parent):
        super().__init__(parent,text="Plots",bg="lightgray",padx=10,pady=10)

        t = np.linspace(0, 5, 300)
        self.t = t
        N = len(t)
        J = 4   # número de juntas

        # q(t)
        self.q_traj = np.column_stack([np.sin(t + i) for i in range(J)])
        self.q_sim  = np.column_stack([np.sin(t + i) + 0.03*np.random.randn(N) for i in range(J)])
        self.q_real = np.column_stack([np.sin(t + i) + 0.06*np.random.randn(N) for i in range(J)])

        # qd(t)
        self.qd_traj = np.column_stack([np.cos(t + i) for i in range(J)])
        self.qd_sim  = np.column_stack([np.cos(t + i) + 0.03*np.random.randn(N) for i in range(J)])
        self.qd_real = np.column_stack([np.cos(t + i) + 0.06*np.random.randn(N) for i in range(J)])

        # qdd(t)
        self.qdd_traj = np.column_stack([0.5*np.sin(2*t + i) for i in range(J)])
        self.qdd_sim  = np.column_stack([0.5*np.sin(2*t + i) + 0.03*np.random.randn(N) for i in range(J)])
        self.qdd_real = np.column_stack([0.5*np.sin(2*t + i) + 0.06*np.random.randn(N) for i in range(J)])



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
        t = self.t

        # Seleciona série da junta
        q_traj  = self.q_traj[:,idx]
        q_sim   = self.q_sim[:,idx]
        q_real  = self.q_real[:,idx]

        qd_traj = self.qd_traj[:,idx]
        qd_sim  = self.qd_sim[:,idx]
        qd_real = self.qd_real[:,idx]

        qdd_traj = self.qdd_traj[:,idx]
        qdd_sim  = self.qdd_sim[:,idx]
        qdd_real = self.qdd_real[:,idx]

        fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
        fig.suptitle(f"Junta {idx+1} - q, qd, qdd", fontsize=14)

        # --- q ---
        ax = axes[0]
        ax.plot(t, q_traj, label="q traj", linewidth=2)
        ax.plot(t, q_sim,  label="q sim",  linestyle="--")
        ax.plot(t, q_real, label="q real", linestyle=":")
        ax.set_ylabel("q [rad]")
        ax.grid(True)
        ax.legend(loc="upper right")

        # --- qd ---
        ax = axes[1]
        ax.plot(t, qd_traj, label="qd traj", linewidth=2)
        ax.plot(t, qd_sim,  label="qd sim",  linestyle="--")
        ax.plot(t, qd_real, label="qd real", linestyle=":")
        ax.set_ylabel("qd [rad/s]")
        ax.grid(True)
        ax.legend(loc="upper right")

        # --- qdd ---
        ax = axes[2]
        ax.plot(t, qdd_traj, label="qdd traj", linewidth=2)
        ax.plot(t, qdd_sim,  label="qdd sim",  linestyle="--")
        ax.plot(t, qdd_real, label="qdd real", linestyle=":")
        ax.set_ylabel("qdd [rad/s²]")
        ax.set_xlabel("Tempo [s]")
        ax.grid(True)
        ax.legend(loc="upper right")

        fig.tight_layout()
        plt.show() 