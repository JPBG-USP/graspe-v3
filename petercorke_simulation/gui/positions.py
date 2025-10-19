from .trajectory import gerar_trajetoria, simular_trajetoria, plot_q, plot_qd, plot_qdd
import tkinter as tk
import numpy as np
from ..robot_description import GRASPE_ROBOT


def criar_frame_registro(frame_direita, q_atual_deg, draw_robot, janela, ax, canvas):
    pos_ini_var = tk.StringVar(value="Posição inicial: —")
    pos_fim_var = tk.StringVar(value="Posição final: —")

    q_ini_saved = {"value": None}
    q_fim_saved = {"value": None}
    traj_saved = {}
    states_log = []  # armazena estados da simulação

    def registrar_pos_ini():
        q_rad = np.deg2rad(q_atual_deg)
        T = GRASPE_ROBOT.fkine(q_rad)
        p = T.t
        q_ini_saved["value"] = q_rad
        pos_ini_var.set(f"Posição inicial:\nJuntas={np.round(q_atual_deg,1)}°\nEE=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m")

    def registrar_pos_fim():
        q_rad = np.deg2rad(q_atual_deg)
        T = GRASPE_ROBOT.fkine(q_rad)
        p = T.t
        q_fim_saved["value"] = q_rad
        pos_fim_var.set(f"Posição final:\nJuntas={np.round(q_atual_deg,1)}°\nEE=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m")

    frame_registro = tk.Frame(frame_direita, bg="lightgray")
    frame_registro.pack(fill="x", padx=10, pady=10)

    tk.Button(frame_registro, text="Registrar Posição Inicial", command=registrar_pos_ini).pack(fill="x", pady=4)
    tk.Label(frame_registro, textvariable=pos_ini_var, justify="left", bg="white", anchor="w").pack(fill="x")

    tk.Button(frame_registro, text="Registrar Posição Final", command=registrar_pos_fim).pack(fill="x", pady=4)
    tk.Label(frame_registro, textvariable=pos_fim_var, justify="left", bg="white", anchor="w").pack(fill="x")

    tk.Button(frame_registro, text="Gerar Trajetória", command=lambda: gerar_trajetoria(q_ini_saved, q_fim_saved, traj_saved), bg="orange").pack(fill="x", pady=4)
    tk.Button(frame_registro, text="Simular Trajetória", command=lambda: simular_trajetoria(traj_saved, q_ini_saved, draw_robot, janela, ax, canvas, states_log), bg="yellow").pack(fill="x", pady=4)

    # NOVOS BOTÕES DE PLOT
    tk.Button(frame_registro, text="Plotar q",   command=lambda: plot_q(states_log),   bg="lightblue").pack(fill="x", pady=2)
    tk.Button(frame_registro, text="Plotar qd",  command=lambda: plot_qd(states_log),  bg="lightblue").pack(fill="x", pady=2)
    tk.Button(frame_registro, text="Plotar qdd", command=lambda: plot_qdd(states_log), bg="lightblue").pack(fill="x", pady=2)
    #tk.Button(frame_registro, text="Plotar tau", command=lambda: plot_tau(states_log), bg="lightblue").pack(fill="x", pady=2)
