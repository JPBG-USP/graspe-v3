from .trajectory import gerar_trajetoria, simular_trajetoria, plot_q, plot_qd, plot_qdd
import tkinter as tk
import numpy as np
from graspe_py.simulation import GRASPE_ROBOT


def criar_frame_registro(frame_direita, q_atual_deg, q_real_deg, link, draw_robot, janela, ax, canvas):
    pos_ini_var = tk.StringVar(value="Posição inicial: —")
    pos_fim_var = tk.StringVar(value="Posição final: —")
    usar_real = tk.BooleanVar(value=False)
    
    # Estado da garra (False = aberta, True = fechada)
    gripper_state = tk.BooleanVar(value=False)

    def on_gripper_toggle():
        # Delay para garantir que a posição já foi enviada/estabilizada antes de mexer na garra
        def _send():
            if gripper_state.get():
                link.send("GRIPPERON")
            else:
                link.send("GRIPPEROFF")
        janela.after(200, _send)


    def on_switch_change():
        if usar_real.get():
            # Switch ativado → usar posição real → liberar motores
            link.send("MOTORSOFF")
        else:
            # Switch desativado → usar posição simulada → travar motores
            link.send("MOTORSON")

            # Enviar posição REAL como setpoint
            q_real_rad = np.deg2rad(q_real_deg)
            print(q_real_rad)
            msg = f"SETALLQ {q_real_rad[0]:.3f} {q_real_rad[1]:.3f} {q_real_rad[2]:.3f} {q_real_rad[3]:.3f}"
            link.send(msg)

    q_ini_saved = {"value": None}
    q_fim_saved = {"value": None}
    traj_saved = {}
    states_log = []  # armazena estados da simulação

    def registrar_pos_ini():
        q_deg_fonte = q_real_deg if usar_real.get() else q_atual_deg
        q_rad = np.deg2rad(q_deg_fonte)
        T = GRASPE_ROBOT.fkine(q_rad)
        p = T.t
        q_ini_saved["value"] = q_rad
        estado_garra = "Fechada" if gripper_state.get() else "Aberta"
        pos_ini_var.set(
        "Posição inicial:\n"
        f"Juntas={np.round(q_deg_fonte,1)}°\n"
        f"Garra: {estado_garra}\n"
        f"EE=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m"
    )

    def registrar_pos_fim():
        q_deg_fonte = q_real_deg if usar_real.get() else q_atual_deg
        q_rad = np.deg2rad(q_deg_fonte)
        T = GRASPE_ROBOT.fkine(q_rad)
        p = T.t
        q_fim_saved["value"] = q_rad
        estado_garra = "Fechada" if gripper_state.get() else "Aberta"
        pos_fim_var.set(
        "Posição final:\n"
        f"Juntas={np.round(q_deg_fonte,1)}°\n"
        f"Garra: {estado_garra}\n"
        f"EE=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m"
    )

    frame_registro = tk.Frame(frame_direita, bg="lightgray")
    frame_registro.pack(fill="x", padx=10, pady=10)

    tk.Checkbutton(
        frame_registro,
        text="Usar posição REAL (desligar motores)",
        variable=usar_real,
        command=on_switch_change,
        bg="lightgray"
    ).pack(fill="x", pady=4)

    tk.Checkbutton(
        frame_registro,
        text="Fechar garra",
        variable=gripper_state,
        command=on_gripper_toggle,
        bg="lightgray"
    ).pack(fill="x", pady=4)

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

    return gripper_state
