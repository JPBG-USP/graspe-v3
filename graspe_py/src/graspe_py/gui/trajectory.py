import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import jtraj
from graspe_py.simulation import Simulation, SimulationCfg, ControllerPID


def gerar_trajetoria(q_ini_saved, q_fim_saved, traj_saved):
    if q_ini_saved["value"] is None or q_fim_saved["value"] is None:
        print("⚠️ Registre posição inicial e final primeiro!")
        return
    t = np.linspace(0, 5, int(5/0.005))
    traj_saved["t"] = t
    traj_saved["value"] = jtraj(q_ini_saved["value"], q_fim_saved["value"], t)
    print("✅ Trajetória gerada e armazenada!")


def simular_trajetoria(traj_saved, q_ini_saved, draw_robot, janela, ax, canvas, states_log):
    if traj_saved.get("value", None) is None:
        print("⚠️ Gere uma trajetória primeiro!")
        return

    gains = np.array([
        2.4e3, 8.1e3, 1.22e4, 1.99e4,
        71.3, 176.2, 47.7, 5.5e-4,
        9.8e-4, 9.3e-4, 8.8e-4, 5878.6
    ])

    cfg = SimulationCfg()
    cfg.controller = ControllerPID(gains)
    cfg.initial_q = q_ini_saved["value"]

    sim = Simulation(cfg)
    num_steps = int(5.0/cfg.dt)

    states_log.clear()  # limpa anteriores

    for i in range(num_steps):
        q_des = traj_saved["value"].q[i]
        state = sim.step(q_des)
        # guardamos também o desejado
        state["q_des"] = q_des
        state["qd_des"] = traj_saved["value"].qd[i]
        state["qdd_des"] = traj_saved["value"].qdd[i]
        # torques desejados (não vem do jtraj, deixo zeros por enquanto)
        state["tau_des"] = np.zeros_like(q_des)
        states_log.append(state.copy())

        if i % 5 == 0:
            draw_robot(state["q"], ax, canvas)
            janela.update_idletasks()
            janela.update()


# --------- FUNÇÕES DE PLOTAGEM --------- #

def _plot_states(states_log, key, key_des, ylabel, title):
    if not states_log:
        print(f"⚠️ Nenhum dado para {key}")
        return

    n_joints = len(states_log[0][key])
    t = np.arange(len(states_log)) * 0.005

    plt.figure(figsize=(10, 2.5 * n_joints))
    for j in range(n_joints):
        plt.subplot(n_joints, 1, j+1)
        # curva desejada
        y_des = np.array([s[key_des][j] for s in states_log])
        plt.plot(t, y_des, "k--", label="desejada")
        # curva simulada
        y_sim = np.array([s[key][j] for s in states_log])
        plt.plot(t, y_sim, "b", label="simulada")
        # curva real (por enquanto comentada)
        # y_real = ...
        # plt.plot(t, y_real, "r", label="real")
        plt.ylabel(f"Junta {j+1} [{ylabel}]")
        plt.legend(loc="best")
        if j == 0:
            plt.title(title)
    plt.xlabel("Tempo [s]")
    plt.tight_layout()
    plt.show()


def plot_q(states_log):
    _plot_states(states_log, "q", "q_des", "rad", "Posição das Juntas")


def plot_qd(states_log):
    _plot_states(states_log, "qd", "qd_des", "rad/s", "Velocidade das Juntas")


def plot_qdd(states_log):
    _plot_states(states_log, "qdd", "qdd_des", "rad/s²", "Aceleração das Juntas")


#def plot_tau(states_log):
#    _plot_states(states_log, "tau", "tau_des", "Nm", "Torque das Juntas")
