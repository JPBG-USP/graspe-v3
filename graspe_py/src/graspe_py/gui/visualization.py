from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from graspe_py.simulation import GRASPE_ROBOT

def inicializar_visualizacao(frame_esquerda):
    import tkinter as tk

    frame_visualizacao = tk.Frame(frame_esquerda, bg="white")
    frame_visualizacao.pack(fill="both", expand=True, padx=20, pady=20)

    titulo_label = tk.Label(
        frame_visualizacao,
        text="Visualização 3D",
        bg="white",
        font=("Arial", 14, "bold")
    )
    titulo_label.pack(pady=(10, 5))

    fig = Figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")

    canvas = FigureCanvasTkAgg(fig, master=frame_visualizacao)
    canvas.draw()
    canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=10)

    return fig, ax, canvas

def draw_robot(q_rad, ax, canvas):
    Ts = GRASPE_ROBOT.fkine_all(q_rad)
    xs, ys, zs = [0.0], [0.0], [0.0]
    for T in Ts:
        p = T.t
        xs.append(float(p[0]))
        ys.append(float(p[1]))
        zs.append(float(p[2]))

    ax.cla()
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.plot(xs, ys, zs, marker="o", lw=4, ms=7, c="purple")
    ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], c="r", s=40)

    alcance = np.sum(np.abs([1.672, 1.26, 1.26, 0.5]))*1e-1
    ax.set_xlim([-alcance, alcance])
    ax.set_ylim([-alcance, alcance])
    ax.set_zlim([0, alcance])
    ax.view_init(elev=30, azim=45)

    canvas.draw_idle()
