from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from graspe_py.simulation import GRASPE_ROBOT
import tkinter as tk

def robot_view(frame: tk.Frame, title: str | None = None):

    view_frame = tk.Frame(frame, bg="white")
    view_frame.pack(fill="both", expand=True, padx=10, pady=10)

    title = tk.Label(
        view_frame,
        text= title if title is not None else "Vizualização 3D",
        bg="white",
        font=("Arial", 14, "bold")
    )
    title.pack(pady=(5, 5))

    fig = Figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")

    canvas = FigureCanvasTkAgg(fig, master=view_frame)
    canvas.draw()
    canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=10)

    return fig, ax, canvas

def draw_robot(q_rad, ax, canvas, garra: tk.BooleanVar = None):
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
    ax.plot(xs[-1], ys[-1], 0, marker="o", lw=4, ms=7, c="red")
    if garra.get() is not None:
        if garra is True:
            ax.plot(xs[-1], ys[-1], zs[-1], marker="o", lw=4, ms=7, c="green")
        else:
            ax.plot(xs[-1], ys[-1], zs[-1], marker="o", lw=4, ms=7, c="red")
    ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], c="r", s=40)

    alcance = np.sum(np.abs([1.672, 1.26, 1.26, 0.5]))*1e-1
    ax.set_xlim([-alcance, alcance])
    ax.set_ylim([-alcance, alcance])
    ax.set_zlim([0, alcance])
    ax.view_init(elev=30, azim=90)

    canvas.draw_idle()
