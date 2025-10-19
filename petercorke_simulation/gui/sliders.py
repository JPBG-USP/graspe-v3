import tkinter as tk
from tkinter import Scale, HORIZONTAL
import numpy as np

def criar_sliders(frame_direita, q_init_deg, qlims_deg, q_atual_deg, draw_robot, ax, canvas):
    n = len(q_init_deg)

    tk.Label(frame_direita, text="Juntas (graus)", bg="green").pack(pady=(8, 0), anchor="w")
    sliders = []

    def atualizar_junta(i, valor):
        q_atual_deg[i] = float(valor)
        q_rad = np.deg2rad(q_atual_deg)
        draw_robot(q_rad, ax, canvas)

    for i in range(n):
        lo, hi = float(qlims_deg[0, i]), float(qlims_deg[1, i])
        tk.Label(frame_direita, text=f"Junta {i+1}", bg="green").pack(anchor="w", padx=10)

        s = Scale(
            frame_direita,
            from_=lo, to=hi,
            orient=HORIZONTAL,
            resolution=1,
            length=250,
            command=lambda val, j=i: atualizar_junta(j, val)
        )
        s.set(q_init_deg[i])
        s.pack(padx=10, pady=6, fill="x")
        sliders.append(s)

    return sliders
