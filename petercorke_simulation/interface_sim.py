import tkinter as tk
import numpy as np
from roboticstoolbox import jtraj
from .robot_description import GRASPE_ROBOT
from .simulation_class import *


qz = np.array([-0.994, 1.57, 0.64, 0.75])  # estado inicial (rad)


# ===================== Janela e Layout =====================
janela = tk.Tk()
janela.title("Simulação Tkinter")
janela.geometry("1000x600")
janela.configure(bg="purple")

paned = tk.PanedWindow(janela, orient=tk.HORIZONTAL, sashrelief="raised", bg="purple")
paned.pack(fill="both", expand=True)

frame_esquerda = tk.Frame(paned, bg="purple", width=700, height=600)
frame_direita  = tk.Frame(paned, bg="green", width=300, height=600)
paned.add(frame_esquerda)
paned.add(frame_direita)

# ===================== Frame de Visualização =====================
frame_visualizacao = tk.Frame(frame_esquerda, bg="white")
frame_visualizacao.pack(fill="both", expand=True, padx=20, pady=20)

# --- título acima do gráfico ---
titulo_label = tk.Label(
    frame_visualizacao,
    text="Visualização em palito",
    bg="white",
    font=("Arial", 14, "bold")
)
titulo_label.pack(pady=(10, 5))

# ===================== Matplotlib (TkAgg) =====================
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

fig = Figure(figsize=(6, 6))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")

# fundo branco
fig.patch.set_facecolor("white")
ax.set_facecolor("white")

canvas = FigureCanvasTkAgg(fig, master=frame_visualizacao)
canvas.draw()
canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=10)

# ===================== Função de desenho =====================
def draw_robot(q_rad):
    Ts = GRASPE_ROBOT.fkine_all(q_rad)   # lista de transformações homogêneas
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
    # aumenta a espessura da linha (lw) e o tamanho do marcador (ms)
    ax.plot(xs, ys, zs, marker="o", lw=4, ms=7, c="purple")

    # bolinha do efetuador maiorzinho
    ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], c="r", s=40)

    alcance = np.sum(np.abs([1.672, 1.26, 1.26, 0.5]))*1e-1
    ax.set_xlim([-alcance, alcance])
    ax.set_ylim([-alcance, alcance])
    ax.set_zlim([0, alcance])
    ax.view_init(elev=30, azim=45)
    ax.set_facecolor("white")  # manter fundo após cla()
    canvas.draw_idle()

# ===================== Sliders na DIREITA (graus) =====================
from tkinter import Scale, HORIZONTAL

n = GRASPE_ROBOT.n  # deve ser 4
qlims = np.asarray(GRASPE_ROBOT.qlim, dtype=float)
if qlims.shape == (n, 2):   # caso venha como (n,2)
    qlims = qlims.T
assert qlims.shape == (2, n), f"qlim esperado (2,{n}) ou ({n},2), obtido {qlims.shape}"

# converte limites de rad para graus
qlims_deg = np.rad2deg(qlims)

# estado inicial em graus (a partir de qz)
if qz is not None and len(qz) == n:
    q_init_deg = np.clip(np.rad2deg(qz), qlims_deg[0, :], qlims_deg[1, :])
else:
    q_init_deg = np.mean(qlims_deg, axis=0)

q_atual_deg = list(q_init_deg)  # estado em graus

def atualizar_junta(i, valor):
    q_atual_deg[i] = float(valor)
    q_rad = np.deg2rad(q_atual_deg)
    draw_robot(q_rad)
    update_ee_label(q_rad)

# limpa painel direito e cria sliders
for w in frame_direita.winfo_children():
    w.destroy()

tk.Label(frame_direita, text="Juntas (graus)", bg="green").pack(pady=(8, 0), anchor="w")

sliders = []
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

# ===================== Desenho inicial =====================
draw_robot(np.deg2rad(q_init_deg))

# ===================== Posição End-Effector =====================
pose_var = tk.StringVar(value="EE (m): x=—, y=—, z=—")

pose_label = tk.Label(
    frame_direita,
    textvariable=pose_var,
    bg="lightgreen",
    font=("Arial", 10, "bold"),
    justify="left"
)
pose_label.pack(padx=10, pady=(8, 4), anchor="w")

def update_ee_label(q_rad):
    T = GRASPE_ROBOT.fkine(q_rad)   # SE3
    p = T.t
    rpy_deg = T.rpy(order='xyz', unit='deg')
    pose_var.set(
     f"EE (m): x={p[0]:.3f}, y={p[1]:.3f}, z={p[2]:.3f}\n"
     f"RPY (°): roll={rpy_deg[0]:.1f}, pitch={rpy_deg[1]:.1f}, yaw={rpy_deg[2]:.1f}"
     )

update_ee_label(np.deg2rad(q_init_deg))

# ===================== Registro de Posições =====================
pos_ini_var = tk.StringVar(value="Posição inicial: —")
pos_fim_var = tk.StringVar(value="Posição final: —")

q_ini_saved = None
q_fim_saved = None
traj_saved = None  # trajetória gerada será armazenada aqui

def registrar_pos_ini():
    global q_ini_saved
    q_rad = np.deg2rad(q_atual_deg)
    T = GRASPE_ROBOT.fkine(q_rad)
    p = T.t
    q_ini_saved = q_rad
    pos_ini_var.set(f"Posição inicial:\nJuntas={np.round(q_atual_deg,1)}°\nEE=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m")

def registrar_pos_fim():
    global q_fim_saved
    q_rad = np.deg2rad(q_atual_deg)
    T = GRASPE_ROBOT.fkine(q_rad)
    p = T.t
    q_fim_saved = q_rad
    pos_fim_var.set(f"Posição final:\nJuntas={np.round(q_atual_deg,1)}°\nEE=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m")

def gerar_trajetoria():
    global q_ini_saved, q_fim_saved, traj_saved
    if q_ini_saved is None or q_fim_saved is None:
        print("⚠️ Registre posição inicial e final primeiro!")
        return
    t = np.linspace(0, 5, int(5/0.005))
    traj_saved = jtraj(q_ini_saved, q_fim_saved, t)
    print("✅ Trajetória gerada e armazenada!")

def simular_trajetoria():
    global traj_saved #trajetória gerada a partir do jtraj
    gains = np.array([
        2.42546949e+03,  8.14028079e+03,  1.22690767e+04,  1.99268404e+04,
        7.13473580e+01,  1.76258937e+02,  4.77135702e+01, 5.55385357e-04,
        9.84752097e-04, 9.38973701e-04, 8.86627296e-04,  5.87867781e+03
    ])

    cfg = SimulationCfg()
    cfg.controller = ControllerPID(gains)
    cfg.initial_q = q_ini_saved

    sim = Simulation(cfg)

    if traj_saved is None:
        print("⚠️ Gere uma trajetória primeiro!")
        return
    
    num_steps = int(5.0/cfg.dt)

    pos_q = []
    
    for i in range(num_steps):
        q_des = traj_saved.q[i]
        
        state = sim.step(q_des)
        pos_q.append(state["q"])
    
    for i, qk in enumerate(pos_q):
        if i % 5 == 0:
            draw_robot(qk)
            janela.update_idletasks()
            janela.update()

frame_registro = tk.Frame(frame_direita, bg="lightgray")
frame_registro.pack(fill="x", padx=10, pady=10)

tk.Button(frame_registro, text="Registrar Posição Inicial", command=registrar_pos_ini).pack(fill="x", pady=4)
tk.Label(frame_registro, textvariable=pos_ini_var, justify="left", bg="white", anchor="w").pack(fill="x")

tk.Button(frame_registro, text="Registrar Posição Final", command=registrar_pos_fim).pack(fill="x", pady=4)
tk.Label(frame_registro, textvariable=pos_fim_var, justify="left", bg="white", anchor="w").pack(fill="x")

tk.Button(frame_registro, text="Gerar Trajetória", command=gerar_trajetoria, bg="orange").pack(fill="x", pady=4)
tk.Button(frame_registro, text="Simular Trajetória", command=simular_trajetoria, bg="yellow").pack(fill="x", pady=4)

# ===================== Loop principal =====================
janela.mainloop()