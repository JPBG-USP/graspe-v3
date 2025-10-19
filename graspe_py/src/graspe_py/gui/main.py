import tkinter as tk
import numpy as np
from graspe_py.simulation import GRASPE_ROBOT, qz
from .gui import criar_janela
from .visualization import inicializar_visualizacao, draw_robot  # Usando draw_robot para 3D
from .sliders import criar_sliders
from .positions import criar_frame_registro
from .serial_controls import SerialControlFrame
from ..comms.serial_link import SerialLink


# cria janela e frames
janela, frame_esquerda, frame_direita = criar_janela()

# Cria a primeira visualização 3D no frame esquerdo (visualmente do mesmo tamanho)
fig_3d_1, ax_3d_1, canvas_3d_1 = inicializar_visualizacao(frame_esquerda)
fig_3d_2, ax_3d_2, canvas_3d_2 = inicializar_visualizacao(frame_esquerda)

# inicializa estado inicial
n = GRASPE_ROBOT.n
qlims = np.asarray(GRASPE_ROBOT.qlim, dtype=float)
qlims_deg = np.rad2deg(qlims.T if qlims.shape == (n, 2) else qlims)
q_init_deg = np.clip(np.rad2deg(qz), qlims_deg[0], qlims_deg[1])
q_atual_deg = list(q_init_deg)

# desenha o robô nas duas visualizações 3D (usando draw_robot)
draw_robot(np.deg2rad(q_init_deg), ax_3d_1, canvas_3d_1)
draw_robot(np.deg2rad(q_init_deg), ax_3d_2, canvas_3d_2)

# cria sliders no frame direito
sliders = criar_sliders(frame_direita, q_init_deg, qlims_deg, q_atual_deg, draw_robot, ax_3d_1, canvas_3d_1)

# cria o frame de registro de posições no frame direito
criar_frame_registro(frame_direita, q_atual_deg, draw_robot, janela, ax_3d_1, canvas_3d_1)

# Cria o link serial e o frame de controle no frame direito
link = SerialLink(port="/dev/ttyACM0", baudrate=115200)

serial_frame = SerialControlFrame(frame_direita, link, q_atual_deg)
serial_frame.pack(fill="x", padx=10, pady=10)

# Função para atualizar a visualização 3D com o estado dos sliders
def update_stick_view():
    q_deg = np.array(q_atual_deg)  # pegamos os valores atuais dos sliders
    # Atualiza ambas as visualizações 3D com as novas juntas
    draw_robot(np.deg2rad(q_deg), ax_3d_1, canvas_3d_1)  # Atualiza a 3D 1
    draw_robot(np.deg2rad(q_deg), ax_3d_2, canvas_3d_2)  # Atualiza a 3D 2

# Callback para atualizar o estado do robô sempre que os sliders mudam
def atualizar_slider(i, valor):
    q_atual_deg[i] = float(valor)  # Atualiza o valor da junta correspondente
    update_stick_view()  # Atualiza ambas as visualizações 3D

# Adicionando a função de callback para cada slider
for i, slider in enumerate(sliders):
    slider.config(command=lambda val, j=i: atualizar_slider(j, val))

# loop principal
janela.mainloop()