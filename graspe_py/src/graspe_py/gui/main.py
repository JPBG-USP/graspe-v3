import tkinter as tk
import numpy as np
from graspe_py.simulation import GRASPE_ROBOT, qz
from graspe_py.comms import SerialLink
from .gui import criar_janela, sim_and_real_windows
from .visualization import robot_view, draw_robot  # Usando draw_robot para 3D
from .sliders import criar_sliders
from .positions import criar_frame_registro
from .serial_controls import SerialControlFrame

# load windows and frames
window, real_robot_frame, sim_robot_frame, frame_direita = sim_and_real_windows()

# Create the canvas view of the robot
fig_3d_1, ax_3d_1, canvas_3d_1 = robot_view(real_robot_frame, title="Robo Real")
fig_3d_2, ax_3d_2, canvas_3d_2 = robot_view(sim_robot_frame, title="Robo Simulado")

# initial state
num_joints = GRASPE_ROBOT.n
qlims = np.asanyarray(GRASPE_ROBOT.qlim, dtype=float)
qlims_deg = np.rad2deg(qlims.T if qlims.shape == (num_joints, 2) else qlims)
q_init_deg = np.clip(np.rad2deg(qz), qlims_deg[0], qlims_deg[1])

# states
q_real_deg = list(q_init_deg)
q_sim_deg = list(q_init_deg)

q_atual_deg = list(q_init_deg)

# desenha o robô nas duas visualizações 3D (usando draw_robot)
draw_robot(np.deg2rad(q_init_deg), ax_3d_1, canvas_3d_1)
draw_robot(np.deg2rad(q_init_deg), ax_3d_2, canvas_3d_2)

# cria sliders no frame direito
sliders = criar_sliders(frame_direita, q_init_deg, qlims_deg, q_atual_deg, draw_robot, ax_3d_1, canvas_3d_1)

# cria o frame de registro de posições no frame direito
criar_frame_registro(frame_direita, q_atual_deg, draw_robot, window, ax_3d_2, canvas_3d_2)

# Cria o link serial e o frame de controle no frame direito
link = SerialLink(port="/dev/ttyACM0", baudrate=115200)

serial_frame = SerialControlFrame(frame_direita, link, q_atual_deg)
serial_frame.pack(fill="x", padx=10, pady=10)

# Função para atualizar a visualização 3D com o estado dos sliders
def update_stick_view():
    q_deg = np.array(q_atual_deg)  # pegamos os valores atuais dos sliders
    # Atualiza ambas as visualizações 3D com as novas juntas
    draw_robot(np.deg2rad(q_deg), ax_3d_2, canvas_3d_2)  # Atualiza a 3D 1

# Callback para atualizar o estado do robô sempre que os sliders mudam
def atualizar_slider(i, valor):
    q_atual_deg[i] = float(valor)  # Atualiza o valor da junta correspondente
    update_stick_view()  # Atualiza ambas as visualizações 3D

def update_real_robot():
    
    if link.is_connected():
        q_real = link.obter_posicao_motor()
        if  q_real is None:
            pass
        else:
            q_real_deg = np.rad2deg(q_real)
            draw_robot(np.deg2rad(q_real_deg), ax_3d_1, canvas_3d_1)

    # recall again this function after 40ms
    window.after(40, update_real_robot)

# Adicionando a função de callback para cada slider
for i, slider in enumerate(sliders):
    slider.config(command=lambda val, j=i: atualizar_slider(j, val))

# loop principal
update_real_robot()
window.mainloop()