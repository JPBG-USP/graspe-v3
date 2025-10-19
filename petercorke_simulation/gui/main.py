import tkinter as tk
import numpy as np
from .gui import criar_janela
from .visualization import inicializar_visualizacao, draw_robot
from .sliders import criar_sliders
from .positions import criar_frame_registro
from ..robot_description import GRASPE_ROBOT
from .serial_controls import SerialControlFrame
from ..comms.serial_link import SerialLink

# Restauro o valor original de qz e qlims
qz = np.array([-0.994, 1.57, 0.64, 0.75])  # Valor original de qz (sem alterar)

janela, frame_esquerda, frame_direita = criar_janela()

# cria a visualização 3D no frame esquerdo (visualização 3D para o motor 1)
fig_3d_1, ax_3d_1, canvas_3d_1 = inicializar_visualizacao(frame_esquerda)
fig_3d_2, ax_3d_2, canvas_3d_2 = inicializar_visualizacao(frame_esquerda)

# inicializa estado inicial com os valores de qz e qlims corretos
q_init_deg = np.clip(np.rad2deg(qz), qlims[0], qlims[1])  # Limita os valores de qz dentro de qlims
q_atual_deg = list(q_init_deg)

# desenha o robô nas duas visualizações 3D usando draw_robot
draw_robot(np.deg2rad(q_init_deg), ax_3d_1, canvas_3d_1)
draw_robot(np.deg2rad(q_init_deg), ax_3d_2, canvas_3d_2)

# cria sliders no frame direito
sliders = criar_sliders(frame_direita, q_init_deg, qlims, q_atual_deg, draw_robot, ax_3d_1, canvas_3d_1)

# cria o frame de registro de posições no frame direito
criar_frame_registro(frame_direita, q_atual_deg, draw_robot, janela, ax_3d_1, canvas_3d_1)

# Cria o link serial e o frame de controle no frame direito
link = SerialLink(port="/dev/ttyACM0", baudrate=115200)

serial_frame = SerialControlFrame(frame_direita, link, q_atual_deg)
serial_frame.pack(fill="x", padx=10, pady=10)

# Função para atualizar a visualização 3D com o estado dos sliders
def update_stick_view():
    q_deg = np.array(q_atual_deg)  # pegamos os valores atuais dos sliders
    draw_robot(np.deg2rad(q_deg), ax_3d_1, canvas_3d_1)  # Atualiza a 3D 1
    draw_robot(np.deg2rad(q_deg), ax_3d_2, canvas_3d_2)  # Atualiza a 3D 2

# Função para obter a posição do motor 1 e atualizar as visualizações
def ler_posicao_esp32():
    motor_position = link.obter_posicao_motor()  # Lê a posição do motor 1
    if motor_position is not None:
        q_atual_deg[0] = motor_position  # Atualiza a posição do motor 1 (junta 1)
        update_stick_view()  # Atualiza a visualização com a nova posição

# Função para ler a posição a cada 100ms
def atualizar_posicoes():
    ler_posicao_esp32()
    janela.after(100, atualizar_posicoes)  # Rechama a função após 100ms

# Começa a leitura das posições
atualizar_posicoes()

# loop principal
janela.mainloop()
