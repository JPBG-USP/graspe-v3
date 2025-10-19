import tkinter as tk

def criar_janela():
    janela = tk.Tk()
    janela.title("Simulação Tkinter")
    janela.geometry("1000x600")
    janela.configure(bg="purple")

    paned = tk.PanedWindow(janela, orient=tk.HORIZONTAL, sashrelief="raised", bg="purple")
    paned.pack(fill="both", expand=True)

    # Frame esquerdo (onde vamos adicionar 2 visualizações)
    frame_esquerda = tk.Frame(paned, bg="purple", width=700, height=600)
    # Frame direito (onde ficam os sliders e controles)
    frame_direita = tk.Frame(paned, bg="green", width=300, height=600)

    paned.add(frame_esquerda)
    paned.add(frame_direita)

    return janela, frame_esquerda, frame_direita

def sim_and_real_windows():
    
    # Config main window
    window = tk.Tk()
    window.title("Grasp-E Interface")
    window.geometry("1000x600")
    window.configure(
        bg="purple",
    )

    paned = tk.PanedWindow(window, 
        orient=tk.HORIZONTAL,
        sashrelief="raised", 
        bg="purple"
    )
    paned.pack(fill="both", expand=True)

    robot_paned = tk.PanedWindow(
        orient=tk.VERTICAL,
        sashrelief="raised",
        background="purple"
    )
    robot_paned.pack(fill="both", expand=True)

    sim_robot_frame = tk.Frame(robot_paned,
        bg="purple",
        width=700,
        height=300
    )

    real_robot_frame = tk.Frame(robot_paned,
        bg="purple",
        width=700,
        height=300
    )

    robot_paned.add(sim_robot_frame)
    robot_paned.add(real_robot_frame)

    right_frame = tk.Frame(paned, 
        bg="green",
        width=300,
        height=600
    )

    paned.add(robot_paned)
    paned.add(right_frame)
    
    return window, real_robot_frame, sim_robot_frame, right_frame