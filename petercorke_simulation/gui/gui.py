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
