import tkinter as tk
import numpy as np

class SerialControlFrame(tk.LabelFrame):
    def __init__(self, parent, link, q_atual_deg):
        super().__init__(parent, text="Comunicação com ESP32", bg="lightgray", padx=10, pady=10)
        self.link = link
        self.q_atual_deg = q_atual_deg
        self.status_var = tk.StringVar(value="Desconectado ❌")

        # status
        status_frame = tk.Frame(self, bg="lightgray")
        status_frame.pack(fill="x", pady=(0, 5))
        tk.Label(status_frame, text="Status:", bg="lightgray").pack(side="left")
        tk.Label(status_frame, textvariable=self.status_var, bg="lightgray", font=("Arial", 10, "bold")).pack(side="left", padx=(5, 0))

        # botão conectar
        tk.Button(self, text="Conectar ESP32", bg="orange", command=self.conectar_esp).pack(fill="x", pady=(4, 8))

        # posição completa
        tk.Label(self, text="Posições:", bg="lightgray", font=("Arial", 10, "bold")).pack(anchor="w")
        tk.Button(self, text="Enviar posição atual (todas)", command=self.enviar_todas).pack(fill="x", pady=3)

        # juntas individuais
        for i in range(4):
            tk.Button(self, text=f"Enviar q{i+1}", command=lambda j=i: self.enviar_uma(j)).pack(fill="x", pady=2)

    # --- callbacks
    def conectar_esp(self):
        if not self.link.connect():
            self.status_var.set("Falha na conexão ❌")
            return

        self.status_var.set("Conectando...")
        if self.link.handshake():
            self.status_var.set("Conectado ✅")
        else:
            self.status_var.set("Erro no handshake ⚠️")
            self.link.disconnect()

    def enviar_todas(self):
        q_rad = np.deg2rad(self.q_atual_deg)
        msg = f"SETQ {q_rad[0]:.3f} {q_rad[1]:.3f} {q_rad[2]:.3f} {q_rad[3]:.3f}"
        self.link.send(msg)

    def enviar_uma(self, i):
        q_rad = np.deg2rad(self.q_atual_deg[i])
        msg = f"SETQ{i+1} {q_rad:.3f}"
        self.link.send(msg)
