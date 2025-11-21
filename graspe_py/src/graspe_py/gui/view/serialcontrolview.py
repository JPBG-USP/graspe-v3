import numpy as np
import tkinter as tk

class SerialControlFrame(tk.LabelFrame):
    def __init__(self, parent, q_init, link = None):
        super().__init__(parent, text="Comunicação com ESP32", bg="lightgray", padx=10, pady=10)
        
        if link is None:
            print("[WARN] No SerialLink was provided in SerialControlFrame")
        
        # state variables
        self.link = link
        self._q = q_init
        self._q_init = q_init.copy()
        self.gripper_state: bool = False
        self.motor_state: bool = True
        self.parent = parent

        # status label
        self.status_var = tk.StringVar(value="Desconectado")
        status_frame = tk.Frame(self, bg="lightgray")
        status_frame.pack(fill="x", pady=(0, 5))
        tk.Label(status_frame, text="Status:", bg="lightgray").pack(side="left")
        status_label = tk.Label(
            status_frame,
            textvariable=self.status_var,
            bg="lightgray", 
            font=("Arial", 10, "bold")
        )
        status_label.pack(side="left", padx=(5, 0))

        # connect to esp btn
        self.btn_connect = tk.Button(
            self, text="Conectar ESP32",
            bg="orange", command=self.connect_esp
        )
        self.btn_connect.pack(fill="x", pady=(4, 8))

        # send current pos btn
        tk.Label(self, text="Comandos:", bg="lightgray", font=("Arial", 10, "bold")).pack(anchor="w")
        self.btn_sendpos = tk.Button(
            self, text="Enviar posição atual",
            command=self.send_pos
        )
        self.btn_sendpos.pack(fill="x", pady=3)

        # gripper btn
        def toggle_gripper():
            self.gripper_state = not self.gripper_state
            if self.gripper_state:
                self.btn_gripper.config(text="Abrir garra")
                msg = "GRIPPERON"
            else:
                self.btn_gripper.config(text="Fechar garra")
                msg = "GRIPPEROFF"
            self.link.send(msg)

        self.btn_gripper = tk.Button(
            self, text="Fechar garra", command=toggle_gripper,
        )
        self.btn_gripper.pack(fill="x", pady=4)

        # send traj btn
        self.btn_send_traj = tk.Button(
            self, text="Enviar trajetória", command=self.send_traj,
        )
        self.btn_send_traj.pack(fill="x", pady=4)
        
        # motors btn
        def toggle_motor():
            self.motor_state = not self.motor_state
            if self.motor_state:
                self.btn_motors.config(text="Desligar motores", bg="red")
                msg = "MOTORSON"
            else:
                self.btn_motors.config(text="Ligar motores", bg="green")
                msg = "MOTORSOFF"
            self.link.send(msg)

        self.btn_motors = tk.Button(
            self, text="Desligar motores", command=toggle_motor, bg="red"
        )
        self.btn_motors.pack(side="bottom", fill="x", pady=4)

        self.btn_returndock = tk.Button(
            self, text="Retornar ao repouso",
            command=self.return_dock
        )
        self.btn_returndock.pack(fill="x", pady=3)

    def send_traj(self):
        # TODO: implement send trajectory method
        pass

    def connect_esp(self):

        if self.link is None:
            print("[ERROR] No SerialLink was provided in SerialControlFrame, impossible to handshake.")
            self.status_var.set("Nenhum link foi fornecido")
            return
        if not self.link.connect():
            self.status_var.set("Falha na conexão")
            return

        self.status_var.set("Conectando...")
        if self.link.handshake():
            self.status_var.set("Conectado")
            self.btn_connect.config(bg="darkgray")
        else:
            self.status_var.set("Erro no handshake")
            self.link.disconnect()

    def send_pos(self):

        if self.link is None:
            print("[ERROR] No SerialLink was provided in SerialControlFrame, impossible to send joint position")
            return
        if not self.link.is_connected():
            print("[ERROR] Link is not connected, impossible to send joint positions")
            return

        msg = f"SETALLQ {self._q[0]:.3f} {self._q[1]:.3f} {self._q[2]:.3f} {self._q[3]:.3f}"
        self.link.send(msg)

    def return_dock(self):

        if self.link is None:
            print("[ERROR] No SerialLink was provided in SerialControlFrame, impossible to send joint position")
            return
        if not self.link.is_connected():
            print("[ERROR] Link is not connected, impossible to send joint positions")
            return
    
        msg = f"SETALLQ {self._q_init[0]:.3f} {self._q_init[1]:.3f} {self._q_init[2]:.3f} {self._q_init[3]:.3f}"
        self.link.send(msg)

    @property
    def get_gripper_state(self):
        return self.gripper_state
    
    def get_real_robot_joint(self) -> np.ndarray:
        if self.link is None:
            return None
        if not self.link.is_connected():
            return None        
        return self.link.obter_posicao_motor()

if __name__ == "__main__":
    import tkinter as tk
    import numpy as np

    root = tk.Tk()
    root.geometry("1000x600")

    robot_view = SerialControlFrame(root, np.array([0.0, 0.0, 0.0, 0.0]))
    robot_view.pack(fill="both", expand=True)

    root.mainloop()