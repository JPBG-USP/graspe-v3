import tkinter as tk
import math

class StickView(tk.Frame):
    """
    Frame que desenha um 'stick robot' (palito) com base em x, y, theta.
    Coordinates: assumimos x, y em mm; será feito scaling automático.
    """
    def __init__(self, master, width=400, height=400, mm_per_pixel=5.0, *args, **kwargs):
        super().__init__(master, *args, **kwargs)
        self.width = width
        self.height = height
        self.mm_per_pixel = mm_per_pixel
        self.canvas = tk.Canvas(self, width=self.width, height=self.height, bg='white')
        self.canvas.pack(fill='both', expand=True)
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.theta_deg = 0.0
        self.scale = 1.0
        self.offset_x = self.width / 2
        self.offset_y = self.height / 2

    def mm_to_px(self, x_mm, y_mm):
        # Converte as coordenadas em mm (com origem no centro) para px no canvas
        px = self.offset_x + (x_mm / self.mm_per_pixel) * self.scale
        py = self.offset_y - (y_mm / self.mm_per_pixel) * self.scale
        return px, py

    def update_state(self, x_mm, y_mm, theta_deg):
        # Atualiza o estado interno e redesenha o stick robot
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.theta_deg = theta_deg
        self._draw_stick()

    def _draw_stick(self):
        # Parâmetros do corpo em mm
        head_r_mm = 40
        body_len_mm = 80
        leg_len_mm = 50
        arm_len_mm = 45

        cx, cy = self.mm_to_px(self.x_mm, self.y_mm)
        theta = math.radians(self.theta_deg)

        head_r = head_r_mm / self.mm_per_pixel * self.scale
        body_end_x = cx + math.cos(theta) * (body_len_mm / self.mm_per_pixel * self.scale)
        body_end_y = cy - math.sin(theta) * (body_len_mm / self.mm_per_pixel * self.scale)

        # Desenhando as pernas e os braços
        left_leg_x = body_end_x + math.cos(theta + math.pi/2) * 12
        left_leg_y = body_end_y - math.sin(theta + math.pi/2) * 12
        right_leg_x = body_end_x + math.cos(theta - math.pi/2) * 12
        right_leg_y = body_end_y - math.sin(theta - math.pi/2) * 12

        # Desenhando a cabeça, corpo e pernas
        self.canvas.delete("all")
        self.canvas.create_oval(cx - head_r, cy - head_r, cx + head_r, cy + head_r, outline='black', width=2)
        self.canvas.create_line(cx, cy, body_end_x, body_end_y, width=3)
        self.canvas.create_line(left_leg_x, left_leg_y, left_leg_x + math.cos(theta) * leg_len_mm, left_leg_y - math.sin(theta) * leg_len_mm, width=2)
        self.canvas.create_line(right_leg_x, right_leg_y, right_leg_x + math.cos(theta) * leg_len_mm, right_leg_y - math.sin(theta) * leg_len_mm, width=2)

        # Desenha orientação
        arrow_len = 30
        ax = cx + math.cos(theta) * (head_r + 6)
        ay = cy - math.sin(theta) * (head_r + 6)
        fx = ax + math.cos(theta) * arrow_len
        fy = ay - math.sin(theta) * arrow_len
        self.canvas.create_line(ax, ay, fx, fy, arrow=tk.LAST, width=2)

        # Exibe coordenadas e orientação
        self.canvas.create_text(8, 8, anchor='nw', text=f"x={self.x_mm:.1f} mm\ny={self.y_mm:.1f} mm\nθ={self.theta_deg:.1f}°", font=("TkDefaultFont", 8))
