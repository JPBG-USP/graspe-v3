import numpy as np
import tkinter as tk
from graspe_py.gui.view.homeview import HomeView

class Application(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Grasp-e Operator Interface")
        self.geometry("1000x600")

        self.configure(
            bg="purple"
        )

        self.robot_view = HomeView(self, np.array([0.0, 0.0, 0.0, 0.0]))
        self.robot_view.pack(fill="both", expand=True)
        
if __name__ == "__main__":
    root = Application()
    root.mainloop()