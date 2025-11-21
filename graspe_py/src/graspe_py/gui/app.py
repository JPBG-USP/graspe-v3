import numpy as np
import tkinter as tk
from graspe_py.gui.view.homeview import HomeView
from graspe_py.comms.serial_link import SerialLink

class Application(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Grasp-e Operator Interface")
        self.geometry("1000x600")

        self.configure(
            bg="purple"
        )

        self.link = SerialLink(port="/dev/ttyACM0", baudrate=115200)

        self.robot_view = HomeView(self, np.array([0.0, 0.0, 0.0, 0.0]), self.link)
        self.robot_view.pack(fill="both", expand=True)
        
if __name__ == "__main__":
    root = Application()
    root.mainloop()