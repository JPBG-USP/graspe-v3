import numpy as np
from roboticstoolbox import Link, ET, ERobot

# Dimentions in meters
l1 = 0.1672
l2 = 0.1026
l3 = 0.1026
l4 = 0.05

# Mass in kg
m0 = 1.845725     # Base
m1 = 0.097279     # Link 1
m2 = 0.097279     # Link 2
m3 = 0.097279     # Link 3
m_grip = 0.094064 # Gripper

# Centers of mass in meters
r0 = [ 0.001216, -1.795e-09, 0.035877]  # Base
r1 = [-0.000006,  0.004577,  0.136874]  # Link 1
r2 = [-0.000006,  0.004677,  0.229368]  # Link 2
r3 = [-0.000006,  0.004777,  0.321862]  # Link 3
r_grip = [0.007842, 0.004905, 0.384644] # Gripper

# Diagonal inertia matrices in kg·m²
# (converted from g·mm² by multiplying by 1e-9)
I0 = np.diag([3.758e6, 3.885e6, 3.689e6]) * 1e-9            # Base
I1 = np.diag([68412.39, 41832.092, 38052.111]) * 1e-9       # Link 1
I2 = np.diag([68412.39, 41832.092, 38052.111]) * 1e-9       # Link 2
I3 = np.diag([68412.39, 41832.092, 38052.111]) * 1e-9       # Link 3
I_grip = np.diag([49916.944, 38851.159, 30025.728]) * 1e-9  # Gripper

# Friction/motor parameters
Tc = 0
Jm = 0
B  = 0

# Define robot links
link0 = Link(
    ET.Rz(qlim=[-np.pi/2, np.pi/2]),
    r=r0, m=m0, I=I0, Tc=Tc, Jm=Jm, B=B
)

link1 = Link(
    ET.tz(l1) * ET.Ry(qlim=(-0.57, np.pi/2)),
    r=r1, m=m1, I=I1, Tc=Tc, Jm=Jm, B=B
)

link2 = Link(
    ET.tz(l2) * ET.Ry(qlim=(-2.3, 0)),
    r=r2, m=m2, I=I2, Tc=Tc, Jm=Jm, B=B
)

link3 = Link(
    ET.tz(l3) * ET.Ry(qlim=(-np.pi/2, np.pi/2)),
    r=r3, m=m3, I=I3, Tc=Tc, Jm=Jm, B=B
)

link4 = Link(
    ET.tz(l4),
    r=r_grip, m=m_grip, I=I_grip, Tc=Tc, Jm=Jm, B=B
)

# Create the robot model
GRASPE_ROBOT = ERobot([link0, link1, link2, link3, link4], name="Graspe")

# Test the robot model
if __name__ == "__main__":
    print("=== Parâmetros do robô Graspe ===")
    print(GRASPE_ROBOT.dynamics())
    print("===graspe=====")
    print(GRASPE_ROBOT)