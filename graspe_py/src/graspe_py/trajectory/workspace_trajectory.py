import numpy as np
from typing import List
from spatialmath import SE3
from graspe_py.simulation import GRASPE_ROBOT

def linear_wtraj(transforms, points_hz: int, linear_vel: float) -> List[SE3]:
    
    wtraj_list: list = []

    for i in range(len(transforms) - 1):
        T1: SE3 = transforms[i]
        T2: SE3 = transforms[i + 1]

        dPos = np.linalg.norm(T2.t - T1.t)
        time = dPos / linear_vel
        num_points = int(time * points_hz)

        seg = T1.interp(T2, num_points)
        for k in range(len(seg)):
            wtraj_list.append(seg[k])

    return wtraj_list


if __name__ == "__main__":

    import matplotlib.pyplot as plt

    q_list = np.array([
        [0.00, 1.57, 1.29, -1.19],
        [0.00, 0.72, 1.62, -0.71],
        [0.73, 0.51, 1.43, -0.32],
        [0.73, 1.29, 0.79, -1.24],
        [1.52, 1.29, 0.79, -1.24],
    ])

    transforms = GRASPE_ROBOT.fkine(q_list)
    smooth_T = linear_wtraj(transforms, 10, 0.05)

    # Extract XYZ points
    xs = np.array([T.t[0] for T in smooth_T])
    ys = np.array([T.t[1] for T in smooth_T])
    zs = np.array([T.t[2] for T in smooth_T])

    # Original waypoints transformed into XYZ
    wp_T = GRASPE_ROBOT.fkine(q_list)
    wp_x = np.array([T.t[0] for T in wp_T])
    wp_y = np.array([T.t[1] for T in wp_T])
    wp_z = np.array([T.t[2] for T in wp_T])

    # --- PLOT ---
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")

    # Smoothed trajectory
    ax.scatter(xs, ys, zs, s=5, color="blue", label="Smoothed trajectory")

    # Waypoints from q_list
    ax.scatter(wp_x, wp_y, wp_z, s=40, color="red", marker="o", label="Original waypoints")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Manipulator End-Effector 3D Path")
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    plt.show()