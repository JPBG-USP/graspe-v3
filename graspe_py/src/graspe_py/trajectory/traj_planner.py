import numpy as np
from typing import List
from spatialmath import SE3
from graspe_py.simulation import GRASPE_ROBOT
from graspe_py.trajectory.joint_trajectory import smooth_jtraj
from graspe_py.trajectory.workspace_trajectory import linear_wtraj


def generate_traj(q_list: np.ndarray, gripper_list: List[bool] = None, max_joint_vel: float = 0.3, linear_vel: float = 0.02, points_hz: int = 6):

    # Finding checkpoints base on q_list
    transforms = GRASPE_ROBOT.fkine(q_list)

    # Making linear trajectory on workpace
    wtraj_list, gripper_list_out = linear_wtraj(transforms, gripper_list, int(points_hz/2), linear_vel)

    # Finding joint's pose on each mid checkpoint
    q_from_wtraj = []
    for T in wtraj_list:
        sol = GRASPE_ROBOT.ikine_LM(T, seed=42)
        # q = GRASPE_ROBOT_IK.ikine(T)
        # if q is None:
        #     print("[ERROR] Posição não alcançável pela garra.")
        #     return None, None
        q_from_wtraj.append(sol.q)
    q_from_wtraj = np.array(q_from_wtraj)

    # Smooth joint traj using 5th-order polynomial
    smothed_jtraj, gripper_list_out = smooth_jtraj(q_from_wtraj, gripper_list_out, int(points_hz/2) , max_joint_vel)

    return smothed_jtraj, wtraj_list, gripper_list_out


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import numpy as np

    q_list = np.array([
        [0.00, 1.57, 1.29, -1.19],
        [0.00, 0.72, 1.62, -0.71],
        [0.73, 0.51, 1.43, -0.32],
        [0.73, 1.29, 0.79, -1.24],
        [1.52, 1.29, 0.79, -1.24],
    ])

    jtraj_list, smooth_T, _ = generate_traj(q_list)

    plt.plot(jtraj_list)
    plt.show()


    num_joints = q_list.shape[1]
    fig, axs = plt.subplots(num_joints, 1, figsize=(8, 10), sharex=True)

    for j in range(num_joints):
        axs[j].plot(jtraj_list[:, j], color="blue")

        axs[j].set_ylabel(f"q{j+1} (rad)")
        axs[j].grid(True)

    axs[-1].set_xlabel("Samples")

    plt.tight_layout()
    plt.show()

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

    GRASPE_ROBOT.plot(jtraj_list, dt=0.1)