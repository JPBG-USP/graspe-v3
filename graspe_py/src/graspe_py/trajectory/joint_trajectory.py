import numpy as np
from rpds import List

def compute_waypoint_velocities(q_list, max_vel):
    q_list = np.asarray(q_list)
    N = len(q_list)
    dof = q_list.shape[1]

    dq = np.zeros((N, dof))

    for i in range(N):
        if i == 0:
            # Forward difference
            dq[i] = q_list[i+1] - q_list[i]
        elif i == N - 1:
            # Backward difference
            dq[i] = q_list[i] - q_list[i-1]
        else:
            # Central difference
            dq[i] = 0.5 * (q_list[i+1] - q_list[i-1])

        # Clamp velocities to max_vel, preserving sign
        dq[i] = np.clip(dq[i], -max_vel, max_vel)

    return dq


def smooth_jtraj(q_list: np.ndarray, gripper_list, num_points: int, max_jvel: float) -> np.ndarray:
    smooth_jtraj = []
    gripper_list_out = []
    dq_list = compute_waypoint_velocities(q_list, max_jvel)

    for i in range(q_list.shape[0] - 1):
        qi = q_list[i]
        qf = q_list[i+1]
        dqi = dq_list[i]
        dqf = dq_list[i+1]

        seg, _, _ = polynomial_traj(qi, qf, dqi, dqf, num_points)

        smooth_jtraj.extend(seg[1:] if i > 0 else seg)  # Avoid duplicating points
        if gripper_list:
            gripper_list_out.extend([gripper_list[i]] * num_points)

    return np.array(smooth_jtraj), gripper_list_out


def polynomial_traj(qi, qf, dqi, dqf, num_points):
    """
    5th-order polynomial (quintic) trajectory with zero acceleration
    at start and end, parametrized by s in [0, 1].

    q(s) = qi + dqi*s + a3*s^3 + a4*s^4 + a5*s^5

    Parameters
    ----------
    qi, qf, dqi, dqf : array-like, shape (dof,)
        Joint positions and velocities (initial/final).
    num_points : int
        Number of samples (will include endpoints at s=0 and s=1).

    Returns
    -------
    q_list, dq_list, ddq_list : np.ndarray
        Arrays with shapes (num_points, dof)
    """
    qi = np.asarray(qi, dtype=float)
    qf = np.asarray(qf, dtype=float)
    dqi = np.asarray(dqi, dtype=float)
    dqf = np.asarray(dqf, dtype=float)

    dof = qi.shape[0]

    # Correct A matrix for a3, a4, a5 (constraints at s=1)
    A = np.array([
        [1.0, 1.0, 1.0],     # a3 + a4 + a5 = qf-qi-dqi
        [3.0, 4.0, 5.0],     # 3a3 + 4a4 + 5a5 = dqf-dqi
        [6.0, 12.0, 20.0]    # 6a3 + 12a4 + 20a5 = 0 (accel end = 0)
    ], dtype=float)

    # Build B with shape (3, dof)
    B = np.vstack([
        (qf - qi) - dqi,     # (dof,)
        (dqf - dqi),         # (dof,)
        np.zeros(dof)        # (dof,)
    ])

    # Solve for coefficients (rows correspond to a3,a4,a5)
    coeffs = np.linalg.solve(A, B)  # shape (3, dof)

    a3 = coeffs[0]   # shape (dof,)
    a4 = coeffs[1]
    a5 = coeffs[2]

    q_list = []
    dq_list = []
    ddq_list = []

    # sample s so that last point has s = 1.0
    for i in range(num_points):
        s = i / (num_points - 1)

        q = qi + dqi * s + a3 * s**3 + a4 * s**4 + a5 * s**5
        dq = dqi + 3 * a3 * s**2 + 4 * a4 * s**3 + 5 * a5 * s**4
        ddq = 6 * a3 * s + 12 * a4 * s**2 + 20 * a5 * s**3

        q_list.append(q)
        dq_list.append(dq)
        ddq_list.append(ddq)

    return np.array(q_list), np.array(dq_list), np.array(ddq_list)


if __name__ == "__main__":
    import matplotlib.pyplot as plt


    # 5th-order polynomial (quintic) trajectory
    qi = np.array([0.0, 0.0, 0.0, 0.0])
    dqi = np.array([0.0, 0.0, 0.0, 0.0])
    qf = np.array([0.5, 1.0, -0.5, 0.3])
    dqf = np.array([0.0, 0.0, 0.0, 0.0])

    q_list, dq_list, ddq_list = polynomial_traj(qi, qf, dqi, dqf, 200)

    fig, axes = plt.subplots(3, 1, figsize=(8, 10), sharex=True)

    # plot each joint separately with labels
    for j in range(q_list.shape[1]):
        axes[0].plot(q_list[:, j], label=f"q{j+1}")
    axes[0].set_ylabel("q(t)")
    axes[0].legend()
    axes[0].grid(True)

    for j in range(dq_list.shape[1]):
        axes[1].plot(dq_list[:, j], label=f"dq{j+1}")
    axes[1].set_ylabel("dq(t)")
    axes[1].legend()
    axes[1].grid(True)

    for j in range(ddq_list.shape[1]):
        axes[2].plot(ddq_list[:, j], label=f"ddq{j+1}")
    axes[2].set_ylabel("ddq(t)")
    axes[2].set_xlabel("Samples")
    axes[2].legend()
    axes[2].grid(True)

    plt.tight_layout()
    plt.show()

    # SMOOTH JOINT TRAJECTORY
    q_list = np.array([
        [0.00, 1.57, 1.29, -1.19],
        [0.00, 0.72, 1.62, -0.71],
        [0.73, 0.51, 1.43, -0.32],
        [0.73, 1.29, 0.79, -1.24],
        [1.52, 1.29, 0.79, -1.24],
    ])

    jtraj_list, _ = smooth_jtraj(q_list, None, 100, 1.0)
    num_joints = q_list.shape[1]
    fig, axs = plt.subplots(num_joints, 1, figsize=(8, 10), sharex=True)

    for j in range(num_joints):
        axs[j].plot(jtraj_list[:, j], color="blue")

        axs[j].set_ylabel(f"q{j+1} (rad)")
        axs[j].grid(True)

    axs[-1].set_xlabel("Samples")

    plt.tight_layout()
    plt.show()
