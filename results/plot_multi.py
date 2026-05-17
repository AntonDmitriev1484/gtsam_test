import numpy as np
import matplotlib
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import os

USERS = [2, 3, 4]


def load_tum_trajectory(filepath):
    """Load TUM trajectory file into HTMs."""
    htms = []

    with open(filepath, 'r') as f:
        for line in f:
            if line.strip().startswith("#") or line.strip() == "":
                continue

            parts = [p.strip() for p in line.strip().split(' ')]

            if len(parts) != 8:
                print(f"Skipping malformed line: {line}")
                continue

            try:
                _, tx, ty, tz, qx, qy, qz, qw = map(float, parts)

                R = quat_to_rotmat([qx, qy, qz, qw])

                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = [tx, ty, tz]

                htms.append(T)

            except ValueError:
                print(f"Skipping invalid line: {line}")
                continue

    return np.array(htms)


def quat_to_rotmat(q):
    """Quaternion (x,y,z,w) -> rotation matrix."""
    x, y, z, w = q

    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])


def draw_axes(ax, T, length=0.1):
    """Draw coordinate frame axes from HTM."""
    H = np.linalg.inv(T)

    origin = (H @ np.array([0, 0, 0, 1]))[:3]
    x_axis = (H @ np.array([1, 0, 0, 1]))[:3]
    y_axis = (H @ np.array([0, 1, 0, 1]))[:3]
    z_axis = (H @ np.array([0, 0, 1, 1]))[:3]

    ax.quiver(*origin, *(x_axis-origin) * length, color='r')
    ax.quiver(*origin, *(y_axis-origin) * length, color='g')
    ax.quiver(*origin, *(z_axis-origin) * length, color='b')


def extract_inverse_positions(htms):
    """Extract positions from inverse HTMs."""
    positions = []

    for T in htms:
        T_inv = np.linalg.inv(T)
        positions.append(T_inv[:3, 3])

    return np.array(positions)


def plot_trajectories(base_dir,
                      slam_stride=0,
                      est_stride=0,
                      show=True,
                      scatter=False):

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    colors = {
        2: "blue",
        3: "red",
        4: "purple"
    }

    for user in USERS:

        trial_dir = os.path.join(base_dir, str(user))

        est_path = os.path.join(trial_dir, "est.txt")
        slam_path = os.path.join(trial_dir, "slam.txt")

        print(f"\nUser {user}")
        print(f"EST : {est_path}")
        print(f"SLAM: {slam_path}")

        est_htms = load_tum_trajectory(est_path)
        slam_htms = load_tum_trajectory(slam_path)

        est_positions = extract_inverse_positions(est_htms)
        slam_positions = extract_inverse_positions(slam_htms)

        color = colors[user]

        if scatter:
            ax.scatter(
                est_positions[:, 0],
                est_positions[:, 1],
                est_positions[:, 2],
                color=color,
                s=0.5,
                label=f"User {user} Estimated"
            )

            ax.scatter(
                slam_positions[:, 0],
                slam_positions[:, 1],
                slam_positions[:, 2],
                color=color,
                s=0.5,
                alpha=0.3,
                label=f"User {user} SLAM"
            )

        else:
            ax.plot(
                est_positions[:, 0],
                est_positions[:, 1],
                est_positions[:, 2],
                linewidth=1.0,
                color=color,
                label=f"User {user} Estimated"
            )

            ax.plot(
                slam_positions[:, 0],
                slam_positions[:, 1],
                slam_positions[:, 2],
                linestyle='--',
                linewidth=1.0,
                color=color,
                alpha=0.5,
                label=f"User {user} SLAM"
            )

        # Start marker
        ax.scatter(
            *est_positions[0],
            color=color,
            marker='o',
            s=60
        )

        # End marker
        ax.scatter(
            *est_positions[-1],
            color=color,
            marker='x',
            s=80
        )

        ax.text(
            *est_positions[0],
            f"{user} start",
            fontsize=8
        )

        ax.text(
            *est_positions[-1],
            f"{user} end",
            fontsize=8
        )

        # Orientation axes
        if est_stride > 0:
            for i in range(0, len(est_htms), est_stride):
                draw_axes(ax, est_htms[i], length=0.3)

        if slam_stride > 0:
            for i in range(0, len(slam_htms), slam_stride):
                draw_axes(ax, slam_htms[i], length=0.3)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    ax.set_xlim((-2, 2))
    ax.set_ylim((-2, 2))
    ax.set_zlim((0, 2))

    ax.set_title(f"Trajectory Comparison: {base_dir}")

    ax.legend()

    if show:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Plot multi-user estimated/slam trajectories."
    )

    parser.add_argument(
        "trial_name",
        help="Example: opti_multi1_passing"
    )

    parser.add_argument(
        "--slam_stride",
        type=int,
        default=0,
        help="Draw SLAM axes every Nth point"
    )

    parser.add_argument(
        "--est_stride",
        type=int,
        default=0,
        help="Draw estimated axes every Nth point"
    )

    parser.add_argument(
        "--scatter",
        action="store_true",
        help="Use scatter plot instead of line plot"
    )

    args = parser.parse_args()

    base_dir = f"/home/antond2/Desktop/Research/gtsam_test/results/out/integration_tests/{args.trial_name}"

    plot_trajectories(
        base_dir,
        slam_stride=args.slam_stride,
        est_stride=args.est_stride,
        show=True,
        scatter=args.scatter
    )


if __name__ == "__main__":
    main()