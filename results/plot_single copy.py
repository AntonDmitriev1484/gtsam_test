import numpy as np
import matplotlib
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import os

def load_tum_trajectory(filepath):
    """Load TUM trajectory file: id, tx, ty, tz, qx, qy, qz, qw (CSV-style). Returns list of HTMs."""
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
    """Convert quaternion (x, y, z, w) to 3x3 rotation matrix."""
    x, y, z, w = q
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])
    return R


def draw_axes(ax, origin, R, length=0.2):
    """Draw coordinate axes at a given origin and orientation matrix R (3x3)."""
    x_axis = R[:, 0] * length
    y_axis = R[:, 1] * length
    z_axis = R[:, 2] * length
    ax.quiver(*origin, *x_axis, color='r', length=length, normalize=False)
    ax.quiver(*origin, *y_axis, color='g', length=length, normalize=False)
    ax.quiver(*origin, *z_axis, color='b', length=length, normalize=False)


def set_axes_equal(ax):
    """Make axes of 3D plot have equal scale."""
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d()
    ])
    spans = limits[:, 1] - limits[:, 0]
    centers = np.mean(limits, axis=1)
    radius = 0.5 * max(spans)
    ax.set_xlim3d(centers[0] - radius, centers[0] + radius)
    ax.set_ylim3d(centers[1] - radius, centers[1] + radius)
    ax.set_zlim3d(centers[2] - radius, centers[2] + radius)


def plot_trajectories(trial_dir, stride=0, show=True):
    """Plot estimated vs slam trajectories from the given trial directory."""
    est_path = os.path.join(trial_dir, "est.txt")
    slam_path = os.path.join(trial_dir, "slam.txt")

    print(trial_dir)
    est_htms = load_tum_trajectory(est_path)
    slam_htms = load_tum_trajectory(slam_path)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Extract inverted poses (origins)
    est_inv_positions = []
    slam_inv_positions = []
    for T in est_htms:
        T_inv = np.linalg.inv(T)
        est_inv_positions.append(T_inv[:3, 3])
    for T in slam_htms:
        T_inv = np.linalg.inv(T)
        slam_inv_positions.append(T_inv[:3, 3])

    est_inv_positions = np.array(est_inv_positions)
    slam_inv_positions = np.array(slam_inv_positions)

    ax.plot(est_inv_positions[:, 0], est_inv_positions[:, 1], est_inv_positions[:, 2],
            label='Estimated', color='blue', linewidth=0.5)
    ax.plot(slam_inv_positions[:, 0], slam_inv_positions[:, 1], slam_inv_positions[:, 2],
            label='SLAM Ground Truth', color='green', linewidth=0.5)

    if stride > 0:
        for i in range(0, len(est_htms), stride):
            T_inv = np.linalg.inv(est_htms[i])
            R = T_inv[:3, :3]
            t = T_inv[:3, 3]
            draw_axes(ax, t, R, length=0.3)
        for i in range(0, len(slam_htms), stride):
            T_inv = np.linalg.inv(slam_htms[i])
            R = T_inv[:3, :3]
            t = T_inv[:3, 3]
            draw_axes(ax, t, R, length=0.3)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Trajectory Comparison: {trial_dir}')
    ax.legend()
    set_axes_equal(ax)

    if show:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Compare two TUM-format trajectory files with optional orientation axes (using inverted HTM).")
    parser.add_argument("trial_dir", help="Directory containing est.txt and slam.txt (e.g. stereoi_circle2/synthetic_1_5)")
    parser.add_argument("--stride", "-s", type=int, default=0, help="Draw orientation axes every Nth point (0 = skip)")
    args = parser.parse_args()

    plot_trajectories(args.trial_dir, stride=args.stride, show=True)


if __name__ == "__main__":
    main()


# import numpy as np
# import matplotlib
# matplotlib.use('QtAgg')
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import argparse
# import os

# def load_tum_trajectory(filepath):
#     """Load TUM trajectory file: id, tx, ty, tz, qx, qy, qz, qw (CSV-style)"""
#     poses = []
#     quats = []
#     with open(filepath, 'r') as f:
#         for line in f:
#             if line.strip().startswith("#") or line.strip() == "":
#                 continue
#             parts = [p.strip() for p in line.strip().split(' ')]
#             if len(parts) != 8:
#                 print(f"Skipping malformed line: {line}")
#                 continue
#             try:
#                 _, tx, ty, tz, qx, qy, qz, qw = map(float, parts)
#                 poses.append([tx, ty, tz])
#                 quats.append([qx, qy, qz, qw])
#             except ValueError:
#                 print(f"Skipping invalid line: {line}")
#                 continue
#     return np.array(poses), np.array(quats)


# def quat_to_rotmat(q):
#     """Convert quaternion (x, y, z, w) to 3x3 rotation matrix."""
#     x, y, z, w = q
#     R = np.array([
#         [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
#         [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
#         [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
#     ])
#     return R


# def draw_axes(ax, origin, R, length=0.2):
#     """Draw coordinate axes at a given origin and orientation matrix R (3x3)."""
#     x_axis = R[:, 0] * length
#     y_axis = R[:, 1] * length
#     z_axis = R[:, 2] * length
#     ax.quiver(*origin, *x_axis, color='r', length=length, normalize=False)
#     ax.quiver(*origin, *y_axis, color='g', length=length, normalize=False)
#     ax.quiver(*origin, *z_axis, color='b', length=length, normalize=False)


# def set_axes_equal(ax):
#     """Make axes of 3D plot have equal scale"""
#     limits = np.array([
#         ax.get_xlim3d(),
#         ax.get_ylim3d(),
#         ax.get_zlim3d()
#     ])
#     spans = limits[:, 1] - limits[:, 0]
#     centers = np.mean(limits, axis=1)
#     radius = 0.5 * max(spans)
#     # ax.set_xlim3d(centers[0] - radius, centers[0] + radius)
#     # ax.set_ylim3d(centers[1] - radius, centers[1] + radius)
#     # ax.set_zlim3d(centers[2] - radius, centers[2] + radius)
#     ax.set_xlim3d(-7.5,-3.5)
#     ax.set_ylim3d(2.5,6.5)
#     ax.set_zlim3d(0,2)


# def plot_trajectories(trial_dir, stride=0, show=True):
#     """Plot estimated vs slam trajectories from the given trial directory."""
#     est_path = os.path.join(trial_dir, "est.txt")
#     slam_path = os.path.join(trial_dir, "slam.txt")

#     print(trial_dir)
#     est_poses, est_quats = load_tum_trajectory(est_path)
#     slam_poses, slam_quats = load_tum_trajectory(slam_path)

#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')

#     ax.plot(est_poses[:, 0], est_poses[:, 1], est_poses[:, 2], label='Estimated', color='blue', linewidth=0.5)
#     ax.plot(slam_poses[:, 0], slam_poses[:, 1], slam_poses[:, 2], label='SLAM Ground Truth', color='green', linewidth=0.5)

#     if stride > 0:
#         for i in range(0, len(est_poses), stride):
#             R = quat_to_rotmat(est_quats[i])
#             draw_axes(ax, est_poses[i], R, length=0.3)
#         for i in range(0, len(slam_poses), stride):
#             R = quat_to_rotmat(slam_quats[i])
#             draw_axes(ax, slam_poses[i], R, length=0.3)

#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')
#     ax.set_title(f'Trajectory Comparison: {trial_dir}')
#     ax.legend()
#     set_axes_equal(ax)

#     if show:
#         plt.show()


# def main():
#     parser = argparse.ArgumentParser(description="Compare two TUM-format trajectory files with optional orientation axes.")
#     parser.add_argument("trial_dir", help="Directory containing est.txt and slam.txt (e.g. stereoi_circle2/synthetic_1_5)")
#     parser.add_argument("--stride", "-s", type=int, default=0, help="Draw orientation axes every Nth point (0 = skip)")
#     args = parser.parse_args()

#     plot_trajectories(args.trial_dir, stride=args.stride, show=True)


# if __name__ == "__main__":
#     main()
