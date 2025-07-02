import numpy as np
import matplotlib
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse

# NOTE: Middle click to pan axes!!!!

def load_tum_trajectory(filepath):
    """Load TUM trajectory file: id, tx, ty, tz, qx, qy, qz, qw (CSV-style)"""
    poses = []
    quats = []
    with open(filepath, 'r') as f:
        for line in f:
            if line.strip().startswith("#") or line.strip() == "":
                continue
            parts = [p.strip() for p in line.strip().split(',')]
            if len(parts) != 8:
                print(f"Skipping malformed line: {line}")
                continue
            try:
                _, tx, ty, tz, qx, qy, qz, qw = map(float, parts)
                poses.append([tx, ty, tz])
                quats.append([qx, qy, qz, qw])
            except ValueError:
                print(f"Skipping invalid line: {line}")
                continue
    return np.array(poses), np.array(quats)


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
    """Make axes of 3D plot have equal scale"""
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d()
    ])
    spans = limits[:, 1] - limits[:, 0]
    centers = np.mean(limits, axis=1)
    radius = 0.5 * max(spans)
    ax.set_xlim3d(-6,-2)
    ax.set_ylim3d(1,5)
    ax.set_zlim3d(0,2)


def main():
    parser = argparse.ArgumentParser(description="Compare two TUM-format trajectory files with optional orientation axes.")
    parser.add_argument("--stride", "-s", type=int, default=0, help="Draw orientation axes every Nth point (0 = skip)")
    args = parser.parse_args()

    est_poses, est_quats = load_tum_trajectory('./estimated.txt')
    slam_poses, slam_quats = load_tum_trajectory('./slam.txt')

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(est_poses[:, 0], est_poses[:, 1], est_poses[:, 2], label='Estimated', color='blue', linewidth=0.5)
    ax.plot(slam_poses[:, 0], slam_poses[:, 1], slam_poses[:, 2], label='SLAM Ground Truth', color='green', linewidth=0.5)

    # Optionally draw axes every Nth pose
    if args.stride > 0:
        for i in range(0, len(est_poses), args.stride):
            R = quat_to_rotmat(est_quats[i])
            draw_axes(ax, est_poses[i], R, length=0.3)
        for i in range(0, len(slam_poses), args.stride):
            R = quat_to_rotmat(slam_quats[i])
            draw_axes(ax, slam_poses[i], R, length=0.3)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Trajectory Comparison')
    ax.legend()
    set_axes_equal(ax)
    plt.show()

if __name__ == "__main__":
    main()
