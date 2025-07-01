import numpy as np
import matplotlib
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


#NOTE: Middle click to pan axes!!!!

def load_tum_trajectory(filepath):
    """Load TUM trajectory file: id, tx, ty, tz, qx, qy, qz, qw (CSV-style)"""
    poses = []
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
            except ValueError:
                print(f"Skipping invalid line: {line}")
                continue
    return np.array(poses)


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

    # ax.set_xlim3d([centers[0] - radius, centers[0] + radius])
    # ax.set_ylim3d([centers[1] - radius, centers[1] + radius])
    # ax.set_zlim3d([centers[2] - radius, centers[2] + radius])

    ax.set_xlim3d(-6,-2)
    ax.set_ylim3d(1,5)
    ax.set_zlim3d(0,2)

def main():
    est_poses = load_tum_trajectory('./estimated.txt')
    slam_poses = load_tum_trajectory('./slam.txt')

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(est_poses[:, 0], est_poses[:, 1], est_poses[:, 2], label='Estimated', color='blue', linewidth = 0.1)
    ax.plot(slam_poses[:, 0], slam_poses[:, 1], slam_poses[:, 2], label='SLAM Ground Truth', color='green', linewidth = 0.1)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Trajectory Comparison')
    ax.legend()
    set_axes_equal(ax)
    plt.show()

if __name__ == "__main__":
    main()
