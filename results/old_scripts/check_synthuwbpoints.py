
import numpy as np
import matplotlib
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import json

# === Hardcoded path to JSON file ===
JSON_PATH = "/home/antond2/ws/post/out/stereoi_circle2_post/synthetic/all_synthetic_1_5.json"  # <<< HARD-CODE YOUR JSON PATH HERE

def draw_axes(ax, origin, R, length=0.2):
    x_axis = R[:, 0] * length
    y_axis = R[:, 1] * length
    z_axis = R[:, 2] * length
    ax.quiver(*origin, *x_axis, color='r', length=length, normalize=False)
    ax.quiver(*origin, *y_axis, color='g', length=length, normalize=False)
    ax.quiver(*origin, *z_axis, color='b', length=length, normalize=False)

def set_axes_equal(ax):
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

def plot_json_trajectories(stride=0):
    with open(JSON_PATH, 'r') as f:
        data = json.load(f)

    slam_positions = []
    slam_rotations = []

    uwb_positions = []
    uwb_rotations = []

    for entry in data:
        if "type" not in entry or "T_body_slam" not in entry:
            continue
        pose = np.array(entry["T_body_slam"])
        R = np.array(pose[:3, :3])
        t = np.array(pose[:3, 3])

        if entry["type"] == "slam_pose":
            slam_positions.append(t)
            slam_rotations.append(R)
        elif entry["type"] == "synthetic_uwb":
            uwb_positions.append(t)
            uwb_rotations.append(R)

    slam_positions = np.array(slam_positions)
    uwb_positions = np.array(uwb_positions)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(slam_positions[:, 0], slam_positions[:, 1], slam_positions[:, 2], c='green', label='slam_pose', s=5)
    ax.scatter(uwb_positions[:, 0], uwb_positions[:, 1], uwb_positions[:, 2], c='red', label='synth_uwb', s=5)

    if stride > 0:
        for i in range(0, len(slam_positions), stride):
            draw_axes(ax, slam_positions[i], slam_rotations[i], length=0.2)
        for i in range(0, len(uwb_positions), stride):
            draw_axes(ax, uwb_positions[i], uwb_rotations[i], length=0.2)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    set_axes_equal(ax)
    plt.title("slam_pose (green) vs synthetic_uwb (red)")
    plt.show()

def main():
    parser = argparse.ArgumentParser(description="Plot slam_pose and synth_uwb from JSON")
    parser.add_argument("--stride", "-s", type=int, default=0, help="Draw axes every Nth point (0 = none)")
    args = parser.parse_args()

    plot_json_trajectories(stride=args.stride)

if __name__ == "__main__":
    main()
