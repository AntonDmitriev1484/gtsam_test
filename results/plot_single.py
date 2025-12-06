import numpy as np
import matplotlib
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import os
import json

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

def load_tum_trajectory_timestampless(filepath):
    """Load TUM trajectory file: id, tx, ty, tz, qx, qy, qz, qw (CSV-style). Returns list of HTMs."""
    htms = []
    with open(filepath, 'r') as f:
        for line in f:
            if line.strip().startswith("#") or line.strip() == "":
                continue
            parts = [p.strip() for p in line.strip().split(' ')]
            if len(parts) != 7:
                print(f"Skipping malformed line: {line}")
                continue
            try:
                tx, ty, tz, qx, qy, qz, qw = map(float, parts)
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


def draw_axes(ax, T, length=0.1):
    """Draw coordinate axes from transformation matrix T."""
    H = np.linalg.inv(T)
    origin = (H @ np.array([0,0,0,1]))[:3]
    x_axis = (H @ np.array([1,0,0,1]))[:3]
    y_axis = (H @ np.array([0,1,0,1]))[:3]
    z_axis = (H @ np.array([0,0,1,1]))[:3]

    ax.quiver(*origin, *(x_axis-origin) * length, color='r')
    ax.quiver(*origin, *(y_axis-origin) * length, color='g')
    ax.quiver(*origin, *(z_axis-origin) * length, color='b')


def plot_trajectories(trial_dir, slam_stride=0, est_stride=0, show=True, scatter=False):
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

    if scatter:
        ax.scatter(est_inv_positions[:, 0], est_inv_positions[:, 1], est_inv_positions[:, 2],
            label='Estimated', color='blue',s=0.1)
        ax.scatter(slam_inv_positions[:, 0], slam_inv_positions[:, 1], slam_inv_positions[:, 2],
                label='SLAM Ground Truth', color='green',s=0.1)
    else:
        ax.plot(est_inv_positions[:, 0], est_inv_positions[:, 1], est_inv_positions[:, 2],
                label='Estimated', color='blue', linewidth=0.5)
        ax.plot(slam_inv_positions[:, 0], slam_inv_positions[:, 1], slam_inv_positions[:, 2],
                label='SLAM Ground Truth', color='green', linewidth=0.5)

    if est_stride > 0:
        for i in range(0, len(est_htms), est_stride):
            T = est_htms[i]
            draw_axes(ax, T, length=0.3)
    if slam_stride > 0:
        for i in range(0, len(slam_htms), slam_stride):
            T = slam_htms[i]
            draw_axes(ax, T, length=0.3)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim((-2,2))
    ax.set_ylim((-2,2))
    ax.set_zlim((0,2))
    ax.set_title(f'Trajectory Comparison: {trial_dir}')
    ax.legend()

    if show:
        plt.show()

def plot_anchors(trialname):

    transforms_file = open(f"/home/antond2/ws/post/out/{trialname}_post/transforms.json", 'r')
    T = json.load(transforms_file)

    gt_anchors_file = open(f"/home/antond2/ws/post/out/{trialname}_post/gt_anchors_{trialname}.json", 'r')
    gt_anchors = {"2":[], "3":[], "4":[]} # each item will be a position in world frame
    j = json.load(gt_anchors_file)

    for id, pos in gt_anchors.items():
        j_ = [a for a in j if a["id"] == int(id)][0]
        gt_anchors[id] = j_["position"]


    est_anchors = {"2":[], "3":[], "4":[]} # each item will be a full array of poses
    for id, poses in est_anchors.items():
        est_anchors_file = f"/home/antond2/Desktop/Research/gtsam_test/results/out/{trialname}/anchor_{id}_optimization.txt"
        poses = load_tum_trajectory_timestampless(est_anchors_file)
        print(poses[0])
        pos = []
        for p in poses:
            pos.append(p[:3,3])
        est_anchors[id] = np.array(pos)


    # Now rotate gt_anchors into SLAM frame for comparison

    for id, pos in gt_anchors.items():
        pos = pos + [1] # concat 1
        pos_world = np.array(pos)
        pos_slam = T["T_world_to_slam"] @ pos_world
        gt_anchors[id] = pos_slam[:3] # cut off the 1

    for id, _ in gt_anchors.items():
        print(f"Anchor {id}")

        print(f" GT {gt_anchors[id]}")
        print(f" EST {est_anchors[id][-1]}")


    
    # parse est_anchors by dumping their est_trajectory, reading that full file
        # Ok I think I wrote this dumping est_trajectory code, run when I return
    # if we want to see the optimization, we can plot the full trajectory, otherwise, we just plot the final point in that list.


def main():
    parser = argparse.ArgumentParser(description="Compare two TUM-format trajectory files with optional orientation axes (using inverted HTM).")
    parser.add_argument("trial_dir", help="Directory containing est.txt and slam.txt (e.g. stereoi_circle2/synthetic_1_5)")
    parser.add_argument("--slam_stride", type=int, default=0, help="Draw orientation axes every Nth point (0 = skip)")
    parser.add_argument("--est_stride", type=int, default=0, help="Draw orientation axes every Nth point (0 = skip)")  
    parser.add_argument("--scatter", action="store_true", help="scatter")  
    parser.add_argument("--anchor_compare", action="store_true", help="plot ground truth and estimated anchor locations")
    args = parser.parse_args()

    directory = f"./out/{args.trial_dir}"
    plot_trajectories(directory, slam_stride=args.slam_stride, est_stride=args.est_stride, show=True, scatter=args.scatter)

    if args.anchor_compare:
        plot_anchors(args.trial_dir) # trial_dir is really just trial name


if __name__ == "__main__":
    main()
