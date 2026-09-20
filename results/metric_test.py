"""
Focused metric test: compare the "no_uwb" and "uwb" estimated trajectories of a
single trial against Optitrack ground truth.

Behaves like eval.py --no_run: nothing is re-run, the trajectories already
written under results/out are read straight off disk.

    python3 metric_test.py <id> <trial_name>
"""

import argparse
import json
import os

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the 3d projection)

from evo.tools import file_interface
from evo.core import metrics
from evo.core import sync


RUN_CONFIGS = [("no_uwb", "IMU"), ("uwb", "Flock")]

# Matching colors between the 3D plot and the per-config jitter windows
CONFIG_COLORS = {"no_uwb": "tab:orange", "uwb": "tab:blue"}

JITTER_METRICS = ["Jitter", "Jitter Estimate-only", "Jitter Estimate-displacement"]


def angle_between(v1, v2):
    cos_theta = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    return np.degrees(np.arccos(cos_theta))


def array_stats(arr):
    """Same summary statistics evo reports, for a plain array."""
    arr = np.asarray(arr, dtype=float)
    if arr.size == 0:
        return {k: float("nan") for k in ["mean", "median", "min", "max", "std", "rmse"]}
    return {
        "mean": float(np.nanmean(arr)),
        "median": float(np.nanmedian(arr)),
        "min": float(np.nanmin(arr)),
        "max": float(np.nanmax(arr)),
        "std": float(np.nanstd(arr)),
        "rmse": float(np.sqrt(np.nanmean(arr ** 2))),
    }


def compute_jitter(traj_ref_sync, traj_est_sync):
    """
    Translational jitter over a 3-pose sliding window, identical to eval.py.

    Returns the three jitter series plus the timestamp of each window's centre
    pose, so they can be plotted against time.
    """
    jitter = []                    # est jitter with the ground-truth jitter subtracted
    jitter_est = []                # non-normalized
    jitter_est_displacement = []   # non-normalized and no angle scaling
    timestamps = []

    ref_pos = traj_ref_sync.positions_xyz
    est_pos = traj_est_sync.positions_xyz

    for i in range(len(est_pos) - 2):
        # Compute on est
        window = est_pos[i:i + 3]
        d1 = window[1] - window[0]
        d2 = window[2] - window[1]  # displacement of jitter
        theta = 180 - angle_between(d1, d2)
        est_displacement = np.linalg.norm(d1) + np.linalg.norm(d2)
        est_angle = (theta / 360)

        # Compute on ref
        window = ref_pos[i:i + 3]
        d1 = window[1] - window[0]
        d2 = window[2] - window[1]  # displacement of jitter
        theta = 180 - angle_between(d1, d2)
        ref_displacement = np.linalg.norm(d1) + np.linalg.norm(d2)
        ref_angle = (theta / 360)

        jitter.append((est_displacement * est_angle) - (ref_displacement * ref_angle))
        jitter_est.append(est_displacement * est_angle)
        jitter_est_displacement.append(est_displacement)
        timestamps.append(traj_est_sync.timestamps[i + 1])

    return (np.array(jitter), np.array(jitter_est),
            np.array(jitter_est_displacement), np.array(timestamps))


def compute_metrics(traj_ref_sync, traj_est_sync):
    """APE (translation + rotation) and the three jitter series."""
    report = {}

    ape_trans = metrics.APE(metrics.PoseRelation.translation_part)
    ape_trans.process_data((traj_ref_sync, traj_est_sync))
    report["APE Translation (m)"] = ape_trans.get_all_statistics()

    ape_rot = metrics.APE(metrics.PoseRelation.rotation_angle_deg)
    ape_rot.process_data((traj_ref_sync, traj_est_sync))
    report["APE Rotation (deg)"] = ape_rot.get_all_statistics()

    jitter, jitter_est, jitter_est_disp, jitter_ts = compute_jitter(
        traj_ref_sync, traj_est_sync)

    series = dict(zip(JITTER_METRICS, [jitter, jitter_est, jitter_est_disp]))
    for name, arr in series.items():
        report[name] = array_stats(arr)

    return report, series, jitter_ts


def format_stats(report):
    """Summary statistics as a monospace block, for the console and the figure."""
    keys = ["mean", "median", "min", "max", "std", "rmse"]
    width = max(len(n) for n in report) + 2

    lines = [" " * width + "".join(f"{k:>11}" for k in keys)]
    for name, stats in report.items():
        row = "".join(f"{stats.get(k, float('nan')):>11.4f}" for k in keys)
        lines.append(f"{name:<{width}}{row}")
    return "\n".join(lines)


def resolve_paths(args):
    if 'multi' in args.trial_name:
        results_path = (f"/home/antond2/Desktop/Research/gtsam_test/results/out/multi/"
                        f"{args.id}/{args.trial_name}")
    else:
        results_path = (f"/home/antond2/Desktop/Research/gtsam_test/results/out/"
                        f"{args.trial_name}")

    post_path = (f"/home/antond2/Desktop/Research/MultiXR-Post/{args.id}/post/"
                 f"{args.trial_name}_post/")

    return results_path, post_path


def plot_trajectories_3d(trajectories, gt_traj, args):
    """One interactive 3D window with ground truth and both estimates."""
    fig = plt.figure(f"Trajectories - {args.trial_name} (id {args.id})", figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    s = args.stride
    all_xyz = []

    if gt_traj is not None:
        xyz = gt_traj.positions_xyz[::s]
        all_xyz.append(xyz)
        ax.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2],
                color="0.4", linestyle="--", linewidth=1.0, label="Optitrack (GT)")

    for run_config, name in RUN_CONFIGS:
        entry = trajectories.get(run_config)
        if entry is None:
            continue
        xyz = entry["traj"].positions_xyz[::s]
        all_xyz.append(xyz)
        # alpha so the two estimates stay distinguishable where they overlap
        ax.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2],
                color=CONFIG_COLORS[run_config], linewidth=1.0, alpha=0.75,
                label=f"est_{run_config} ({name})")

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_title(f"{args.trial_name} - nuc{args.id}")
    ax.legend()

    # Equal aspect so the trajectory shape is not distorted
    if all_xyz:
        pts = np.vstack(all_xyz)
        centre = (pts.max(axis=0) + pts.min(axis=0)) / 2
        span = (pts.max(axis=0) - pts.min(axis=0)).max() / 2
        span = span if span > 0 else 1.0
        ax.set_xlim(centre[0] - span, centre[0] + span)
        ax.set_ylim(centre[1] - span, centre[1] + span)
        ax.set_zlim(centre[2] - span, centre[2] + span)
        ax.set_box_aspect((1, 1, 1))

    return fig


def plot_jitter_window(run_config, name, entry, args):
    """One window per run config: every jitter metric over time on shared axes."""
    fig, ax = plt.subplots(
        num=f"Jitter - est_{run_config} ({name})", figsize=(13, 7))

    t0 = entry["jitter_ts"][0] if len(entry["jitter_ts"]) else 0.0
    t = entry["jitter_ts"] - t0

    for metric_name, arr in entry["series"].items():
        ax.plot(t, arr, linewidth=0.8, label=metric_name)

    ax.set_xlabel("time since trajectory start (s)")
    ax.set_ylabel("jitter (m)")
    ax.set_title(f"{args.trial_name} - nuc{args.id} - est_{run_config} ({name})")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    # Summary statistics, the same ones eval.py dumps, on the plot itself
    ax.text(
        0.5, -0.18, format_stats(entry["report"]),
        transform=ax.transAxes,
        ha="center", va="top",
        family="monospace", fontsize=8,
        bbox=dict(boxstyle="round", facecolor="0.95", edgecolor="0.7"),
    )
    fig.subplots_adjust(bottom=0.34)

    return fig


def run_metric_test(args):
    results_path, post_path = resolve_paths(args)
    opti_path = f"{post_path}/opti.txt"

    try:
        gt_traj = file_interface.read_tum_trajectory_file(opti_path)
    except Exception as e:
        print(f"Could not read ground-truth trajectory {opti_path}: {e}")
        return None

    if len(gt_traj.timestamps) == 0:
        print(f"Empty ground-truth trajectory: {opti_path}")
        return None

    trajectories = {}

    for run_config, name in RUN_CONFIGS:
        est_path = f"{results_path}/est_{run_config}.txt"

        if not os.path.exists(est_path):
            print(f"Missing estimated trajectory: {est_path}")
            continue

        try:
            est_traj = file_interface.read_tum_trajectory_file(est_path)
        except Exception as e:
            print(f"Could not read {est_path}: {e}")
            continue

        if len(est_traj.timestamps) == 0:
            print(f"Empty estimated trajectory: {est_path}")
            continue

        traj_ref_sync, traj_est_sync = sync.associate_trajectories(
            gt_traj,
            est_traj,
            max_diff=0.05
        )

        try:
            report, series, jitter_ts = compute_metrics(traj_ref_sync, traj_est_sync)
        except Exception as e:
            print(f"Could not compute metrics for est_{run_config}: {e}")
            continue

        trajectories[run_config] = {
            "name": name,
            "traj": traj_est_sync,
            "ref": traj_ref_sync,
            "report": report,
            "series": series,
            "jitter_ts": jitter_ts,
        }

        print()
        print("----------------------------------")
        print(f"est_{run_config} ({name}) - {len(traj_est_sync.timestamps)} synced poses")
        print("----------------------------------")
        print(format_stats(report))
        print()
        print(json.dumps(report, indent=1))

    if not trajectories:
        print("No trajectories could be evaluated.")
        return None

    if not args.no_plot:
        plot_trajectories_3d(trajectories, gt_traj, args)
        for run_config, name in RUN_CONFIGS:
            entry = trajectories.get(run_config)
            if entry is not None and len(entry["jitter_ts"]) > 0:
                plot_jitter_window(run_config, name, entry, args)

        if not args.hide_plots:
            plt.show()

    return trajectories


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="Compare est_no_uwb and est_uwb against Optitrack: APE + jitter.")
    parser.add_argument("id", type=int)
    parser.add_argument("trial_name", help="Trial name")
    parser.add_argument("--stride", type=int, default=1,
                        help="Stride for the 3D trajectory plot (matplotlib 3D is slow "
                             "with very dense trajectories)")
    parser.add_argument("--hide_plots", action="store_true",
                        help="Build the figures but do not call plt.show()")
    parser.add_argument("--no_plot", action="store_true",
                        help="Console summary statistics only")
    args = parser.parse_args()

    run_metric_test(args)
