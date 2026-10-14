import re
import argparse
import matplotlib.pyplot as plt
import os
import subprocess
import json
import numpy as np

from evo.tools import file_interface
from evo.tools import plot as evo_plot
from evo.core import metrics
from evo.core import sync
from evo.core.trajectory import PoseTrajectory3D
from scipy.signal import savgol_filter
from plot_runtimes import plot_isam_runtimes

import sys
sys.path.append("/home/antond2/Desktop/Research/MultiXR-Post/")
sys.path.append("/home/antond2/Desktop/Research/Cappella/")
from plot_all import plot_trial_paper
from convert_to_cappella import convert_to_cappella
from run_cappella import run_cappella
from types import SimpleNamespace

import copy


def read_inverted_tum_trajectory_file(path):
    """
    Read a TUM trajectory and invert every pose, keeping evo's format.

    The .txt files store T_body_world (write_trajectory_TUM_format applies
    .inverse() before writing, and post_process.py builds opti.txt the same
    way), so the raw translation is the world origin in the body frame rather
    than the body position in the world. Inverting recovers T_world_body,
    which is what plot_all plots and what APE/jitter should be computed on.
    """
    traj = file_interface.read_tum_trajectory_file(path)

    return PoseTrajectory3D(
        poses_se3=[np.linalg.inv(pose) for pose in traj.poses_se3],
        timestamps=traj.timestamps
    )

def percentiles(errors):
    """p95/p99 of an evo metric's error array, which get_all_statistics omits."""
    errors = np.asarray(errors, dtype=float)
    errors = errors[np.isfinite(errors)]
    if errors.size == 0:
        return {"p95": float("nan"), "p99": float("nan")}
    return {
        "p95": float(np.percentile(errors, 95)),
        "p99": float(np.percentile(errors, 99)),
    }


# Jerk: uniform grid the positions are resampled onto before differentiating,
# and the Savitzky-Golay differentiator applied on that grid. Same settings as
# /home/antond2/Desktop/Research/jitter_metric/test.py
RESAMPLE_HZ = 100.0
SG_WINDOW_S = 0.25
SG_POLYORDER = 5


def interp_positions(traj, t):
    """Linearly interpolate a trajectory's positions onto timestamps t."""
    ts, idx = np.unique(traj.timestamps, return_index=True)
    xyz = traj.positions_xyz[idx]
    return np.column_stack([np.interp(t, ts, xyz[:, k]) for k in range(3)])


def compute_jerk(traj_ref_sync, traj_est_sync):
    """
    Jerk error between the estimate and the ground truth, as |jerk_est - jerk_gt|.

    Both trajectories are resampled onto a common uniform grid and
    differentiated three times with a Savitzky-Golay filter, the same way
    jitter_metric/test.py does it. The returned array is the normalized jerk:
    the per-sample magnitude of the difference between the two jerk vectors.

    The filter fits a polynomial over a moving window, so the first and last
    half-window of samples are extrapolated rather than fitted. test.py avoids
    those by keeping a margin of real data either side of its evaluation
    window; here the trajectories arrive already synced (and, for failure
    intervals, already cropped), so there is no margin to draw on and the
    affected samples are trimmed off instead.

    Returns an empty array if the segment is too short to differentiate.
    """
    dt = 1.0 / RESAMPLE_HZ

    lo = max(traj_ref_sync.timestamps[0], traj_est_sync.timestamps[0])
    hi = min(traj_ref_sync.timestamps[-1], traj_est_sync.timestamps[-1])
    t = np.arange(lo, hi, dt)

    sg_len = int(round(SG_WINDOW_S / dt)) | 1   # must be odd
    half = sg_len // 2

    # Needs a full filter window, plus something left over once both edges go
    if len(t) <= sg_len + 2 * half:
        return np.array([])

    p_est = interp_positions(traj_est_sync, t)
    p_ref = interp_positions(traj_ref_sync, t)

    def jerk(p):
        return savgol_filter(p, sg_len, SG_POLYORDER, deriv=3, delta=dt, axis=0)

    jerk_err = np.linalg.norm(jerk(p_est) - jerk(p_ref), axis=1)

    return jerk_err[half:-half]


def compute_jitter(traj_ref_sync, traj_est_sync):
    """
    Translational jitter over a 3-pose sliding window, identical to eval.py.

    Returns the two jitter series, the same measure computed on the reference
    trajectory, and the timestamp of each window's centre pose, so they can be
    plotted against time.
    """
    jitter = []                    # est jitter with the ground-truth jitter subtracted
    jitter_est = []                # non-normalized
    jitter_ref = []                # the same measure computed on the reference
    timestamps = []

    ref_pos = traj_ref_sync.positions_xyz
    est_pos = traj_est_sync.positions_xyz

    for i in range(len(est_pos) - 2):
        # Compute on est
        window = est_pos[i:i + 3]
        d1 = window[1] - window[0]
        d2 = window[2] - window[1]  # displacement of jitter
        est_displacement = (np.linalg.norm(d1) + np.linalg.norm(d2)) / np.linalg.norm(window[2]-window[0])

        # Compute on ref
        window = ref_pos[i:i + 3]
        d1 = window[1] - window[0]
        d2 = window[2] - window[1]  # displacement of jitter
        ref_displacement = (np.linalg.norm(d1) + np.linalg.norm(d2)) / np.linalg.norm(window[2]-window[0])
     
        jitter.append((est_displacement) - (ref_displacement))
        jitter_est.append((est_displacement))
        jitter_ref.append((ref_displacement))
        timestamps.append(traj_est_sync.timestamps[i + 1])

    return np.array(jitter)

def crop_traj_by_time(traj, ids):
    """
    Crop evo trajectory to timestamps in [t_start, t_end]
    """

    return PoseTrajectory3D(
        positions_xyz=traj.positions_xyz[ids],
        orientations_quat_wxyz=traj.orientations_quat_wxyz[ids],
        timestamps=traj.timestamps[ids]
    )
def angle_between(v1, v2):
    cos_theta = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    return np.degrees(np.arccos(cos_theta))


def dump_stats(traj_ref_sync, traj_est_sync, print_stat=True, label=""):

    # Plot the synced reference/estimate pair with evo's own trajectory plotter
    # try:
    #     evo_fig = plt.figure(figsize=(7, 6))
    #     evo_ax = evo_plot.prepare_axis(evo_fig, evo_plot.PlotMode.xyz)
    #     evo_plot.traj(evo_ax, evo_plot.PlotMode.xyz, traj_ref_sync,
    #                   style='--', color='gray', label='reference',
    #                   plot_start_end_markers=True)
    #     evo_plot.traj(evo_ax, evo_plot.PlotMode.xyz, traj_est_sync,
    #                   style='-', color='blue', label='estimate',
    #                   plot_start_end_markers=True)
    #     evo_ax.set_title(f"{label} ({len(traj_est_sync.timestamps)} synced poses)")
    #     evo_ax.legend()
    # except Exception as e:
    #     print(e)

    # Translation APE
    ape_metric_trans, ape_metric_rot = (None, None)
    try:
        ape_metric_trans = metrics.APE(metrics.PoseRelation.translation_part)
        ape_metric_trans.process_data((traj_ref_sync, traj_est_sync))
        ape_stats = ape_metric_trans.get_all_statistics()
        if print_stat: print(f" Translation APE {json.dumps(ape_stats, indent=1)}")

        # Rotation APE
        ape_metric_rot = metrics.APE(metrics.PoseRelation.rotation_angle_deg)
        ape_metric_rot.process_data((traj_ref_sync, traj_est_sync))
        ape_stats = ape_metric_rot.get_all_statistics()
       
        # if print_stat: print(f" Rotation APE {json.dumps(ape_stats, indent=1)}")
    except Exception as e:
        print(e)

    # Translation RPE
    rpe_metric_trans, rpe_metric_rot = (None, None)
    try:
        rpe_metric_trans = metrics.RPE(metrics.PoseRelation.translation_part, delta=1.0, delta_unit=metrics.Unit.meters)
        rpe_metric_trans.process_data((traj_ref_sync, traj_est_sync))
        rpe_stats = rpe_metric_trans.get_all_statistics()
        # print(f"    Translation APE,\n\t{ape_stats["mean"]=},\n\t{ape_stats["rmse"]=}")
        # print(f" Translation APE {json.dumps(ape_stats, indent=1)}")

        # Rotation RPE - Can also do seconds? if you upgrade version.
        rpe_metric_rot = metrics.RPE(metrics.PoseRelation.rotation_angle_deg, delta=1.0, delta_unit=metrics.Unit.meters)
        rpe_metric_rot.process_data((traj_ref_sync, traj_est_sync))
        rpe_stats = rpe_metric_rot.get_all_statistics()

    except Exception as e:
        print(e)

    try:
        arr = compute_jitter(traj_ref_sync, traj_est_sync)
        jitter_stats = {
                "mean": float(np.nanmean(arr)),
                "median": float(np.nanmedian(arr)),
                "min": float(np.nanmin(arr)),
                "max": float(np.nanmax(arr)),
                "std": float(np.nanstd(arr)),
                "rmse": float(np.sqrt(np.nanmean(arr ** 2))),
                "p95": float(np.nanpercentile(arr, 95)),
                "p99": float(np.nanpercentile(arr, 99)),
            }
        # if print_stat:
            # print(f" Jitter {json.dumps(jitter_stats, indent=1)}")
    except Exception as e:
        print(e)

    jerk = np.array([])
    try:
        jerk = compute_jerk(traj_ref_sync, traj_est_sync)
        jerk_stats = {
                "mean": float(np.nanmean(jerk)),
                "median": float(np.nanmedian(jerk)),
                "min": float(np.nanmin(jerk)),
                "max": float(np.nanmax(jerk)),
                "std": float(np.nanstd(jerk)),
                "rmse": float(np.sqrt(np.nanmean(jerk ** 2))),
                "p95": float(np.nanpercentile(jerk, 95)),
                "p99": float(np.nanpercentile(jerk, 99)),
        }
        print(f" Jerk {json.dumps(jerk_stats, indent=1)}")
    except Exception as e:
        print(e)

    return ape_metric_trans, ape_metric_rot, rpe_metric_trans, rpe_metric_rot, \
        jitter_stats, jitter_stats, jitter_stats, jerk

def plot_metric_cdf(
    metric,
    fig=None,
    ax=None,
    label=None,
    title="",
    xlabel="Error",
    ylabel="CDF",
):

    errors = np.asarray(metric.error)

    # Remove NaNs/Infs just in case
    errors = errors[np.isfinite(errors)]

    # Sort errors
    sorted_errors = np.sort(errors)

    # Compute CDF
    cdf = np.arange(
        1,
        len(sorted_errors) + 1
    ) / len(sorted_errors)

    # Plot
    ax.plot(sorted_errors, cdf, label=label)

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True)
    ax.set_title(title)

    ax.set_xlim({0,10})

    if label is not None:
        ax.legend()

    return fig, ax

def run_eval(args):

    if 'multi' in args.trial_name:
        results_path = f"/home/antond2/Desktop/Research/gtsam_test/results/out/multi/{args.id}/{args.trial_name}"
    else:
        results_path = f"/home/antond2/Desktop/Research/gtsam_test/results/out/{args.trial_name}"

    exe_path = "/home/antond2/Desktop/Research/gtsam_test/out/build/linux-debug/gtsam_test"
    post_path = f"/home/antond2/Desktop/Research/MultiXR-Post/{args.id}/post/{args.trial_name}_post/"
    metadata = json.load(open(f"/home/antond2/Desktop/Research/MultiXR-Post/{args.id}/collect/{args.trial_name}_nuc{args.id}_raw/meta.json", 'r'))
    
    synth_failures_path = f"/home/antond2/Desktop/Research/MultiXR-Post/{args.id}/synth_failures/{args.trial_name}.json"
    try: synth_failures = len(json.load(open(synth_failures_path, 'r'))) > 0
    except Exception as e: synth_failures = False

    real_failures_path = f"/home/antond2/Desktop/Research/MultiXR-Post/real_fails/"
    real_failures = f"{args.trial_name}_nuc{args.id}_slam_cam_traj.csv" in os.listdir(real_failures_path)

    fails = []
    if real_failures: fails = json.load(open(real_failures_path + f"{args.trial_name}_nuc{args.id}_fail.json", 'r'))
    if synth_failures: fails = json.load(open(synth_failures_path, 'r'))

    print()

    fig, (axt, axr) = plt.subplots(1, 2) # Define CDF plot up here, axt - translation
    axt.set_title("")

    # Define a separate CDF plot
    cfig, (caxt, caxr) = plt.subplots(1, 2) # Define CDF plot up here, axt - translation


    metric_report = {
        "IMU": [],
        "Flock": [],
        "Cappella": [],
        "Live-SLAM": []
    }

    plot_report = {
        "IMU": [],
        "Flock": [],
        "Cappella": [],
        "Live-SLAM": []
    }


    for run_config, name in [('no_uwb', "IMU"), ('uwb', "Flock")]:

        ### Run graph executable
        if not args.no_run:
            print(f"Running graph with {run_config}")

            run_args = [
                exe_path,
                args.trial_name,
                "none",
                run_config,
                "0.0",
                "true"
            ]

            if args.lpf_off: run_args.append("--lpf_off")
            if args.rcf_on: run_args.append("--rcf_on")

            subprocess.run(run_args,
            capture_output=True,
            text=True)
            print("Graph complete")

        
        ### Organize filepaths

        plot_paths = SimpleNamespace()
        if real_failures: plot_paths.live_slam_path = f"{post_path}/all.json" # Fetch the real live SLAM from all.json
        else: plot_paths.live_slam_path = f"{results_path}/aligned_live_slam.json" # Fetch what we generated with the graph
        plot_paths.post_slam_path = f"{post_path}/all.json"
        plot_paths.est_path = f"{results_path}/est_{run_config}.json"
        # plot_paths.est_path = None
        plot_paths.opti_path = f"{post_path}/all.json"
        plot_paths.slam_path = f"{post_path}/all.json" # I belive this is unaligned post SLAM always

        eval_paths = SimpleNamespace()
        if real_failures: eval_paths.slam_path = f"{post_path}/aligned_live_slam.txt" # Fetch the real live SLAM from all.json
        else: eval_paths.slam_path = f"{results_path}/aligned_live_slam.txt" # Fetch what we generated with the graph
        eval_paths.est_path = f"{results_path}/est_{run_config}.txt"
        eval_paths.opti_path = f"{post_path}/opti.txt"

        if not args.no_plot:
            # Plot trajectories with MultiXR-Post
            plot_report[name] = plot_trial_paper(args.id, 
                    args.trial_name,
                    slam_stride = -2,
                    est_stride = -1,
                    opti_stride = -1,
                    run_config = run_config,
                    label_text = name,
                    show_live_slam = True,
                    paths = plot_paths,
                    show=False)
        
        
        # Evaluate with EVO
        # We have the estimated trajectory as a .txt in TUM format and .json in HTM format
        # We have the optitrack trajectory as a .json in all.json
        est_traj = []
        gt_traj = []
        try:
            est_traj = read_inverted_tum_trajectory_file(eval_paths.est_path)
            gt_traj = read_inverted_tum_trajectory_file(eval_paths.opti_path)
            if len(est_traj.timestamps) == 0:
                print(f"Empty estimated trajectory: {eval_paths.est_path}")
                return None, None
            if len(gt_traj.timestamps) == 0:
                print(f"Empty ground-truth trajectory: {eval_paths.opti_path}")
                return None, None
        except Exception as e:
            print(e)
            return None, None

        traj_ref_sync, traj_est_sync = sync.associate_trajectories(
                                            gt_traj,
                                            est_traj,
                                            max_diff = 0.05
                                        )
        print(f"{name}")
        print(f"Error Metrics")
        print()

        # Print metrics over entire trajectory
        # print(f"Entire trajectory")
        ape_trans, ape_rot, rpe_trans, rpe_rot, jitter, jitter_est, jitter_est_displacement, jerk = dump_stats(traj_ref_sync, traj_est_sync)
        metric_report[name].append(
            {
                "full_traj": True,
                "ape_trans": ape_trans,
                "ape_rot": ape_rot,
                "rpe_trans": rpe_trans,
                "rpe_rot": rpe_rot,
                "jerk": jerk,
            }
        )
        print()

        # Print metrics for each individual failure segment
        # BUG: Somehow the trajectory lengths are greater than 0, but cropping sends them to 0?
        # BUG: I'm cropping based on the trajectory timestamp, not from the absolute start of the dataset
        # So this is not the right interval that I'm looking at.
        for interval in fails:
            # start, end = traj_ref_sync.timestamps[0] + interval["start"] , traj_ref_sync.timestamps[0] + interval["end"]
            start, end = (metadata["start_ns"] * 1e-9) + interval["start"] , (metadata["start_ns"] * 1e-9) + interval["end"]

            print(f"Failure {interval["start"]}s - {interval["end"]}s")

            ref_ids = np.where(
                (traj_ref_sync.timestamps >= start) &
                (traj_ref_sync.timestamps <= end)
            )[0]

            est_ids = np.where(
                (traj_est_sync.timestamps >= start) &
                (traj_est_sync.timestamps <= end)
            )[0]

            # WHY trajectory lengths OFF BY 1 sometimes????

            ids = est_ids

            if len(traj_est_sync.timestamps) == 0:
                print(f"Empty estimated trajectory")
                return None, None
            if len(traj_ref_sync.timestamps) == 0:
                print(f"Empty ground-truth trajectory")
                return None, None
            
            try:
                cropped_traj_ref_sync = crop_traj_by_time(traj_ref_sync, ids) # Need to limit to the smallest number of poses?
                cropped_traj_est_sync = crop_traj_by_time(traj_est_sync, ids)
            except Exception as e:
                print(e)
                return None, None

            crop_ape_trans, crop_ape_rot, crop_rpe_trans, crop_rpe_rot, crop_jitter, crop_jitter_est, crop_jitter_est_displacement, crop_jerk = dump_stats(cropped_traj_ref_sync, cropped_traj_est_sync)

            if not args.no_plot:
                plot_metric_cdf(
                    crop_ape_trans,
                    fig=cfig,
                    ax=caxt,
                    label=name,
                    title=f"Failure {interval["start"]}s - {interval["end"]}s",
                    xlabel="APE Translation Error (m)"
                )
                plot_metric_cdf(
                    crop_ape_rot,
                    fig=cfig,
                    ax=caxr,
                    label=name,
                    title=f"Failure {interval["start"]}s - {interval["end"]}s",
                    xlabel="APE Rotation Error (deg)"
                )

                # Plot CDF over entire trajectory
                plot_metric_cdf(
                    crop_rpe_trans,
                    fig=cfig,
                    ax=axt,
                    label=name,
                    title=f"Failure {interval["start"]}s - {interval["end"]}s",
                    xlabel="RPE (Delta=1m) Translation Error (m)"
                )
                plot_metric_cdf(
                    crop_rpe_rot,
                    fig=cfig,
                    ax=axr,
                    label=name,
                    title=f"Failure {interval["start"]}s - {interval["end"]}s",
                    xlabel="RPE (Delta=1m) Rotation Error (deg)"
                )

            metric_report[name].append(
                {
                    "fail": interval,
                    "ape_trans": crop_ape_trans,
                    "ape_rot": crop_ape_rot,
                    "rpe_trans": crop_rpe_trans,
                    "rpe_rot": crop_rpe_rot,
                    "jerk": crop_jerk,
                }
            )
        
        print()
        print("----------------------------------")

    if not args.hide_plots:
        plt.tight_layout()
        plt.show()

    return metric_report, plot_report

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("id", type=int)
    parser.add_argument("trial_name", help="Trial name")
    parser.add_argument("--no_run", action="store_true")
    parser.add_argument("--hide_plots", action="store_true")
    parser.add_argument("--no_plot", action="store_true")
    args = parser.parse_args()

    # Hard coded defaults, added for calls from MultiXR-Eval running component ablation
    args.lpf_off = False
    args.rcf_on = False

    run_eval(args)