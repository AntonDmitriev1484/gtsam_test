import re
import argparse
import matplotlib.pyplot as plt
import os
import subprocess
import json
import numpy as np

from evo.tools import file_interface
from evo.core import metrics
from evo.core import sync
from evo.core.trajectory import PoseTrajectory3D
from plot_runtimes import plot_isam_runtimes

import sys
sys.path.append("/home/antond2/Desktop/Research/MultiXR-Post/")
from plot_all import plot_trial


import copy


def crop_traj_by_time(traj, ids):
    """
    Crop evo trajectory to timestamps in [t_start, t_end]
    """

    return PoseTrajectory3D(
        positions_xyz=traj.positions_xyz[ids],
        orientations_quat_wxyz=traj.orientations_quat_wxyz[ids],
        timestamps=traj.timestamps[ids]
    )

def dump_stats(traj_ref_sync, traj_est_sync):

    # Translation APE
    ape_metric_trans = metrics.APE(metrics.PoseRelation.translation_part)
    ape_metric_trans.process_data((traj_ref_sync, traj_est_sync))
    ape_stats = ape_metric_trans.get_all_statistics()
    print(f"    Translation APE,\n\t{ape_stats["mean"]=},\n\t{ape_stats["rmse"]=}")
    # print(f" Translation APE {json.dumps(ape_stats, indent=1)}")

    # Rotation APE
    ape_metric_rot = metrics.APE(metrics.PoseRelation.rotation_angle_deg)
    ape_metric_rot.process_data((traj_ref_sync, traj_est_sync))
    ape_stats = ape_metric_rot.get_all_statistics()
    # print(f" Rotational APE {json.dumps(ape_stats, indent=1)}")
    print(f"    Rotation APE,\n\t{ape_stats["mean"]=},\n\t{ape_stats["rmse"]=}")

    return ape_metric_trans, ape_metric_rot

def plot_metric_cdf(
    metric,
    fig=None,
    ax=None,
    label=None,
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

    if label is not None:
        ax.legend()

    return fig, ax

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("id", type=int)
    parser.add_argument("trial_name", help="Trial name")
    args = parser.parse_args()

    results_path = f"/home/antond2/Desktop/Research/gtsam_test/results/out/multi/{args.id}/{args.trial_name}"
    exe_path = "/home/antond2/Desktop/Research/gtsam_test/out/build/linux-debug/gtsam_test"
    post_path = f"/home/antond2/Desktop/Research/MultiXR-Post/{args.id}/post/{args.trial_name}_post/"
    optitrack_gt_path = post_path + "opti.txt"
    failures_path = f"/home/antond2/Desktop/Research/MultiXR-Post/{args.id}/synth_failures/{args.trial_name}.json"

    gt_traj = file_interface.read_tum_trajectory_file(optitrack_gt_path)
    fails = json.load(open(failures_path, 'r'))


    fig, (axt, axr) = plt.subplots(1, 2) # Define CDF plot up here, axt - translation
    axt.set_title("")

    for run_config, name in [('no_uwb', "IMU"), ('uwb', "Flock")]:

        ### Run graph executable
        print(f"Running graph with {run_config}")
        subprocess.run([
            exe_path,
            args.trial_name,
            "none",
            run_config,
            "0.0",
            "true"
        ],
        capture_output=True,
        text=True)
        print("Graph complete")

        # Plot trajectories with MultiXR-Post
        plot_trial(args.id, 
                args.trial_name,
                slam_stride = -1,
                est_stride = -1,
                run_config = run_config,
                label_text = name,
                show=False)
        # Its this call here thats causing it to plot early
        
        # Evaluate with EVO
        # We have the estimated trajectory as a .txt in TUM format and .json in HTM format
        # We have the optitrack trajectory as a .json in all.json

        est_path = f"{results_path}/est_{run_config}.txt"
        est_traj = file_interface.read_tum_trajectory_file(est_path)

        traj_ref_sync, traj_est_sync = sync.associate_trajectories(
                                            gt_traj,
                                            est_traj
                                        )
        print(f"Error Metrics")
        print()

        # Print metrics over entire trajectory
        print(f"Entire trajectory")
        ape_metric_trans, ape_metric_rot = dump_stats(traj_ref_sync, traj_est_sync)
        print()

        # Print metrics for each individual failure segment
        for interval in fails:
            start, end = traj_ref_sync.timestamps[0] + interval["start"] , traj_ref_sync.timestamps[0] + interval["end"]
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
            
            cropped_traj_ref_sync = crop_traj_by_time(traj_ref_sync, ids) # Need to limit to the smallest number of poses?
            cropped_traj_est_sync = crop_traj_by_time(traj_est_sync, ids)
            dump_stats(cropped_traj_ref_sync, cropped_traj_est_sync)

        # Plot CDF over entire trajectory
        plot_metric_cdf(
            ape_metric_trans,
            fig=fig,
            ax=axt,
            label=name,
            xlabel="APE Translation Error (m)"
        )
        plot_metric_cdf(
            ape_metric_rot,
            fig=fig,
            ax=axr,
            label=name,
            xlabel="APE Rotation Error (deg)"
        )
        
        print()
        print("----------------------------------")

    # Plot iSam / smoother runtimes for trail
    # plot_isam_runtimes(
    #     f"{results_path}/log_dump.txt",
    #     f"{args.trial_name} Tracker Runtimes"
    # )

    # Plot error CDF
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
