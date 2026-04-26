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

import sys
sys.path.append("/home/antond2/Desktop/Research/MultiXR-Post/")
from plot_all import plot_trial

import copy


def crop_traj_by_time(traj, t_start, t_end):
    """
    Crop evo trajectory to timestamps in [t_start, t_end]
    """
    ids = np.where(
        (traj.timestamps >= t_start) &
        (traj.timestamps <= t_end)
    )[0]

    return PoseTrajectory3D(
        positions_xyz=traj.positions_xyz[ids],
        orientations_quat_wxyz=traj.orientations_quat_wxyz[ids],
        timestamps=traj.timestamps[ids]
    )

def dump_stats(traj_ref_sync, traj_est_sync):
    # Translation APE
    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
    ape_metric.process_data((traj_ref_sync, traj_est_sync))
    ape_stats = ape_metric.get_all_statistics()
    print(f"    Translation APE, {ape_stats["mean"]=}, {ape_stats["rmse"]=}")
    # print(f" Translation APE {json.dumps(ape_stats, indent=1)}")

    # Rotation APE
    ape_metric = metrics.APE(metrics.PoseRelation.rotation_angle_deg)
    ape_metric.process_data((traj_ref_sync, traj_est_sync))
    ape_stats = ape_metric.get_all_statistics()
    # print(f" Rotational APE {json.dumps(ape_stats, indent=1)}")
    print(f"    Rotation APE, {ape_stats["mean"]=}, {ape_stats["rmse"]=}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("id", type=int)
    parser.add_argument("trial_name", help="Trial name")
    args = parser.parse_args()

    exe_path = "/home/antond2/Desktop/Research/gtsam_test/out/build/linux-debug/gtsam_test"
    post_path = f"/home/antond2/Desktop/Research/MultiXR-Post/{args.id}/post/{args.trial_name}_post/"
    optitrack_gt_path = post_path + "opti.txt"
    failures_path = f"/home/antond2/Desktop/Research/MultiXR-Post/{args.id}/synth_failures/{args.trial_name}.json"

    gt_traj = file_interface.read_tum_trajectory_file(optitrack_gt_path)
    fails = json.load(open(failures_path, 'r'))

    for run_config in ['no_uwb', 'uwb']:
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

        plot_trial(args.id, 
                args.trial_name,
                slam_stride = -1,
                est_stride = -1,
                run_config = run_config)
        
        # Evaluation
        # We have the estimated trajectory as a .txt in TUM format and .json in HTM format
        # We have the optitrack trajectory as a .json in all.json

        est_path = f"/home/antond2/Desktop/Research/gtsam_test/results/out/{args.trial_name}/est_{run_config}.txt"
        est_traj = file_interface.read_tum_trajectory_file(est_path)

        traj_ref_sync, traj_est_sync = sync.associate_trajectories(
                                            gt_traj,
                                            est_traj
                                        )
        print(f"Error Metrics")
        print()

        # Print metrics over entire trajectory
        print(f"Entire trajectory")
        dump_stats(traj_ref_sync, traj_est_sync)
        print()

        # Print metrics for each individual failure segment
        for interval in fails:
            start, end = traj_ref_sync.timestamps[0] + interval["start"] , traj_ref_sync.timestamps[0] + interval["end"]
            print(f"Failure {interval["start"]}s - {interval["end"]}s")
            cropped_traj_ref_sync = crop_traj_by_time(traj_ref_sync, start, end)
            cropped_traj_est_sync = crop_traj_by_time(traj_est_sync, start, end)
            dump_stats(cropped_traj_ref_sync, cropped_traj_est_sync)
        
        print()
        print("----------------------------------")

   # TODO:
   # Automate EVO evaluation using evo api
   # Automate runtime plotting

    plt.show()

if __name__ == "__main__":
    main()
