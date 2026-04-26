import re
import argparse
import matplotlib.pyplot as plt
import os
import subprocess
import json

from evo.tools import file_interface
from evo.core import metrics
from evo.core import sync

import sys
sys.path.append("/home/antond2/Desktop/Research/MultiXR-Post/")
from plot_all import plot_trial


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("id", type=int)
    parser.add_argument("trial_name", help="Trial name")
    args = parser.parse_args()

    exe_path = "/home/antond2/Desktop/Research/gtsam_test/out/build/linux-debug/gtsam_test"
    optitrack_gt_path = f"/home/antond2/Desktop/Research/MultiXR-Post/{args.id}/post/{args.trial_name}_post/opti.txt"

    gt_traj = file_interface.read_tum_trajectory_file(optitrack_gt_path)


    for run_config in ['no_uwb', 'uwb']:
        print("Running graph")
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
        # TODO: Later we can read in the synth_failures file from post and use that to evaluate each error segment individually
            # Apparently once you have a trajectory object, you can write a function to crop it by timestamps (ask Chat if you need to do this later)
        est_path = f"/home/antond2/Desktop/Research/gtsam_test/results/out/{args.trial_name}/est_{run_config}.txt"
        est_traj = file_interface.read_tum_trajectory_file(est_path)

        traj_ref_sync, traj_est_sync = sync.associate_trajectories(
                                            gt_traj,
                                            est_traj
                                        )
        # Translation APE
        ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
        ape_metric.process_data((traj_ref_sync, traj_est_sync))
        ape_stats = ape_metric.get_all_statistics()
        print(f" Translation APE {json.dumps(ape_stats, indent=1)}")

        # Rotation APE
        ape_metric = metrics.APE(metrics.PoseRelation.rotation_angle_deg)
        ape_metric.process_data((traj_ref_sync, traj_est_sync))
        ape_stats = ape_metric.get_all_statistics()
        print(f" Rotational APE {json.dumps(ape_stats, indent=1)}")

        # Do we really need RPE?


   
   # TODO:
   # Automate EVO evaluation using evo api
   # Automate runtime plotting

    plt.show()

if __name__ == "__main__":
    main()
