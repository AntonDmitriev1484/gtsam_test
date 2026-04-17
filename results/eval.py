import re
import argparse
import matplotlib.pyplot as plt
import os
import subprocess


import sys
sys.path.append("/home/antond2/Desktop/Research/MultiXR-Post/")
from plot_all import plot_trial

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("id", type=int)
    parser.add_argument("trial_name", help="Trial name")
    args = parser.parse_args()

    exe_path = "/home/antond2/Desktop/Research/gtsam_test/out/build/linux-debug/gtsam_test"
    graph_out_path = "/home/antond2/Desktop/Research/gtsam_test/results/out/"+args.trial_name
    optitrack_gt_path = f"/home/antond2/Desktop/Research/MultiXR-Post/{args.id}/collect/{args.trial_name}_nuc{args.id}_raw"


    for run_config in ['no_uwb']:
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
   
   # TODO:
   # Automate EVO evaluation using evo api
   # Automate runtime plotting

    plt.show()

if __name__ == "__main__":
    main()
