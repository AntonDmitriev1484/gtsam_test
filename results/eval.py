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
    # print(f"    Translation APE,\n\t{ape_stats["mean"]=},\n\t{ape_stats["rmse"]=}")
    print(f" Translation APE {json.dumps(ape_stats, indent=1)}")

    # Rotation APE
    ape_metric_rot = metrics.APE(metrics.PoseRelation.rotation_angle_deg)
    ape_metric_rot.process_data((traj_ref_sync, traj_est_sync))
    ape_stats = ape_metric_rot.get_all_statistics()
    # print(f" Rotational APE {json.dumps(ape_stats, indent=1)}")
    # print(f"    Rotation APE,\n\t{ape_stats["mean"]=},\n\t{ape_stats["rmse"]=}")
    print(f" Rotation APE {json.dumps(ape_stats, indent=1)}")

    # Translation RPE
    rpe_metric_trans = metrics.RPE(metrics.PoseRelation.translation_part, delta=1.0, delta_unit=metrics.Unit.meters)
    rpe_metric_trans.process_data((traj_ref_sync, traj_est_sync))
    rpe_stats = rpe_metric_trans.get_all_statistics()
    # print(f"    Translation APE,\n\t{ape_stats["mean"]=},\n\t{ape_stats["rmse"]=}")
    # print(f" Translation APE {json.dumps(ape_stats, indent=1)}")

    # Rotation RPE - Can also do seconds? if you upgrade version.
    rpe_metric_rot = metrics.RPE(metrics.PoseRelation.rotation_angle_deg, delta=1.0, delta_unit=metrics.Unit.meters)
    rpe_metric_rot.process_data((traj_ref_sync, traj_est_sync))
    rpe_stats = rpe_metric_rot.get_all_statistics()
    # print(f" Rotational APE {json.dumps(ape_stats, indent=1)}")
    # print(f"    Rotation APE,\n\t{ape_stats["mean"]=},\n\t{ape_stats["rmse"]=}")
    # print(f" Rotation APE {json.dumps(ape_stats, indent=1)}")

    return ape_metric_trans, ape_metric_rot, rpe_metric_trans, rpe_metric_rot

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

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("id", type=int)
    parser.add_argument("trial_name", help="Trial name")
    parser.add_argument("--no_run", action="store_true")
    args = parser.parse_args()

    if 'multi' in args.trial_name:
        results_path = f"/home/antond2/Desktop/Research/gtsam_test/results/out/multi/{args.id}/{args.trial_name}"
    else:
        results_path = f"/home/antond2/Desktop/Research/gtsam_test/results/out/{args.trial_name}"

    exe_path = "/home/antond2/Desktop/Research/gtsam_test/out/build/linux-debug/gtsam_test"
    post_path = f"/home/antond2/Desktop/Research/MultiXR-Post/{args.id}/post/{args.trial_name}_post/"
    optitrack_gt_path = post_path + "opti.txt"


    synth_failures_path = f"/home/antond2/Desktop/Research/MultiXR-Post/{args.id}/synth_failures/{args.trial_name}.json"

    real_failures_path = f"/home/antond2/Desktop/Research/MultiXR-Post/real_fails/"
    real_failures = f"{args.trial_name}_nuc{args.id}_slam_cam_traj.csv" in os.listdir(real_failures_path)

    gt_traj = file_interface.read_tum_trajectory_file(optitrack_gt_path)

    fails = []
    if real_failures:
        fails = json.load(open(real_failures_path + f"{args.trial_name}_nuc{args.id}_fail.json", 'r'))
    else:
        fails = json.load(open(synth_failures_path, 'r'))

    fig, (axt, axr) = plt.subplots(1, 2) # Define CDF plot up here, axt - translation
    axt.set_title("")

    # Define a separate CDF plot
    cfig, (caxt, caxr) = plt.subplots(1, 2) # Define CDF plot up here, axt - translation

    for run_config, name in [('no_uwb', "IMU"), ('uwb', "Flock")]:
    # for run_config, name in [('no_uwb', "IMU")]:
        ### Run graph executable
        if not args.no_run:
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
                show_live_slam = real_failures,
                est_path = f"{results_path}/est_{run_config}.json",
                show=False)
        
        
        # Evaluate with EVO
        # We have the estimated trajectory as a .txt in TUM format and .json in HTM format
        # We have the optitrack trajectory as a .json in all.json

        est_path = f"{results_path}/est_{run_config}.txt"
        est_traj = file_interface.read_tum_trajectory_file(est_path)

        traj_ref_sync, traj_est_sync = sync.associate_trajectories(
                                            gt_traj,
                                            est_traj,
                                            max_diff = 0.05
                                        )
        print(f"Error Metrics")
        print()

        # Print metrics over entire trajectory
        # print(f"Entire trajectory")
        # ape_metric_trans, ape_metric_rot, _, _ = dump_stats(traj_ref_sync, traj_est_sync)
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
            crop_ape_trans, crop_ape_rot, crop_rpe_trans, crop_rpe_rot = dump_stats(cropped_traj_ref_sync, cropped_traj_est_sync)

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
        
        print()
        print("----------------------------------")

    if real_failures:
        name = "Live SLAM"
        print("Comparing with Live SLAM")
    else:
        name = "Synthetic Live SLAM"
        
    slam_traj = file_interface.read_tum_trajectory_file(post_path + "aligned_live_slam.txt")

    traj_ref_sync, traj_est_sync = sync.associate_trajectories(
                                        gt_traj,
                                        slam_traj,
                                        max_diff = 0.05
                                    )
    
    # Add SLAM trajectory to the error metrics:
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
        crop_ape_trans, crop_ape_rot, crop_rpe_trans, crop_rpe_rot = dump_stats(cropped_traj_ref_sync, cropped_traj_est_sync)

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

    # Plot error CDF
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()


    # if real_failures:
    #     for run_config, name in [('no_uwb', "IMU"), (None, "Live-SLAM"), ('uwb', "Flock")]:

    #         ### Run graph executable
    #         if not args.no_run and run_config is not None:
    #             print(f"Running graph with {run_config}")
    #             subprocess.run([
    #                 exe_path,
    #                 args.trial_name,
    #                 "none",
    #                 run_config,
    #                 "0.0",
    #                 "true"
    #             ],
    #             capture_output=True,
    #             text=True)
    #             print("Graph complete")

    #         # Plot trajectories with MultiXR-Post
    #         plot_trial(args.id, 
    #                 args.trial_name,
    #                 slam_stride = -1,
    #                 est_stride = -1,
    #                 run_config = run_config,
    #                 label_text = name,
    #                 est_path = f"{results_path}/est_{run_config}.json",
    #                 show=False)
            
    #         # Evaluate with EVO
    #         # We have the estimated trajectory as a .txt in TUM format and .json in HTM format
    #         # We have the optitrack trajectory as a .json in all.json

    #         est_path = ""
    #         if name == "Live-SLAM":
    #             est_path = f"{post_path}"+"aligned_live_slam.txt" # If we're using live SLAM, fetch from Post
    #         else:
    #             est_path = f"{results_path}/est_{run_config}.txt"
    #         est_traj = file_interface.read_tum_trajectory_file(est_path)


    #         traj_ref_sync, traj_est_sync = sync.associate_trajectories(
    #                                             gt_traj,
    #                                             est_traj,
    #                                             max_diff = 0.05
    #                                         )
    #         print(f"Error Metrics")
    #         print()

    #         # Print metrics over entire trajectory
    #         print(f"Entire trajectory")
    #         ape_metric_trans, ape_metric_rot, _, _ = dump_stats(traj_ref_sync, traj_est_sync)
    #         print()

    #         # Print metrics for each individual failure segment
    #         for interval in fails:
    #             start, end = traj_ref_sync.timestamps[0] + interval["start"] , traj_ref_sync.timestamps[0] + interval["end"]
    #             print(f"Failure {interval["start"]}s - {interval["end"]}s")

    #             ref_ids = np.where(
    #                 (traj_ref_sync.timestamps >= start) &
    #                 (traj_ref_sync.timestamps <= end)
    #             )[0]

    #             est_ids = np.where(
    #                 (traj_est_sync.timestamps >= start) &
    #                 (traj_est_sync.timestamps <= end)
    #             )[0]

    #             # WHY trajectory lengths OFF BY 1 sometimes????

    #             ids = est_ids
                
    #             cropped_traj_ref_sync = crop_traj_by_time(traj_ref_sync, ids) # Need to limit to the smallest number of poses?
    #             cropped_traj_est_sync = crop_traj_by_time(traj_est_sync, ids)
    #             crop_ape_trans, crop_ape_rot, crop_rpe_trans, crop_rpe_rot = dump_stats(cropped_traj_ref_sync, cropped_traj_est_sync)

    #             plot_metric_cdf(
    #                 crop_ape_trans,
    #                 fig=cfig,
    #                 ax=caxt,
    #                 label=name,
    #                 title=f"Failure {interval["start"]}s - {interval["end"]}s",
    #                 xlabel="APE Translation Error (m)"
    #             )
    #             plot_metric_cdf(
    #                 crop_ape_rot,
    #                 fig=cfig,
    #                 ax=caxr,
    #                 label=name,
    #                 title=f"Failure {interval["start"]}s - {interval["end"]}s",
    #                 xlabel="APE Rotation Error (deg)"
    #             )

    #                         # Plot CDF over entire trajectory
    #             plot_metric_cdf(
    #                 crop_rpe_trans,
    #                 fig=cfig,
    #                 ax=axt,
    #                 label=name,
    #                 title=f"Failure {interval["start"]}s - {interval["end"]}s",
    #                 xlabel="RPE (Delta=1m) Translation Error (m)"
    #             )
    #             plot_metric_cdf(
    #                 crop_rpe_rot,
    #                 fig=cfig,
    #                 ax=axr,
    #                 label=name,
    #                 title=f"Failure {interval["start"]}s - {interval["end"]}s",
    #                 xlabel="RPE (Delta=1m) Rotation Error (deg)"
    #             )
            
    #         print()
    #         print("----------------------------------")

    #     # Plot error CDF
    #     plt.tight_layout()
    #     plt.show()