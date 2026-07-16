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
from types import SimpleNamespace

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

def dump_stats(traj_ref_sync, traj_est_sync, print_stat=True):

    # Translation APE
    ape_metric_trans, ape_metric_rot = (None, None)
    try:
        ape_metric_trans = metrics.APE(metrics.PoseRelation.translation_part)
        ape_metric_trans.process_data((traj_ref_sync, traj_est_sync))
        ape_stats = ape_metric_trans.get_all_statistics()
        # print(f"    Translation APE,\n\t{ape_stats["mean"]=},\n\t{ape_stats["rmse"]=}")
        if print_stat: print(f" Translation APE {json.dumps(ape_stats, indent=1)}")

        # Rotation APE
        ape_metric_rot = metrics.APE(metrics.PoseRelation.rotation_angle_deg)
        ape_metric_rot.process_data((traj_ref_sync, traj_est_sync))
        ape_stats = ape_metric_rot.get_all_statistics()
        # print(f" Rotational APE {json.dumps(ape_stats, indent=1)}")
        # print(f"    Rotation APE,\n\t{ape_stats["mean"]=},\n\t{ape_stats["rmse"]=}")
        if print_stat: print(f" Rotation APE {json.dumps(ape_stats, indent=1)}")
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
        # print(f" Rotational APE {json.dumps(ape_stats, indent=1)}")
        # print(f"    Rotation APE,\n\t{ape_stats["mean"]=},\n\t{ape_stats["rmse"]=}")
        # print(f" Rotation APE {json.dumps(ape_stats, indent=1)}")
    except Exception as e:
        print(e)

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

def run_eval(args):

    if 'multi' in args.trial_name:
        results_path = f"/home/antond2/Desktop/Research/gtsam_test/results/out/multi/{args.id}/{args.trial_name}"
    else:
        results_path = f"/home/antond2/Desktop/Research/gtsam_test/results/out/{args.trial_name}"

    exe_path = "/home/antond2/Desktop/Research/gtsam_test/out/build/linux-debug/gtsam_test"
    post_path = f"/home/antond2/Desktop/Research/MultiXR-Post/{args.id}/post/{args.trial_name}_post/"
    merge_path = f"/home/antond2/Desktop/Research/MultiXR-Post/merged/{args.trial_name}_merged/"
    metadata = json.load(open(f"/home/antond2/Desktop/Research/MultiXR-Post/{args.id}/collect/{args.trial_name}_nuc{args.id}_raw/meta.json", 'r'))
    
    synth_failures_path = f"/home/antond2/Desktop/Research/MultiXR-Post/{args.id}/synth_failures/{args.trial_name}.json"
    try: synth_failures = len(json.load(open(synth_failures_path, 'r'))) > 0
    except Exception as e: synth_failures = False

    real_failures_path = f"/home/antond2/Desktop/Research/MultiXR-Post/real_fails/"
    real_failures = f"{args.trial_name}_nuc{args.id}_slam_cam_traj.csv" in os.listdir(real_failures_path)

    fails = []
    if real_failures: fails = json.load(open(real_failures_path + f"{args.trial_name}_nuc{args.id}_fail.json", 'r'))
    if synth_failures: fails = json.load(open(synth_failures_path, 'r'))

    if synth_failures:
        # Need to complete the synthetic failure by running the graph in live-SLAM mode
        if not args.no_run:
            run_config = "live-slam-integration"
            print(f"Completing synthetic failure! Running graph with {run_config}")
            subprocess.run([
                exe_path,
                args.trial_name,
                "none",
                run_config,
                "0.0",
                "true",
                "no-loc"
            ],
            capture_output=True,
            text=True)
            print("Graph complete")
            # /home/antond2/Desktop/Research/MultiXR-Post/2/collect/opti_multi1_free_circle_nuc2_raw/meta.json

        output_synth_slam = json.load(open(f"{results_path}/aligned_live_slam.json",'r')) # Fetch what we generated with the graph
        metadata = json.load(open(f"/home/antond2/Desktop/Research/MultiXR-Post/{args.id}/collect/{args.trial_name}_nuc{args.id}_raw/meta.json", 'r'))
        all_data_start_ts = metadata["start_ns"] * 1e-9

        input_synth_slam = [j for j in json.load(open(f"{post_path}/all.json")) if j["type"] == "aligned_live_slam_pose"]

        for interval in fails:
            start_fail = all_data_start_ts + interval["start"]
            init_newmap = all_data_start_ts + interval["init_newmap"]
            end_fail = all_data_start_ts + interval["end"]

            for j in output_synth_slam:
                if init_newmap > j["t"] > start_fail: j["status"] = "imu"
                elif end_fail > j["t"] >= init_newmap: j["status"] = "init_newmap"

        class NumpyEncoder(json.JSONEncoder):
            def default(self, obj):
                if isinstance(obj, np.ndarray):
                    return obj.tolist()
                if hasattr(obj, '__dict__'):
                    return vars(obj)
                return super().default(obj)
        json.dump(output_synth_slam, open(f"{results_path}/aligned_live_slam.json",'w'), cls=NumpyEncoder, indent=1)

    print()

    fig, (axt, axr) = plt.subplots(1, 2) # Define CDF plot up here, axt - translation
    axt.set_title("")

    # Define a separate CDF plot
    cfig, (caxt, caxr) = plt.subplots(1, 2) # Define CDF plot up here, axt - translation


    metric_report = {
        "IMU": [],
        "Flock": [],
        "Live-SLAM": [],
        "Anchors": []
    }

    plot_report = {
        "IMU": None,
        "Flock": None,
        "Live-SLAM": None
    }


    for run_config, name in [('no_uwb', "IMU"), ('uwb', "Flock")]:
        ### Run graph executable
        if not args.no_run:
            print(f"Running graph with {run_config}")
            subprocess.run([
                exe_path,
                args.trial_name,
                "none",
                run_config,
                "0.0",
                "true",
                args.anchor_selfloc_strategy
            ],
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
            plot_report[name] = plot_trial(args.id, 
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
            est_traj = file_interface.read_tum_trajectory_file(eval_paths.est_path)
            gt_traj = file_interface.read_tum_trajectory_file(eval_paths.opti_path)
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
        print(f"Error Metrics")
        print()

        # Print metrics over entire trajectory
        # print(f"Entire trajectory")
        ape_trans, ape_rot, rpe_trans, rpe_rot = dump_stats(traj_ref_sync, traj_est_sync, print_stat=True)
        metric_report[name].append(
            {
                "full_traj": True,
                "ape_trans": ape_trans,
                "ape_rot": ape_rot,
                "rpe_trans": rpe_trans,
                "rpe_rot": rpe_rot,
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

            crop_ape_trans, crop_ape_rot, crop_rpe_trans, crop_rpe_rot = dump_stats(cropped_traj_ref_sync, cropped_traj_est_sync)

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
                }
            )

        
        print()
        print("----------------------------------")

        # If running Flock, evaluate anchor self-localization results

        if run_config == "uwb":

            for id in [1,5]:
                plot_paths.anchor_optimization_path = f"{results_path}/anchor_{id}_optimization.json"
                plot_paths.final_anchor_estimate_path = f"{results_path}/anchors_estimate.json"
                eval_paths.final_anchor_estimate_path = plot_paths.final_anchor_estimate_path

                anchors_est = json.load(open(eval_paths.final_anchor_estimate_path, 'r'))
                anchors_gt = json.load(open(f"{merge_path}anchors_gt.json", 'r'))

                if not args.no_plot:
                    # Plot trajectories with MultiXR-Post
                    plot_report[name] = plot_trial(args.id, 
                            args.trial_name,
                            slam_stride = -2,
                            est_stride = -1,
                            opti_stride = -1,
                            run_config = run_config,
                            label_text = name,
                            show_live_slam = True,
                            paths = plot_paths,
                            anchors = True,
                            show=False)
                    

                # GTSAM test and post_process both write the position of the anchor in the world frame already
                anchor_est = [j["position"] for j in anchors_est if j["ID"] == id][0]
                anchor_gt = [j["position"] for j in anchors_gt if j["ID"] == id][0]

                distance = np.linalg.norm(np.array(anchor_est) - np.array(anchor_gt))
                print(f"Anchor {id} error: {distance}")
                metric_report["Anchors"].append({"id":id, "error": distance})


    
    # Add SLAM trajectory to the error metrics:
    # Print metrics for each individual failure segment

    if real_failures:
        name = "Live SLAM"
    else:
        name = "Synthetic Live SLAM"
    print(f"Comparing with {name}")

    slam_traj = []
    try:
        slam_traj = file_interface.read_tum_trajectory_file(eval_paths.slam_path)
    except Exception as e:
        print(e)
        return None, None

    traj_ref_sync, traj_est_sync = sync.associate_trajectories(
                                        gt_traj,
                                        slam_traj,
                                        max_diff = 0.05
                                    )
    
    ape_trans, ape_rot, rpe_trans, rpe_rot = dump_stats(traj_ref_sync, traj_est_sync, print_stat=True)
    metric_report["Live-SLAM"].append(
        {
            "full_traj": True,
            "ape_trans": ape_trans,
            "ape_rot": ape_rot,
            "rpe_trans": rpe_trans,
            "rpe_rot": rpe_rot,
        }
    )
    print()

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

        ids = est_ids
        
        cropped_traj_ref_sync = crop_traj_by_time(traj_ref_sync, ids) # Need to limit to the smallest number of poses?
        cropped_traj_est_sync = crop_traj_by_time(traj_est_sync, ids)
        crop_ape_trans, crop_ape_rot, crop_rpe_trans, crop_rpe_rot = dump_stats(cropped_traj_ref_sync, cropped_traj_est_sync)

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

        plot_paths = SimpleNamespace()
        if real_failures: plot_paths.live_slam_path = f"{post_path}/all.json" # Fetch the real live SLAM from all.json
        else: plot_paths.live_slam_path = f"{results_path}/aligned_live_slam.json" # Fetch what we generated with the graph
        plot_paths.post_slam_path = f"{post_path}/all.json"
        plot_paths.est_path = None
        plot_paths.opti_path = f"{post_path}/all.json"
        plot_paths.slam_path = f"{post_path}/all.json" # I belive this is unaligned post SLAM always

        if not args.no_plot:
            # Plot trajectories with MultiXR-Post
            plot_report["Live-SLAM"] = plot_trial(args.id, 
                    args.trial_name,
                    slam_stride = -1,
                    label_text = "Live-SLAM",
                    show_live_slam = True,
                    paths=plot_paths,
                    show=False)

        metric_report["Live-SLAM"].append(
            {
                "fail": interval,
                "ape_trans": crop_ape_trans,
                "ape_rot": crop_ape_rot,
                "rpe_trans": crop_rpe_trans,
                "rpe_rot": crop_rpe_rot,
            }
        )

    if not args.hide_plots:
        plt.tight_layout()
        plt.show()
    
    print(metric_report)

    return metric_report, plot_report


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("id", type=int)
    parser.add_argument("trial_name", help="Trial name")
    parser.add_argument("--no_run", action="store_true")
    parser.add_argument("--hide_plots", action="store_true")
    parser.add_argument("--no_plot", action="store_true")
    parser.add_argument("--anchor_selfloc_strategy", type=str, default="pre-loc")

    args = parser.parse_args()

    run_eval(args)