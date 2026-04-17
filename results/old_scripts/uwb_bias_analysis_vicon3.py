import re
import argparse
import os
from collections import defaultdict
import numpy as np
import json
import copy

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def plot_uwb_metrics(data, anchor, time_markers = [], show=False, minline=None):
    timestamps = data["t_"]

    fig, axs = plt.subplots(4, 1, figsize=(10, 8), sharex=True)

    print(data.keys())

    # Plot 1: Ranges
    axs[0].plot(timestamps, data["synth_range_"], label="Synthetic Range", color="green")
    axs[0].plot(timestamps, data["corrected_range_"], label="Corrected Range", color="blue")
    axs[0].plot(timestamps, data["range_"], label="Real Range", color="red")


    rmse_corrected_real = np.sqrt(np.mean((data["corrected_range_"]- data["range_"]) ** 2))
    rmse_corrected_synth = np.sqrt(np.mean((data["corrected_range_"] - data["synth_range_"]) ** 2))
    rmse_real_synth = np.sqrt(np.mean((data["range_"] - data["synth_range_"]) ** 2))

    axs[0].set_ylabel("Range (m)")
    axs[0].set_ylim((0,15))
    axs[0].set_title(
        f"Anchor {anchor} Ranges\n"
        f"RMSE(Corrected vs Real): {rmse_corrected_real:.3f} m | "
        f"RMSE(Corrected vs Synthetic): {rmse_corrected_synth:.3f} m | "
        f"RMSE(Real vs Synthetic): {rmse_real_synth:.3f} m "
    )
    axs[0].legend()
    axs[0].grid(True)
    if minline is not None:
        axs[0].axvline(x=minline, color="pink")

    axs[1].plot(timestamps, data["range_"] - data["synth_range_"], label="Ranging Error", color="green")
    axs[1].set_ylabel("Range Error (m)")
    axs[1].grid(True)

    # Plot 2: NLOS
    axs[2].plot(timestamps, data["nlos_"], label="NLOS Score", color="black")
    axs[2].set_ylabel("NLOS Score")
    axs[2].axhline(y=10, color='red', linestyle='--')
    axs[2].axhline(y=6, color='green', linestyle='--')
    axs[2].legend()
    axs[2].grid(True)

    # Plot 3: SNR
    axs[3].plot(timestamps, data["snr_"], label="SNR", color="purple")
    axs[3].set_xlabel("Time (s)")
    axs[3].set_ylabel("SNR")
    axs[3].legend()
    axs[3].grid(True)

    for i in range(3):
        for t in time_markers:
            axs[i].axvline(x=t)

    plt.tight_layout()

    # if show:
    #     plt.show()

    return fig

def plot_error_over_distance(data_by_anchor):
    fig, axs = plt.subplots(1, 1, figsize=(10, 8))
    # Plot 1: How does error increase with distance?

    all_gt_range_to_error = []
    for anchor, anchor_data in data_by_anchor.items():

        gt_range_to_error = np.zeros((len(anchor_data["synth_range"]), 2))
        print(gt_range_to_error.shape)
        gt_range_to_error[:,0] = np.array(anchor_data["synth_range"])
        gt_range_to_error[:,1] = np.abs(np.array(anchor_data["synth_range"]) - np.array(anchor_data["range"]))

        # gt_range_to_error = all_gt_range_to_error[np.argsort(gt_range_to_error[:, 0])]
        all_gt_range_to_error += list(gt_range_to_error)

    all_gt_range_to_error = np.array(all_gt_range_to_error)
    all_gt_range_to_error = all_gt_range_to_error[np.argsort(all_gt_range_to_error[:, 0])]

    axs.scatter(all_gt_range_to_error[:,0], all_gt_range_to_error[:,1])
    axs.set_title("Suspected Range Error (m) vs GT Distance (m)")
    axs.set_ylabel("Range Error (m)")
    axs.set_xlabel("GT Measurement Distance (m)")
    plt.show()

def parse_all_json(filepath):
    data_by_anchor = defaultdict(lambda: {
        "t_": [],
        "range_": [],
        "maxnoise_": [],
        "firstpathamp1_": [],
        "firstpathamp2_": [],
        "firstpathamp3_": [],
        "stdnoise_": [],
        "maxgrowthcir_": [],
        "rxpreamcount_": [],
        "firstpath_": [],
        "T_body_world_": []
    })

    for k in [3]: _ = data_by_anchor[k]  # triggers the lambda to create the dict
        
    with open(filepath, 'r') as fs:
        all_mes = json.load(fs)
        for mes in all_mes:
            for k,v in mes.items():
                if mes["type"] == "assisted_uwb":
                    if k+"_" in data_by_anchor[mes["id"]]:
                        id = mes["id"]

                        # NOTE: Only for irl3_loops!
                        # if id == 4: id =2
                        # elif id == 2: id =4

                        data_by_anchor[id][k+"_"].append(v)

    return data_by_anchor

def parse_anchors(filepath):
    with open(filepath, 'r') as fs:
        anchors = json.load(fs)
    return anchors

def moving_average(x, window_size):
    window = np.ones(window_size) / window_size
    return np.convolve(x, window, mode='same')

def compute_synthetic_ranges(data_by_anchor, anchors, plot_stride=20):
    synthetic_data_by_anchor = data_by_anchor.copy()

    T_body_to_imu = np.array([
        [1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, -1, 0, 0],
        [0, 0, 0, 1]
    ])

    T_decawave_to_body = np.eye(4)
    # T_body_to_decawave[:3, 3] = np.array([-0.045, -0.15, -0.025])
    # T_body_to_decawave[:3, 3] = np.array([-0.12, 0.015, -0.1])
    t_body_to_decawave = np.array([-0.0725, 0.061, -0.03]) # in body frame
    T_decawave_to_body[:3,3] = t_body_to_decawave


    if plot_stride is not None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title("T_world_to_body Coordinate Frames and Trajectory")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_xlim([0, 10])
        ax.set_ylim([0, 10])
        ax.set_zlim([0, 3])
        trajectory_points = []

        # Plot anchor locations as red points
        for anchor in anchors:
            pos = np.array(anchor['position'])
            ax.scatter(pos[0], pos[1], pos[2], c='r', marker='o', label=f"Anchor {anchor['ID']}")
        ax.legend()
    else:
        ax = None
        trajectory_points = []

    for anchor, data in data_by_anchor.items():
        synthetic_data_by_anchor[anchor]["synth_range_"] = []

        anchor_loc_world = np.array([a for a in anchors if int(a['ID']) == anchor][0]['position'])

        for i in range(len(data["t_"])):
            T_world_to_body = np.array(data["T_body_world_"][i], dtype=np.float64)
            T_world_to_decawave = np.linalg.inv(T_decawave_to_body) @ T_world_to_body 
            decawave_loc_world = np.linalg.inv(T_world_to_decawave)[:3, 3]
            synth_range = np.linalg.norm(decawave_loc_world - anchor_loc_world)

            synthetic_data_by_anchor[anchor]["synth_range_"].append(synth_range)

            if ax is not None:
                origin = T_world_to_body[:3, 3]
                trajectory_points.append(origin)
                if i % plot_stride == 0:
                    plot_T_world_to_body_frame(ax, T_world_to_body)

    if ax is not None and len(trajectory_points) > 0:
        trajectory_points = np.array(trajectory_points)
        ax.plot(
            trajectory_points[:, 0],
            trajectory_points[:, 1],
            trajectory_points[:, 2],
            'k-', linewidth=2, label="Trajectory"
        )
        ax.legend()
        ax.invert_xaxis()
        ax.set_box_aspect([1, 1, 1])
        plt.show()

    return synthetic_data_by_anchor



def plot_T_world_to_body_frame(ax, T_world_to_body, axis_len=0.1):
    origin = T_world_to_body[:3, 3]
    R = T_world_to_body[:3, :3]
    ax.quiver(*origin, *(R[:, 0] * axis_len), color='r')  # X axis
    ax.quiver(*origin, *(R[:, 1] * axis_len), color='g')  # Y axis
    ax.quiver(*origin, *(R[:, 2] * axis_len), color='b')  # Z axis



def compute_corrected_ranges(data_by_anchor):
    for anchor, data in data_by_anchor.items():

        data_by_anchor[anchor]["corrected_range_"] = []
        data_by_anchor[anchor]["nlos_"] = []
        data_by_anchor[anchor]["snr_"] = []

        A = 121.74

        fp_power_ = 10 * np.log10( ((data["firstpathamp1_"] ** 2) + (data["firstpathamp2_"]**2) + (data["firstpathamp3_"]**2)) 
                                  / (data["rxpreamcount_"]**2) ) - A
        rx_power_ = 10 * np.log10( data["maxgrowthcir_"] * (2**17) / (data["rxpreamcount_"] ** 2)) - A

        # Just going to pre-compute these elementwise.
        data["nlos_"] = rx_power_ - fp_power_
        data["snr_"] = data["firstpathamp1_"] / data["maxnoise_"]
        # Assume we're taking a moving average to filter noise in these metrics
        data["nlos_"] = moving_average(data["nlos_"], 15)
        data["snr_"] = moving_average(data["snr_"], 15)

        max_delta_range = 1 / 5 # 1 m/s / 5 Hz is the max we assume we can move between two ranges
        
        # Filter the extreme outliers to be within the maximum displacement I could possibly have between two ranges
        filtered_ranges = []
        prev_range = data["range_"][0]
        filtered_ranges.append(prev_range)
        for i in range(1, data["range_"].shape[0]):
            current_range = data["range_"][i]
            corrected_range = prev_range + max_delta_range
            if (current_range - prev_range) > max_delta_range:
                filtered_ranges.append(corrected_range)
            else:
                filtered_ranges.append(current_range)
            prev_range = corrected_range if (current_range - prev_range) > (corrected_range - prev_range) else current_range

        data["corrected_range_"] = np.array(filtered_ranges)
        # Also, correct each range proportional to its NLoS score!

        nlos_max = 12
        nlos_min = 2
        helmet_bias = 0.2
        static_bias = 0.06 # Getting around just 5cm error standing LoS to antenna, this could be due to inherent UWB bias
        data["corrected_range_"] = data["corrected_range_"] - ( (helmet_bias + static_bias) * ((data["nlos_"] - nlos_min) / (nlos_max-nlos_min)))

        data["corrected_range_"] = moving_average(data["corrected_range_"], window_size=5)

    return data_by_anchor

def main():
    parser = argparse.ArgumentParser(description="Parse UWB logs and plot range/SNR/NLOS per anchor.")
    # parser.add_argument("dir", help="Synthetic trial directory, e.g., stereoi_circle2/synthetic_1_5")
    parser.add_argument("trial_name", help="Trial name")
    args = parser.parse_args()

    data_path = f"/home/antond2/ws/post/out/{args.trial_name}_post/all.json"
    anchor_path = f"/home/antond2/ws/post/out/{args.trial_name}_post/anchors.json"


    ALIGN = True

    data_by_anchor = parse_all_json(data_path)
    anchors = parse_anchors(anchor_path)
    data_by_anchor = compute_synthetic_ranges(data_by_anchor, anchors, 10)

    if args.trial_name == 'irl3_loops':
        # print('innnnn')
        # # Because vicon is sillyhead
        # temp = data_by_anchor[4].copy()
        # data_by_anchor[4] = data_by_anchor[2].copy()
        # data_by_anchor[2] = temp

        data_by_anchor[2], data_by_anchor[4] = data_by_anchor[4], data_by_anchor[2]

    # convert all data to nparrays
    for anchor, data in data_by_anchor.items():
        for key in data:
            if key != "T_body_world_":
                data[key] = np.array(data[key])

    data_by_anchor = compute_corrected_ranges(data_by_anchor)

    for anchor, data in data_by_anchor.items():

        # Taking RMSE over the whole trajectory, instead of separating parts out
        print(f" Anchor {anchor}")
        print(" Real vs Ground Truth (m):")
        rmse =  np.sqrt(np.mean((data["range_"]-data["synth_range_"]) ** 2))
        print(f" - RMSE {rmse}")

        print(f" Anchor {anchor}")
        print(" Corrected vs Ground Truth (m):")
        rmse =  np.sqrt(np.mean((data["corrected_range_"]-data["synth_range_"]) ** 2))
        print(f" - RMSE {rmse}")


    # Compute and print RMSE for each part individually, to get a sense of bias.

    # Plot trajectory stats
    for anchor, anchor_data in data_by_anchor.items():
        fig = plot_uwb_metrics(anchor_data, anchor=anchor, show=True, minline=data["t_"][np.argmin(data["range_"])])
    
    plt.show()


if __name__ == "__main__":
    main()
