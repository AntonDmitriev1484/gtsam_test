import re
import argparse
import matplotlib.pyplot as plt
import os
from collections import defaultdict
import numpy as np

def parse_uwb_log(filepath):
    data_by_anchor = defaultdict(lambda: {
        "timestamps": [],
        "synthetic": [],
        "real": [],
        "corrected": [],
        "nlos": [],
        "snr": [],
    })

    current_time = None
    current_anchor = None

    with open(filepath, 'r') as f:
        lines = f.readlines()

    for line in lines:
        line = line.strip()

        # Time of the UWB measurement
        m_time = re.match(r"Processing assisted range for t=([0-9.]+)", line)
        if m_time:
            current_time = float(m_time.group(1))
            continue

        # Match anchor ID prefix (e.g., UWB X.)
        m_anchor_prefix = re.match(r"UWB (\w+)\. Synthetic Range", line)
        if m_anchor_prefix:
            current_anchor = m_anchor_prefix.group(1)

        # Ranges with anchor prefix
        m_range = re.match(r"UWB (\w+)\. Synthetic Range ([0-9.e+-]+), Real Range ([0-9.e+-]+), Corrected Range ([0-9.e+-]+)", line)
        if m_range and current_time is not None:
            anchor = m_range.group(1)
            data_by_anchor[anchor]["timestamps"].append(current_time)
            data_by_anchor[anchor]["synthetic"].append(float(m_range.group(2)))
            data_by_anchor[anchor]["real"].append(float(m_range.group(3)))
            data_by_anchor[anchor]["corrected"].append(float(m_range.group(4)))
            continue

        # NLOS and SNR with anchor prefix
        m_nlos = re.match(r"UWB (\w+)\. NLOS ([0-9.e+-]+), SNR ([0-9.e+-]+)", line)
        if m_nlos and current_time is not None:
            anchor = m_nlos.group(1)
            data_by_anchor[anchor]["nlos"].append(float(m_nlos.group(2)))
            data_by_anchor[anchor]["snr"].append(float(m_nlos.group(3)))
            continue

    return data_by_anchor


def plot_uwb_metrics(data, anchor, title_prefix="", show=False):
    timestamps = data["timestamps"]

    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    # Plot 1: Ranges
    axs[0].plot(timestamps, data["synthetic"], label="GT Range", color="green")
    axs[0].plot(timestamps, data["real"], label="Real Range", color="red")
    # axs[0].plot(timestamps, data["corrected"], label="Corrected Range", color="blue")

    # rmse_corrected_real = np.sqrt(np.mean((np.array(data["corrected"]) - np.array(data["real"])) ** 2))
    # rmse_corrected_synth = np.sqrt(np.mean((np.array(data["corrected"]) - np.array(data["synthetic"])) ** 2))
    rmse_real_synth = np.sqrt(np.mean((np.array(data["real"]) - np.array(data["synthetic"])) ** 2))

    axs[0].set_ylabel("Range (m)")
    axs[0].set_title(
        f"{title_prefix} Anchor {anchor} Ranges\n"
        # f"RMSE(Corrected vs Real): {rmse_corrected_real:.3f} m | "
        # f"RMSE(Corrected vs Ground Truth): {rmse_corrected_synth:.3f} m | "
        f"RMSE(Real vs Ground Truth): {rmse_real_synth:.3f} m "
    )
    axs[0].legend()
    axs[0].grid(True)

    # Plot 2: NLOS
    axs[1].plot(timestamps, data["nlos"], label="NLOS Score", color="black")
    axs[1].set_ylabel("NLOS Score")
    axs[1].axhline(y=10, color='red', linestyle='--')
    axs[1].axhline(y=6, color='green', linestyle='--')
    axs[1].legend()
    axs[1].grid(True)

    # Plot 3: SNR
    axs[2].plot(timestamps, data["snr"], label="SNR", color="purple")
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel("SNR")
    axs[2].legend()
    axs[2].grid(True)

    # for i in range(3):
    #     axs[i].axvline(x=1753905418.4158046)

    plt.tight_layout()

    # if show:
    #     plt.show()

    return fig

def plot_error_over_distance(data_by_anchor):
    fig, axs = plt.subplots(1, 1, figsize=(10, 8))
    # Plot 1: How does error increase with distance?

    all_gt_range_to_error = []
    for anchor, anchor_data in data_by_anchor.items():

        gt_range_to_error = np.zeros((len(anchor_data["synthetic"]), 2))
        print(gt_range_to_error.shape)
        gt_range_to_error[:,0] = np.array(anchor_data["synthetic"])
        gt_range_to_error[:,1] = np.abs(np.array(anchor_data["synthetic"]) - np.array(anchor_data["real"]))

        # gt_range_to_error = all_gt_range_to_error[np.argsort(gt_range_to_error[:, 0])]
        all_gt_range_to_error += list(gt_range_to_error)

    all_gt_range_to_error = np.array(all_gt_range_to_error)
    all_gt_range_to_error = all_gt_range_to_error[np.argsort(all_gt_range_to_error[:, 0])]

    axs.scatter(all_gt_range_to_error[:,0], all_gt_range_to_error[:,1])
    axs.set_title("Suspected Range Error (m) vs GT Distance (m)")
    axs.set_ylabel("Range Error (m)")
    axs.set_xlabel("GT Measurement Distance (m)")
    plt.show()

def main():
    parser = argparse.ArgumentParser(description="Parse UWB logs and plot range/SNR/NLOS per anchor.")
    parser.add_argument("dir", help="Synthetic trial directory, e.g., stereoi_circle2/synthetic_1_5")
    args = parser.parse_args()

    full_log_path = f"/home/antond2/Desktop/Research/gtsam_test/out_results/{args.dir}/log_dump.txt"

    if not os.path.exists(full_log_path):
        print(f"Log file not found: {full_log_path}")
        return

    data_by_anchor = parse_uwb_log(full_log_path)

    # Plot trajectory stats
    for anchor, anchor_data in data_by_anchor.items():
        print(f"Generating plot for anchor {anchor} with {len(anchor_data['timestamps'])} entries")
        fig = plot_uwb_metrics(anchor_data, anchor=anchor, title_prefix=args.dir, show=True)
    # plt.show()

    # plot_error_over_distance(data_by_anchor)

    plt.show()


if __name__ == "__main__":
    main()
