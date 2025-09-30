import argparse
import zipfile
import numpy as np
import os
import matplotlib.pyplot as plt

def extract_error_array(zip_path, extract_to):
    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        zip_ref.extractall(extract_to)

    for root, dirs, files in os.walk(extract_to):
        for file in files:
            if file == "error_array.npy":
                return np.load(os.path.join(root, file))
    raise FileNotFoundError("error_array.npy not found in extracted zip.")

def plot_cdf_subplot(ax, data, title, color, unit):
    sorted_data = np.sort(data)
    yvals = np.arange(1, len(sorted_data)+1) / float(len(sorted_data)) #TODO: Check that this formula is correct
    ax.plot(sorted_data, yvals, color=color)
    ax.set_title(title)
    ax.set_xlabel(f"APE Error ({unit})")
    ax.set_ylabel("Cumulative Probability")
    ax.grid(True)

def plot_ape_cdfs(dir_path, return_fig=False):
    """
    Generate APE error CDF plots for a given trial directory.
    
    Args:
        dir_path (str): Directory under `out_results` containing evo results.
        return_fig (bool): If True, returns the matplotlib Figure object.
    """
    base_dir = f"{dir_path}"
    temp_dir = "/tmp/evo_extracted"
    os.makedirs(temp_dir, exist_ok=True)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))  # 1 row, 2 columns

    zip_info = [
        ("Rotation APE (deg)", os.path.join(base_dir, "evo_ape_rotation_results.zip"), axes[0], "orange", "deg"),
        ("Translation APE (m)", os.path.join(base_dir, "evo_ape_translation_results.zip"), axes[1], "purple", "m")
    ]

    for title, zip_path, ax, color, unit in zip_info:
        if not os.path.exists(zip_path):
            print(f"[WARN] Missing: {zip_path}")
            continue
        try:
            data = extract_error_array(zip_path, os.path.join(temp_dir, title.replace(" ", "_")))
            plot_cdf_subplot(ax, data, title, color, unit)
        except Exception as e:
            print(f"[ERROR] Failed to process {title}: {e}")

    plt.suptitle(f"APE Error CDFs - {dir_path}", fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.95])

    if return_fig:
        return fig
    
def plot_cdf_subplot_joined(ax, data, title, color, unit, label=None):
    """
    Plot a single CDF subplot on the given axis.

    Args:
        ax (matplotlib.axes.Axes): The subplot axis to draw on.
        data (np.ndarray): Array of APE errors.
        title (str): Title of the subplot.
        color (str): Line color.
        unit (str): Unit of the APE errors for labeling.
        label (str, optional): Label for the line (for use in legends).
    """
    sorted_data = np.sort(data)
    yvals = np.arange(1, len(sorted_data) + 1) / float(len(sorted_data))  # CDF y-values

    ax.plot(sorted_data, yvals, color=color, label=label)
    ax.set_title(title)
    ax.set_xlabel(f"APE Error ({unit})")
    ax.set_ylabel("Cumulative Probability")
    ax.grid(True)


def plot_ape_cdfs_joined(dir_path_no_uwb, dir_path_uwb, return_fig=False):
    """
    Generate joined APE error CDF plots for two trials (e.g., with and without UWB).
    
    Args:
        dir_path_no_uwb (str): Directory under `out_results` for the baseline trial.
        dir_path_uwb (str): Directory under `out_results` for the UWB trial.
        return_fig (bool): If True, returns the matplotlib Figure object.
    """
    temp_dir = "/tmp/evo_extracted"
    os.makedirs(temp_dir, exist_ok=True)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))  # 1 row, 2 columns

    zip_info = [
        ("Rotation APE (deg)", "evo_ape_rotation_results.zip", axes[0], "deg"),
        ("Translation APE (m)", "evo_ape_translation_results.zip", axes[1], "m")
    ]
    
    trial_info = [
        ("No UWB", dir_path_no_uwb, "blue"),
        ("UWB", dir_path_uwb, "green")
    ]

    for title, zip_filename, ax, unit in zip_info:
        for label, base_dir, color in trial_info:
            zip_path = os.path.join(base_dir, zip_filename)
            if not os.path.exists(zip_path):
                print(f"[WARN] Missing: {zip_path}")
                continue
            try:
                sub_temp = os.path.join(temp_dir, f"{title.replace(' ', '_')}_{label.replace(' ', '_')}")
                data = extract_error_array(zip_path, sub_temp)
                plot_cdf_subplot_joined(ax, data, title, color, unit, label=label)
            except Exception as e:
                print(f"[ERROR] Failed to process {title} for {label}: {e}")

        ax.legend()

    plt.suptitle("APE Error CDFs - Comparison", fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.95])

    if return_fig:
        return fig



def main():
    parser = argparse.ArgumentParser(description="Plot CDF of APE errors from evo results.")
    parser.add_argument("dir", help="Synthetic trial directory, e.g., stereoi_circle2/synthetic_1_5")
    args = parser.parse_args()

    base_dir = f"/home/antond2/Desktop/Research/gtsam_test/out_results/{args.dir}"
    temp_dir = "/tmp/evo_extracted"
    os.makedirs(temp_dir, exist_ok=True)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))  # 1 row, 2 columns

    zip_info = [
        ("Rotation APE (deg)", os.path.join(base_dir, "evo_ape_rotation_results.zip"), axes[0], "orange", "deg"),
        ("Translation APE (m)", os.path.join(base_dir, "evo_ape_translation_results.zip"), axes[1], "purple", "m")
    ]

    for title, zip_path, ax, color, unit in zip_info:
        if not os.path.exists(zip_path):
            print(f"Missing: {zip_path}")
            continue
        try:
            data = extract_error_array(zip_path, os.path.join(temp_dir, title.replace(" ", "_")))
            plot_cdf_subplot(ax, data, title, color, unit)
        except Exception as e:
            print(f"Error processing {title}: {e}")

    plt.suptitle(f"APE Error CDFs - {args.dir}", fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()

if __name__ == "__main__":
    main()
