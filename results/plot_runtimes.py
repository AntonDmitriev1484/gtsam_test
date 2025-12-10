import re
import argparse
import matplotlib.pyplot as plt
import os
from collections import defaultdict


def parse_log(filepath):
    runtimes = defaultdict(lambda: {"timestamps": [], "elapsed": []})

    with open(filepath, 'r') as f:
        lines = f.readlines()

    i = 0
    while i < len(lines):
        line = lines[i].strip()

        # General pattern: TIMER | Start <Algorithm>, <Label>, ts=<timestamp> |
        m_start = re.match(r"TIMER \| Start (?P<algo>[A-Za-z0-9_]+), (?P<label>GT|SynthUWB|UWB), ts=(?P<ts>[0-9.]+) \|", line)
        if m_start:
            algo = m_start.group("algo")
            label = m_start.group("label")
            key = f"{algo}_{label}"
            try:
                ts = float(m_start.group("ts").strip('.'))
            except ValueError:
                i += 1
                continue

            # Look ahead for the 'TIMER Elapsed' line
            while i + 1 < len(lines):
                i += 1
                if lines[i].startswith("TIMER Elapsed"):
                    try:
                        elapsed = float(lines[i].strip().split()[-1])
                        runtimes[key]["timestamps"].append(ts)
                        runtimes[key]["elapsed"].append(elapsed)
                    except ValueError:
                        pass
                    break
        i += 1

    return runtimes


def plot_isam_runtimes(log_dir, title=None, show=False):
    log_path = os.path.join(log_dir, "log_dump.txt")
    runtimes = parse_log(log_path)

    if not runtimes:
        print("No runtime data to plot.")
        return None

    fig, ax = plt.subplots()
    for key, data in runtimes.items():
        if data["timestamps"]:
            ax.plot(data["timestamps"], data["elapsed"], label=key)

    ax.set_xlabel("Data Timestamp (s)")
    ax.set_ylabel("Elapsed Time (s)")
    ax.set_title(title or os.path.basename(log_path))
    ax.legend()
    ax.grid(True)
    plt.tight_layout()

    if show:
        plt.show()

    return fig


def main():
    parser = argparse.ArgumentParser(description="Plot timing logs for multiple algorithms.")
    parser.add_argument("dir", help="Synthetic trial directory, e.g., stereoi_circle2/synthetic_1_5")
    args = parser.parse_args()

    full_dir = f"/home/antond2/Desktop/Research/gtsam_test/results/out/{args.dir}"
    plot_isam_runtimes(full_dir, title=f"Runtime - {args.dir}", show=True)


if __name__ == "__main__":
    main()
