import re
import argparse
import matplotlib.pyplot as plt
import os

from plot_ape_ECDF import plot_ape_cdfs, plot_ape_cdfs_joined
# from plot_compare import plot_trajectory_comparison
from plot_single import plot_trajectories
from plot_runtimes import plot_isam_runtimes

def main():
    parser = argparse.ArgumentParser(description="Plot all")
    parser.add_argument("dir", help="Synthetic trial directory, e.g., stereoi_circle2/synthetic_1_5") # Dir, without uwb postfix
    parser.add_argument("--uwb", action='store_true')
    args = parser.parse_args()


    path = "/home/antond2/Desktop/Research/gtsam_test/out_results/"+args.dir
    uwb_path = path + "_uwb"

    if args.uwb:
        plot_ape_cdfs_joined(path, uwb_path)
        plot_trajectories(uwb_path, show=False)
        plot_isam_runtimes(uwb_path)
    else:
        plot_trajectories(path, show=False)
        plot_isam_runtimes(path)

    plt.show()

if __name__ == "__main__":
    main()

