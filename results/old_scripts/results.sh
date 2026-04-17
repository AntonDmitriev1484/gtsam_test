#!/bin/bash

# Usage: ./script.sh <trial_name> <synthetic_path>

# Use to fully compare a UWB trial to non-UWB trial

#synthetic_path ex. stereoi_circle2/synthetic_1_5 <- leave off uwb postfix to compare

# Check for correct number of arguments
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <trial_name> <synthetic_path> <uwb_noise>"
    exit 1
fi
# Assign arguments to variables
trial_name="$1"
synthetic_path="$2"
uwb_noise="$3" # Just used for record keeping.
safe_path="${synthetic_path//\//_}"

# Let synthetic path be without _uwb

# Run metric evaluations
echo "NO UWB"
./eval_trajectory.sh "${trial_name}" "${synthetic_path}" "${uwb_noise}"
python3 superplot.py "${synthetic_path}"

echo "UWB"
./eval_trajectory.sh "${trial_name}" "${synthetic_path}_uwb" "${uwb_noise}"
python3 superplot.py "${synthetic_path}" --uwb

# # Compare trajectories
# python3 plot_compare.py --synthetic "${synthetic_path}"

# # Compare optimizer runtimes
# python3 plot_runtimes.py "${synthetic_path}" # Why does it not show anything without _uwb?
# python3 plot_runtimes.py "${synthetic_path}_uwb"

