#!/bin/bash

# Usage: ./script.sh <trial_name> <synthetic_path>

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

# python3 "/home/antond2/SE3_Pose_Interp/pose_interp.py" --kf_pose "/home/antond2/ws/post/out/${trial_name}_post/slam_20Hz.txt" \
#     --timestamps "$(realpath -m "$synthetic_path")/est_timestamps.txt"

# Take the original SLAM frame trajectory, convert from ms to s.
python3 "/home/antond2/ws/orbslam/convert_to_s.py" "/home/antond2/ws/orbslam/out/${trial_name}_cam_traj.txt" \
        "/home/antond2/ws/orbslam/out/${trial_name}_cam_traj_seconds.txt"

# Feed trajectory to SE3 interpolator
python3 "/home/antond2/SE3_Pose_Interp/pose_interp.py" --kf_pose "/home/antond2/ws/orbslam/out/${trial_name}_cam_traj_seconds.txt" \
    --timestamps "$(realpath -m "$synthetic_path")/est_timestamps.txt"

# Read T_slam_world transform and interpolation from ws dataset directory
# Specify inpath for trajectory, and outpath for trajectory
# python3 unstupidtrajectory.py ${trial_name} "/home/antond2/ws/post/out/${trial_name}_post/slam_20Hz_interp.txt" "$(realpath -m "$synthetic_path")/slam_200Hz.txt"

# mv "/home/antond2/ws/post/out/${trial_name}_post/slam_20Hz_interp.txt" "$(realpath -m "$synthetic_path")/slam_interp.txt"

# Move back to its test case in the out_results directory
mv "/home/antond2/ws/orbslam/out/${trial_name}_cam_traj_seconds.txt" "$(realpath -m "$synthetic_path")/slam_interp.txt"

# Run evo metrics
plot_evo=false

result_dir="/home/antond2/Desktop/Research/gtsam_test/save_results/${trial_name}"
mkdir -p "$result_dir"

if $plot_evo; then
{
    # Translation (m)
    evo_ape "tum" "$(realpath -m "$synthetic_path")/slam_interp.txt" "$(realpath -m "$synthetic_path")/est.txt" \
        -a --pose_relation "trans_part" --save_results "$(realpath -m "$synthetic_path")/evo_ape_translation_results.zip" --plot_mode "xyz" --plot --no_warnings
    evo_rpe "tum" "$(realpath -m "$synthetic_path")/slam_interp.txt" "$(realpath -m "$synthetic_path")/est.txt" \
        -a --pose_relation "trans_part" --save_results "$(realpath -m "$synthetic_path")/evo_rpe_translation_results.zip" --plot_mode "xyz" --plot --no_warnings
    # Rotation (deg)
    evo_ape "tum" "$(realpath -m "$synthetic_path")/slam_interp.txt" "$(realpath -m "$synthetic_path")/est.txt" \
        -a --pose_relation "angle_deg" --save_results "$(realpath -m "$synthetic_path")/evo_ape_rotation_results.zip" --plot_mode "xyz" --plot --no_warnings
    evo_rpe "tum" "$(realpath -m "$synthetic_path")/slam_interp.txt" "$(realpath -m "$synthetic_path")/est.txt" \
        -a --pose_relation "angle_deg" --save_results "$(realpath -m "$synthetic_path")/evo_rpe_rotation_results.zip" --plot_mode "xyz" --plot --no_warnings
}   2>&1 | tee "${result_dir}/results_${safe_path}_${uwb_noise}.txt"
else
{
    # Translation (m)
    evo_ape "tum" "$(realpath -m "$synthetic_path")/slam_interp.txt" "$(realpath -m "$synthetic_path")/est.txt" \
        -a --pose_relation "trans_part" --save_results "$(realpath -m "$synthetic_path")/evo_ape_translation_results.zip" --no_warnings
    evo_rpe "tum" "$(realpath -m "$synthetic_path")/slam_interp.txt" "$(realpath -m "$synthetic_path")/est.txt" \
        -a --pose_relation "trans_part" --save_results "$(realpath -m "$synthetic_path")/evo_rpe_translation_results.zip" --no_warnings
    # Rotation (deg)
    evo_ape "tum" "$(realpath -m "$synthetic_path")/slam_interp.txt" "$(realpath -m "$synthetic_path")/est.txt" \
        -a --pose_relation "angle_deg" --save_results "$(realpath -m "$synthetic_path")/evo_ape_rotation_results.zip" --no_warnings
    evo_rpe "tum" "$(realpath -m "$synthetic_path")/slam_interp.txt" "$(realpath -m "$synthetic_path")/est.txt" \
        -a --pose_relation "angle_deg" --save_results "$(realpath -m "$synthetic_path")/evo_rpe_rotation_results.zip" --no_warnings

}   2>&1 | tee "${result_dir}/results_${safe_path}_${uwb_noise}.txt"
fi