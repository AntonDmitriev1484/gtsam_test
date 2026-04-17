#!/usr/bin/env python3

import sys
import os
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

def load_t_slam_world(json_path):
    with open(json_path, 'r') as f:
        data = json.load(f)

    T = np.array(data["T_slam_world"])
    if T.shape != (4, 4):
        raise ValueError("T_slam_world must be a 4x4 matrix")
    return T

def pose_to_htm(tx, ty, tz, qx, qy, qz, qw):
    t = np.array([tx, ty, tz])
    r = R.from_quat([qx, qy, qz, qw])
    T = np.eye(4)
    T[:3, :3] = r.as_matrix()
    T[:3, 3] = t
    return T

def htm_to_pose(T):
    t = T[:3, 3]
    r = R.from_matrix(T[:3, :3])
    q = r.as_quat()
    return t.tolist(), q.tolist()

def main(trial_name, input_file, output_file):
    input_base = f"/home/antond2/ws/post/out/{trial_name}_post"
    json_path = os.path.join(input_base, "transforms.json")
    # tum_path = os.path.join(input_base, "slam_data_world_frame_tum_interp.txt")
    tum_path = input_file

    output_path = output_file

    T_slam_world = load_t_slam_world(json_path)

    with open(tum_path, 'r') as f_in, open(output_path, 'w') as f_out:
        for line in f_in:
            if not line.strip():
                continue
            parts = line.strip().split()
            if len(parts) != 8:
                print(f"Skipping malformed line: {line}")
                continue

            timestamp, tx, ty, tz, qx, qy, qz, qw = parts
            T_pose = pose_to_htm(float(tx), float(ty), float(tz),
                                 float(qx), float(qy), float(qz), float(qw))

            T_transformed = T_slam_world @ T_pose
            
            # Both wrong:
            # T_transformed =  np.linalg.inv(T_slam_world) @ T_pose
            # T_transformed = T_pose @ T_slam_world
            t_new, q_new = htm_to_pose(T_transformed)

            f_out.write(f"{timestamp} {t_new[0]} {t_new[1]} {t_new[2]} "
                        f"{q_new[0]} {q_new[1]} {q_new[2]} {q_new[3]}\n")

    print(f"Transformed poses saved to: {output_path}")

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python3 transform_tum_poses.py <trial_name> <input_file> <output_dir>")
        sys.exit(1)
    main(sys.argv[1], sys.argv[2], sys.argv[3])
