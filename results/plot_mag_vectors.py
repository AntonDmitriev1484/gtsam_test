import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def read_htms(filepath):
    """
    Reads a file where each line is a flattened 3x4 HTM.
    Returns a list of 4x4 matrices.
    """
    poses = []
    with open(filepath, 'r') as f:
        for line in f:
            nums = list(map(float, line.strip().split()))
            if len(nums) == 12:
                mat = np.array(nums).reshape(3, 4)
                htm = np.vstack([mat, [0, 0, 0, 1]])
                poses.append(htm)
            elif len(nums) == 16:
                htm = np.array(nums).reshape(4, 4)
                poses.append(htm)
            else:
                raise ValueError(f"Unexpected number of values in line: {len(nums)}")
    return poses


def draw_axes(ax, T, length=0.3):
    """Draws a coordinate frame at transform T."""
    origin = T[:3, 3]
    x_axis = T[:3, 0] * length
    y_axis = T[:3, 1] * length
    z_axis = T[:3, 2] * length
    ax.quiver(*origin, *x_axis, color='r', length=length, linewidth=0.2, normalize=False)
    ax.quiver(*origin, *y_axis, color='g', length=length, linewidth=0.2, normalize=False)
    ax.quiver(*origin, *z_axis, color='b', length=length, linewidth=0.2,normalize=False)


def main():
    suwb_poses = read_htms("./stereoi_circle2/synthetic_1_5_uwb/suwb_base_poses.txt")
    mag_vectors = read_htms("./stereoi_circle2/synthetic_1_5_uwb/mag_vectors_fs.txt")

    if len(suwb_poses) != len(mag_vectors):
        raise ValueError("Mismatch: suwb_poses and mag_vectors must have same number of lines")

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for T_body_world, mag_vec_world in zip(suwb_poses, mag_vectors):
        draw_axes(ax, T_body_world)

        # I really don't understand why this is always wrong
        # p_vec_world = np.linalg.inv(T_body_world) @ mag_vec_body
        # p_vec_world = mag_vec_world

        origin = T_body_world[:3, 3]
        mag_vec = mag_vec_world[:3, 3]  # Assumes magnetic vector is in translation

        # draw_axes(ax, p_vec_world)
        ax.quiver(*origin, *mag_vec, color='orange', length=0.2, linewidth=0.4)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim3d(-6,-2)
    ax.set_ylim3d(1,5)
    ax.set_zlim3d(0,2)
    ax.set_title("SUWB Poses and Magnetic Vectors")
    ax.set_box_aspect([1, 1, 1])
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
