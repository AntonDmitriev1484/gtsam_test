import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from types import SimpleNamespace

def plot_transform(ax, T, label, length=0.2):
    """Plot a coordinate frame given a 4x4 transformation matrix."""

    # Doing T x y_vec_world, would express a point in the world frame in the tag frame
    # We want to express the tag frame axes (as points) in the world frame for plotting
    #, So we do T^-1 x y_vec_tag
    
    origin = (np.linalg.inv(T) @ np.array([0,0,0,1]))[:3]
    x_axis = (np.linalg.inv(T) @ np.array([1,0,0,1]))[:3]
    y_axis = (np.linalg.inv(T) @ np.array([0,1,0,1]))[:3]
    z_axis = (np.linalg.inv(T) @ np.array([0,0,1,1]))[:3]

    # origin = T[:3,3]


    ax.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]], 'r')
    ax.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]], 'g')
    ax.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]], 'b')
    ax.text(origin[0], origin[1], origin[2], label)

# --- Define all transforms ---
Transforms = SimpleNamespace()

Transforms.origin = np.eye(4)

# README: Comment in whichever block of code you'd like to see the transforms for.

# -> My default approach
# t_world_to_body_in_world = np.array([4,5,3])
# R_world_to_body = np.array([[0,1,0], 
#                             [0,0,-1], 
#                             [-1,0,0]])
# Transforms.world_to_body = np.eye(4)
# Transforms.world_to_body[:3,:3] = R_world_to_body
# Transforms.world_to_body[:3,3] = t_world_to_body_in_world


# t_world_to_tag_in_world = np.array([0,5,3])
# R_world_to_tag = R_world_to_body
# Transforms.world_to_tag = np.eye(4)
# Transforms.world_to_tag[:3,:3] = R_world_to_tag
# Transforms.world_to_tag[:3,3] = t_world_to_tag_in_world

# -> Approach in the follow up email I sent
# t_world_to_body_in_world = np.array([4,5,3])
# R_world_to_body = np.array([[0,1,0], 
#                             [0,0,-1], 
#                             [-1,0,0]])
# t_body_to_world_in_body = -R_world_to_body @ t_world_to_body_in_world
# Transforms.world_to_body = np.eye(4)
# Transforms.world_to_body[:3,:3] = R_world_to_body
# Transforms.world_to_body[:3,3] = t_body_to_world_in_body


# t_world_to_tag_in_world = np.array([0,5,3])
# R_world_to_tag = R_world_to_body
# t_tag_to_world_in_tag = -R_world_to_tag @ t_world_to_tag_in_world
# Transforms.world_to_tag = np.eye(4)
# Transforms.world_to_tag[:3,:3] = R_world_to_tag
# Transforms.world_to_tag[:3,3] = t_tag_to_world_in_tag

# -> The working approach?
t_world_to_body_in_world = np.array([4,5,3])
R_world_to_body = np.linalg.inv(np.array([[0,1,0], 
                            [0,0,-1], 
                            [-1,0,0]]))
Transforms.world_to_body = np.eye(4)
Transforms.world_to_body[:3,:3] = R_world_to_body
Transforms.world_to_body[:3,3] = t_world_to_body_in_world


t_world_to_tag_in_world = np.array([0,5,3])
R_world_to_tag = R_world_to_body
Transforms.world_to_tag = np.eye(4)
Transforms.world_to_tag[:3,:3] = R_world_to_tag
Transforms.world_to_tag[:3,3] = t_world_to_tag_in_world


# # -> What if we measure the pose of the world origin in the apriltag

# # t_world_to_body_in_world = np.array([4,5,3])
# # t_body_to_world_in_body = np.array([-5,3,4])

# R_world_to_body = np.array([[0,1,0], 
#                             [0,0,-1], 
#                             [-1,0,0]])
# # Transforms.world_to_body = np.eye(4)
# # Transforms.world_to_body[:3,:3] = R_world_to_body
# # Transforms.world_to_body[:3,3] = t_body_to_world_in_body


# t_world_to_tag_in_world = np.array([0,5,3])
# t_tag_to_world_in_tag = np.array([-5,3,0]) # Suppose we measure the coordinates of the origin in the tag frame.


# R_world_to_tag = R_world_to_body
# Transforms.world_to_tag = np.eye(4)
# Transforms.world_to_tag[:3,:3] = R_world_to_tag
# Transforms.world_to_tag[:3,3] = t_tag_to_world_in_tag



# --- Plotting ---
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Transforms in World Frame")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

for name, T in Transforms.__dict__.items():
    plot_transform(ax, T, name)

ax.set_box_aspect([1, 1, 1])
plt.tight_layout()
plt.show()
