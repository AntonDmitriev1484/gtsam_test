import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load your JSON data from a file or string
with open("/home/antond2/ws/post/out/pilot4_loopy_post/synthetic/all_synthetic_1_0.json", "r") as f:
    data = json.load(f)


# Extract XYZ positions from T_body_world
positions = []
for d in data:
    if d["type"] == "assisted_uwb":
        if "T_body_world" in d:
            T = np.array(d["T_body_world"], dtype=np.float64)
            xyz = T[:3, 3]
            positions.append(xyz)

positions = np.array(positions)

# Plotting
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
# ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b.-', label='Trajectory')
ax.scatter(positions[:,0], positions[:, 1] , positions[:, 2], 'b.-', label='Trajectory')
ax.set_title("3D Trajectory from JSON T_body_world")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend()
ax.grid(True)

# Save the figure
plt.tight_layout()
plt.savefig("trajectory_plot.png", dpi=300)
plt.show()

