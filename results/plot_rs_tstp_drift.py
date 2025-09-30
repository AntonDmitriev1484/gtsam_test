import numpy as np
import matplotlib.pyplot as plt

import json

fs= open('/home/antond2/ws/post/out/uwb_calibration_loops_post/all.json', 'r')

data = json.load(fs)


rs_tstp = []
uwb_tstp = []
for mes in data:
    if "uwb" not in mes["type"]: rs_tstp.append(mes["t"])
    else: uwb_tstp.append(mes["t"])

bag_start, bag_end = (1754404365.701453725, 1754404585.520203648)

# Plot
plt.figure(figsize=(10, 4))
plt.scatter(rs_tstp, [1]*len(rs_tstp), marker='.', color='blue', label="rs_tstp events")
plt.scatter(uwb_tstp, [2]*len(uwb_tstp), marker='.', color='red', label="uwb_tstp events")

# Vertical lines
plt.axvline(bag_start, color='green', linestyle='--', label='bag_start')
plt.axvline(bag_end, color='red', linestyle='--', label='bag_end')

plt.xlabel("Timestamp")
plt.yticks([])  # Hide y-axis
plt.title("rs_tstp scatter with bag boundaries")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

