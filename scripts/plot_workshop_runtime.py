import matplotlib.pyplot as plt
import numpy as np
import tikzplotlib

# Data
num_objects = np.array([3, 5, 7, 10])
capture_AP = np.array([0.99, 0.99, 0.99, 0.98])
success_AP = np.array([1, 0.99, 0.99, 1])
runtime_mean = np.array([0.37, 0.66, 0.81, 1.2])
runtime_std = np.array([0.13, 0.39, 0.39, 0.46])

# Plotting
fig, ax1 = plt.subplots()

# Left y-axis (AP values)
ax1.plot(num_objects, capture_AP, label='Capture AP', marker='o', color='blue')
ax1.plot(num_objects, success_AP, label='Success AP', marker='x', color='green')
ax1.set_xlabel('# Objects')
ax1.set_ylabel('Average Precision (AP)')
ax1.set_ylim([0.8, 1.02])
ax1.set_xticks([3, 5, 7, 10])
ax1.set_yticks(np.arange(0.8, 1.02, 0.05))

# Right y-axis (Runtime)
ax2 = ax1.twinx()
ax2.plot(num_objects, runtime_mean, label='Runtime per Data (mean)', color='red')
ax2.fill_between(num_objects, runtime_mean - runtime_std, runtime_mean + runtime_std, color='red', alpha=0.3)
ax2.set_ylabel('Runtime per data (s)')

# Combine legends from both axes
# lines, labels = ax1.get_legend_handles_labels()
# lines2, labels2 = ax2.get_legend_handles_labels()
# ax2.legend(lines + lines2, labels + labels2, loc='lower right')

tikzplotlib.save("runtime-plot.tex")
plt.show()