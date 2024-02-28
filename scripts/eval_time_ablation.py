import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import tikzplotlib

# Sample data
data = {
    'Iteration': [10, 10, 10, 30, 30, 30, 100, 100, 100, 1000, 1000, 1000],
    'AUC': [0.83, 0.8, 0.84, 0.9, 0.82, 0.89, 0.93, 0.89, 0.89, 0.92, 0.91, 0.89],
    'AP': [0.84, 0.79, 0.84, 0.91, 0.87, 0.91, 0.94, 0.91, 0.93, 0.94, 0.94, 0.92],
    'TimePerData': [0.02404, 0.02358, 0.02344, 0.06122, 0.06086, 0.06046, 0.1882, 0.206, 0.1872, 1.73856, 1.7402, 1.77076]
}

df = pd.DataFrame(data)

# Group by iteration and calculate mean and std
grouped = df.groupby('Iteration').agg(['mean', 'std'])

# Plotting all three metrics (AUC, AP, Time Per Data) in a single plot with a logarithmic x-axis
fig, ax1 = plt.subplots(figsize=(10, 6))

# Logarithmic scale for x-axis
ax1.set_xscale('log')

# AUC and AP on the left y-axis
ax1.errorbar(grouped.index, grouped['AUC']['mean'], yerr=grouped['AUC']['std'], fmt='-o', label='AUC', capsize=5)
ax1.errorbar(grouped.index, grouped['AP']['mean'], yerr=grouped['AP']['std'], fmt='-s', label='AP', capsize=5)
ax1.set_xlabel('# Iterations')
ax1.set_ylabel('AUC / AP')
ax1.set_yticks(np.arange(0.79, 0.97, 0.05))
# ax1.legend(loc='upper left')

# Time Per Data on the right y-axis
ax2 = ax1.twinx()
ax2.errorbar(grouped.index, grouped['TimePerData']['mean'], yerr=grouped['TimePerData']['std'], fmt='-^', label='Time Per Data', color='green', capsize=5)
ax2.set_ylabel('Run-time per data (s)')
# ax2.legend(loc='upper right')
ax2.set_yticks(np.arange(0, 2, 0.5))

# Set titles and grid
# ax1.set_title('AUC, AP, and Run-time Per Data vs Iterations')
ax1.grid(True)

tikzplotlib.save("time-ablation-plot.tex")
plt.show()