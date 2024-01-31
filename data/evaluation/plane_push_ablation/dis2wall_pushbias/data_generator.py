
import csv
import numpy as np

# Data pattern observed
num_traj = [0, 1, 2, 3, 4, 5, 6]
xg_values = np.arange(4.45, 5.55, 0.05).tolist()
yo_values = np.arange(4.3, 8.8, 0.5).tolist()
static_values = [0.0, 0.0, 0.0, 0.0]  # xo, thetao, vxo, vyo, omegao
dynamic_values = [0.0, 0.0, 1.0, 0.0]  # thetag, vxg, vyg, omegag

# Generate the extended data
data = []
for id,xg in enumerate(xg_values):
    for j,yo in enumerate(yo_values):
        yg = yo - 0.3
        data0 = [id, j, 5,] + [yo,] + static_values + [xg, yg,] + dynamic_values
        data.append(data0)

filename = 'data/evaluation/plane_push_ablation/dis2wall_pushbias/ablation_dp_PlanePush.csv'
headers = ['offset', 'distance', 'xo', 'yo', 'thetao', 'vxo', 'vyo', 'omegao', 'xg', 'yg', 'thetag', 'vxg', 'vyg', 'omegag']
with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(headers)
    writer.writerows(data)
