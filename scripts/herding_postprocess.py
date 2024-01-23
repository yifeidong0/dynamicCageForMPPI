import os
import numpy as np
import matplotlib.pyplot as plt
import csv

# Read from the CSV file
best_costs = []
success_metric = []
file_num_herders = 'data/evaluation/herding/num_herders/prob-aoest/ao_est_1.csv'
with open(file_num_herders, 'r') as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)
    for id, row in enumerate(csv_reader):
        best_costs.append(float(row[4]))
        success_metric.append(float(row[5]))

num_robots_list = list(range(4,9))
num_samples = 6 # 6 samples per number of robots
mean_costs = []
std_costs = []
mean_metrics = []
std_metrics = []
for num_robots in num_robots_list:
    costs = []
    metrics = []
    for i in range(num_samples):
        costs.append(best_costs[i + (num_robots-num_robots_list[0])*num_samples])
        metrics.append(success_metric[i + (num_robots-num_robots_list[0])*num_samples])
    mean_costs.append(np.mean(costs))
    std_costs.append(np.std(costs))
    mean_metrics.append(np.mean(metrics))
    std_metrics.append(np.std(metrics))

# # Plot costs vs. number of robots in terms of mean and std
# plt.figure()
# plt.errorbar(num_robots_list, mean_costs, yerr=std_costs, fmt='o')
# plt.xlabel('Number of robots')
# plt.ylabel('Best cost')
# # plt.title('AO-RRT')
# plt.show()

# Plot success metric vs. number of robots in terms of mean and std
plt.figure()
plt.errorbar(num_robots_list, mean_metrics, yerr=std_metrics, fmt='o')
plt.xlabel('Number of robots')
plt.ylabel('Success metric')
plt.show()
