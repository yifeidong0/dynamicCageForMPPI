import os
import numpy as np
import matplotlib.pyplot as plt
import csv

# Read from the CSV file
best_costs = []
file_num_herders = 'data/evaluation/herding/num_herders/effort-aorrt/ao_rrt.csv'
with open(file_num_herders, 'r') as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)
    for id, row in enumerate(csv_reader):
        best_costs.append(float(row[4]))
print(len(best_costs)) # 140

num_robots_list = list(range(1, 15))
num_samples = 10 # 10 samples per number of robots
mean_costs = []
std_costs = []
for num_robots in num_robots_list:
    costs = []
    for i in range(num_samples):
        costs.append(best_costs[i + (num_robots-1)*num_samples])
    mean_costs.append(np.mean(costs))
    std_costs.append(np.std(costs))
print(mean_costs)
print(std_costs)

# Plot costs vs. number of robots in terms of mean and std
plt.figure()
plt.errorbar(num_robots_list, mean_costs, yerr=std_costs, fmt='o')
plt.xlabel('Number of robots')
plt.ylabel('Best cost')
# plt.title('AO-RRT')
plt.show()
