import csv
import math
import numpy as np
import random

f_heuristics = 'data/evaluation/perturbed_state_estimation/push_fixture/dataset/scripted_movement_heuristics_PlanePush.csv'
closeset_distance_metrics = []
s_stick_metrics = []
s_engage_metrics = []
with open(f_heuristics, 'r') as file:
    csv_reader = csv.reader(file)
    header_h = next(csv_reader)
    for id, row in enumerate(csv_reader):
        closeset_distance_metrics.append(float(row[2]))
        s_stick_metrics.append(float(row[3]))
        s_engage_metrics.append(float(row[4]))

f_maneuver_labels = 'data/evaluation/perturbed_state_estimation/push_fixture/dataset/scripted_movement_maneuver_labels_PlanePush.csv'
num_traj,data_id = [],[]
fri_coefs = []
fri_coefs_perturbed1 = []
fri_coefs_perturbed2 = []
fri_coefs_perturbed3 = []
fri_coefs_perturbed4 = []
with open(f_maneuver_labels, 'r') as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)
    for id, row in enumerate(csv_reader):
        num_traj.append(int(float(row[0])))
        data_id.append(int(float(row[1])))
        fri_coefs.append(float(row[3]))
        fri_coefs_perturbed1.append(float(row[4]))
        fri_coefs_perturbed2.append(float(row[5]))
        fri_coefs_perturbed3.append(float(row[6]))
        fri_coefs_perturbed4.append(float(row[7]))

# s_stick = (self.lateral_friction_coef*contact_normal_force - abs(contact_friction_force_xy)) * math.cos(np.arctan(self.lateral_friction_coef))
abs_f_laterals = [mu*engage - stick/math.cos(np.arctan(mu)) for mu, engage, stick in zip(fri_coefs, s_engage_metrics, s_stick_metrics)]

###############################
# Add friction coefficient perturbation

# s_stick_metrics_perturbed1 = [(mu*engage - lat) * math.cos(np.arctan(mu)) for mu, engage, lat in zip(fri_coefs_perturbed1, s_engage_metrics, abs_f_laterals)]
# s_stick_metrics_perturbed2 = [(mu*engage - lat) * math.cos(np.arctan(mu)) for mu, engage, lat in zip(fri_coefs_perturbed2, s_engage_metrics, abs_f_laterals)]
# s_stick_metrics_perturbed3 = [(mu*engage - lat) * math.cos(np.arctan(mu)) for mu, engage, lat in zip(fri_coefs_perturbed3, s_engage_metrics, abs_f_laterals)]
# s_stick_metrics_perturbed4 = [(mu*engage - lat) * math.cos(np.arctan(mu)) for mu, engage, lat in zip(fri_coefs_perturbed4, s_engage_metrics, abs_f_laterals)]

# # Save to CSV heuristics files for perturbed state estimation
# f_heuristics_perturbed1 = 'data/evaluation/perturbed_state_estimation/push_fixture/friction/results-hou/scripted_movement_heuristics_perturbed_0.1.csv'
# f_heuristics_perturbed2 = 'data/evaluation/perturbed_state_estimation/push_fixture/friction/results-hou/scripted_movement_heuristics_perturbed_0.2.csv'
# f_heuristics_perturbed3 = 'data/evaluation/perturbed_state_estimation/push_fixture/friction/results-hou/scripted_movement_heuristics_perturbed_0.5.csv'
# f_heuristics_perturbed4 = 'data/evaluation/perturbed_state_estimation/push_fixture/friction/results-hou/scripted_movement_heuristics_perturbed_1.0.csv'

# with open(f_heuristics_perturbed1, 'w') as file:
#     csv_writer = csv.writer(file)
#     csv_writer.writerow(header_h)
#     for i in range(len(num_traj)):
#         csv_writer.writerow([num_traj[i], data_id[i], closeset_distance_metrics[i], s_stick_metrics_perturbed1[i], s_engage_metrics[i]])

# with open(f_heuristics_perturbed2, 'w') as file:
#     csv_writer = csv.writer(file)
#     csv_writer.writerow(header_h)
#     for i in range(len(num_traj)):
#         csv_writer.writerow([num_traj[i], data_id[i], closeset_distance_metrics[i], s_stick_metrics_perturbed2[i], s_engage_metrics[i]])

# with open(f_heuristics_perturbed3, 'w') as file:
#     csv_writer = csv.writer(file)
#     csv_writer.writerow(header_h)
#     for i in range(len(num_traj)):
#         csv_writer.writerow([num_traj[i], data_id[i], closeset_distance_metrics[i], s_stick_metrics_perturbed3[i], s_engage_metrics[i]])

# with open(f_heuristics_perturbed4, 'w') as file:
#     csv_writer = csv.writer(file)
#     csv_writer.writerow(header_h)
#     for i in range(len(num_traj)):
#         csv_writer.writerow([num_traj[i], data_id[i], closeset_distance_metrics[i], s_stick_metrics_perturbed4[i], s_engage_metrics[i]])

###############################
# # Add normal force perturbation
# perturbed_normal_forces = [0.0, 0.5, 1, 3, 5, 10, 20,]
# for p in perturbed_normal_forces:
#     s_engage_metrics_perturbed = [max(0.0, s + random.uniform(-p,p)) for s in s_engage_metrics]
#     s_stick_metrics_perturbed = [(mu*engage - lat) * math.cos(np.arctan(mu)) for mu, engage, lat in zip(fri_coefs, s_engage_metrics_perturbed, abs_f_laterals)]

#     # Save to CSV heuristics files for perturbed state estimation
#     f_heuristics_perturbed = 'data/evaluation/perturbed_state_estimation/push_fixture/normal_force/scripted_movement_heuristics_perturbed_{}.csv'.format(p)
#     with open(f_heuristics_perturbed, 'w') as file:
#         csv_writer = csv.writer(file)
#         csv_writer.writerow(header_h)
#         for i in range(len(num_traj)):
#             csv_writer.writerow([num_traj[i], data_id[i], closeset_distance_metrics[i], s_stick_metrics_perturbed[i], s_engage_metrics_perturbed[i]])

###############################
# Add normal force perturbation
perturbed_normal_forces = [0.0, 0.2, 0.5, 1, 3, 5]
for p in perturbed_normal_forces:
    abs_f_laterals_perturbed = [max(0.0, s + random.uniform(-p,p)) for s in abs_f_laterals]
    s_stick_metrics_perturbed = [(mu*engage - lat) * math.cos(np.arctan(mu)) for mu, engage, lat in zip(fri_coefs, s_engage_metrics, abs_f_laterals_perturbed)]

    # Save to CSV heuristics files for perturbed state estimation
    f_heuristics_perturbed = 'data/evaluation/perturbed_state_estimation/push_fixture/lateral_force/scripted_movement_heuristics_perturbed_{}.csv'.format(p)
    with open(f_heuristics_perturbed, 'w') as file:
        csv_writer = csv.writer(file)
        csv_writer.writerow(header_h)
        for i in range(len(num_traj)):
            csv_writer.writerow([num_traj[i], data_id[i], closeset_distance_metrics[i], s_stick_metrics_perturbed[i], s_engage_metrics[i]])

