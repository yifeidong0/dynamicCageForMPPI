
import time
import csv
# from main import *
import os
import numpy as np

# plannername = 'ao-rrt' # 'ao-est', 'rrt*', 'ao-rrt'
prname = 'BalanceGrasp' # 'BalanceGrasp', 'PlanePush', 'PlanePushRrtstar', 'BoxPivot', 'WaterSwing', 'Shuffling'
num_via_points = 10
num_trajs = 30

if prname == 'PlanePush':
    # f_states = 'data/evaluation/push_fixture/rand_traj/dataset/scripted_movement_viapoints_PlanePush.csv'
    f_success_labels = 'data/evaluation/push_fixture/rand_traj/dataset/scripted_movement_success_labels_PlanePush.csv'
    f_maneuver_labels = 'data/evaluation/push_fixture/rand_traj/dataset/scripted_movement_maneuver_labels_PlanePush.csv'
    f_aorrt_metrics = 'data/evaluation/push_fixture/rand_traj/approaches/ao-rrt-metrics/ao_rrt.csv'
    f_aoest_metrics = 'data/evaluation/push_fixture/rand_traj/approaches/ao-est-metrics/ao_est.csv'
    f_heuristics = 'data/evaluation/push_fixture/rand_traj/approaches/heuristics/scripted_movement_heuristics_PlanePush.csv'
    f_soft_fixture = 'data/evaluation/push_fixture/rand_traj/approaches/rrtstar-softfixture/rrtstar.csv'
    f_quasistatic = 'data/evaluation/push_fixture/rand_traj/approaches/ao-rrt-quasistatic/ao_rrt.csv'
    m_thres = 0.4
    s_thres = 0.5
    dis_thres = -1e-1
    force_thres = 1.0
if prname == 'BalanceGrasp':
    f_success_labels = 'data/evaluation/balance_grasp/rand_traj/dataset/scripted_movement_success_labels_BalanceGrasp.csv'
    f_maneuver_labels = 'data/evaluation/balance_grasp/rand_traj/dataset/scripted_movement_maneuver_labels_BalanceGrasp.csv'
    f_aorrt_metrics = 'data/evaluation/balance_grasp/rand_traj/approaches/ao-rrt-metrics/ao_rrt.csv'
    f_aoest_metrics = ''
    f_soft_fixture = ''
    f_quasistatic = 'data/evaluation/balance_grasp/rand_traj/approaches/ao-rrt-quasistatic/ao_rrt.csv'
    f_heuristics = 'data/evaluation/balance_grasp/rand_traj/approaches/heuristics/scripted_movement_heuristics_BalanceGrasp.csv'
    m_thres = 0.1
    s_thres = 0.5
    dis_thres = -1e-2
    force_thres = 1.0

# if prname == 'BoxPivot':
# if prname == 'Shuffling':

# # Read from the CSV file
# rows = [] # 500
# id_trajs = [] # 500
# id_data = [] # 500
# with open(f_states, 'r') as file:
#     csv_reader = csv.reader(file)
#     header = next(csv_reader)
#     for id, row in enumerate(csv_reader):
#         id_trajs.append(int(row[0]))
#         id_data.append(int(row[1]))
#         rows.append([float(d) for d in row[2:]])

success_labels = [] # 50
with open(f_success_labels, 'r') as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)
    for id, row in enumerate(csv_reader):
        success_labels.append(int(row[1]))

maneuver_labels = [] # 500
with open(f_maneuver_labels, 'r') as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)
    for id, row in enumerate(csv_reader):
        maneuver_labels.append(int(row[2]))

success_metrics_aorrt = []
maneuverability_metric_aorrt = []
with open(f_aorrt_metrics, 'r') as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)
    for id, row in enumerate(csv_reader):
        success_metrics_aorrt.append(float(row[5]))
        maneuverability_metric_aorrt.append(float(row[6]))

# success_metrics_aoest = []
# maneuverability_metric_aoest = []
# with open(f_aoest_metrics, 'r') as file:
#     csv_reader = csv.reader(file)
#     header = next(csv_reader)
#     for id, row in enumerate(csv_reader):
#         success_metrics_aoest.append(float(row[5]))
#         maneuverability_metric_aoest.append(float(row[6]))

# soft_fixture_metrics = []
# with open(f_soft_fixture, 'r') as file:
#     csv_reader = csv.reader(file)
#     header = next(csv_reader)
#     for id, row in enumerate(csv_reader):
#         d = 100.0 if row[4] == 'inf' else float(row[4])
#         soft_fixture_metrics.append(d)

success_metrics_quasistatic = []
maneuverability_metrics_quasistatic = []
with open(f_quasistatic, 'r') as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)
    for id, row in enumerate(csv_reader):
        success_metrics_quasistatic.append(float(row[5]))
        maneuverability_metrics_quasistatic.append(float(row[6]))

contact_force_metrics = [] # 500
closeset_distance_metrics = [] # 500
with open(f_heuristics, 'r') as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)
    for id, row in enumerate(csv_reader):
        contact_force_metrics.append(float(row[4]))
        closeset_distance_metrics.append(float(row[2]))

##################################
def get_m_metric(minval, maxval, thres, metric_original):
    """ maneuverability metric
        metric_original: list of floats, len=num_traj*num_via_points
    """
    metric = [(v-minval) / maxval for v in metric_original]
    maneuver_pred = [m>thres for m in metric]
    tp = sum([(i==1 and j==1) for i,j in zip(maneuver_labels, maneuver_pred)])
    tn = sum([(i==0 and j==0) for i,j in zip(maneuver_labels, maneuver_pred)])
    fp = sum([(i==0 and j==1) for i,j in zip(maneuver_labels, maneuver_pred)])
    fn = sum([(i==1 and j==0) for i,j in zip(maneuver_labels, maneuver_pred)])
    print("True positive: {}".format(tp))
    print("True negative: {}".format(tn))
    print("False positive: {}".format(fp))
    print("False negative: {}".format(fn))

    # Accuracy (ACC): (TP + TN) / (TP + FP + FN +TN)
    accuracy = (tp+tn) / (tp+tn+fp+fn)
    print("Accuracy: {}".format(accuracy))

    # True positive rate (TPR): TP / P
    tpr = tp / (tp+fp)
    print("True positive rate: {}".format(tpr))

    # True negative rate (TNR): TN / N
    tnr = tn / (tn+fn)
    print("True negative rate: {}".format(tnr))

    # Success rate (SR): TPR / (TPR-TNR+1)
    sr = tpr / (tpr-tnr+1)
    print("Success rate: {}".format(sr))

def get_s_metric(minval, maxval, thres, metric_original):
    """ success metric
        metric_original: list of floats, len=num_traj*num_via_points
    """
    metric = [(v-minval) / maxval for v in metric_original]
    metrics_traj = []
    # list of weights of length 10 that add up to 1 and exponentially increase
    weights = [1.5**i for i in range(num_via_points)]
    weights = [w/sum(weights) for w in weights]
    for i in range(num_trajs):
        m = metric[i*num_via_points:(i+1)*num_via_points]
        metrics_traj.append(sum([w*m for w,m in zip(weights,m)]))

    success_pred = [m>thres for m in metrics_traj]
    tp = sum([(i==1 and j==1) for i,j in zip(success_labels, success_pred)])
    tn = sum([(i==0 and j==0) for i,j in zip(success_labels, success_pred)])
    fp = sum([(i==0 and j==1) for i,j in zip(success_labels, success_pred)])
    fn = sum([(i==1 and j==0) for i,j in zip(success_labels, success_pred)])
    print("True positive: {}".format(tp))
    print("True negative: {}".format(tn))
    print("False positive: {}".format(fp))
    print("False negative: {}".format(fn))

    # Accuracy (ACC): (TP + TN) / (TP + FP + FN +TN)
    accuracy = (tp+tn) / (tp+tn+fp+fn)
    print("Accuracy: {}".format(accuracy))

    # True positive rate (TPR): TP / P
    tpr = tp / (tp+fp)
    print("True positive rate: {}".format(tpr))

    # True negative rate (TNR): TN / N
    tnr = tn / (tn+fn)
    print("True negative rate: {}".format(tnr))

    # Success rate (SR): TPR / (TPR-TNR+1)
    sr = tpr / (tpr-tnr+1)
    print("Success rate: {}".format(sr))

##################################
print('######1. AO-RRT Maneuverability Metric######')
minval, maxval, thres, metric_original = 0.0, 1.0, m_thres, maneuverability_metric_aorrt
get_m_metric(minval, maxval, thres, metric_original)

print('######AO-RRT Success Metric######')
minval, maxval, thres, metric_original = 0.0, 1.0, s_thres, success_metrics_aorrt
get_s_metric(minval, maxval, thres, metric_original)

# print('######2. AO-EST Maneuverability Metric######')
# minval, maxval, thres, metric_original = 0.0, 1.0, m_thres, maneuverability_metric_aoest
# get_m_metric(minval, maxval, thres, metric_original)

# print('######AO-EST Success Metric######')
# minval, maxval, thres, metric_original = 0.0, 1.0, s_thres, success_metrics_aoest
# get_s_metric(minval, maxval, thres, metric_original)

# print('######3. RRT* Soft Fixture Maneuverability Metric######')
# minval, maxval, thres, metric_original = 0.09, 3.01, m_thres, soft_fixture_metrics
# get_m_metric(minval, maxval, thres, metric_original)

# print('######RRT* Soft Fixture Success Metric######')
# minval, maxval, thres, metric_original = 0.09, 3.01, s_thres, soft_fixture_metrics
# get_s_metric(minval, maxval, thres, metric_original)

print('######4. AO-RRT Quasistatic Motion Maneuverability Metric######')
minval, maxval, thres, metric_original = 0., 1.0, m_thres, maneuverability_metrics_quasistatic
get_m_metric(minval, maxval, thres, metric_original)

print('######AO-RRT Quasistatic Motion Success Metric######')
minval, maxval, thres, metric_original = 0., 1.0, s_thres, success_metrics_quasistatic
get_s_metric(minval, maxval, thres, metric_original)

# 5,6 are good metrics but hard to determine the threshold
print('######5. Heuristic - Closest Distance - Maneuverability Metric######')
minval, maxval, thres, metric_original = 0., -1.0, dis_thres, closeset_distance_metrics
get_m_metric(minval, maxval, thres, metric_original)

print('######Heuristic - Closest Distance - Success Metric######')
minval, maxval, thres, metric_original = 0., -1.0, dis_thres, closeset_distance_metrics
get_s_metric(minval, maxval, thres, metric_original)

print('######6. Heuristic - Contact Force - Maneuverability Metric######')
minval, maxval, thres, metric_original = 0., 1.0, force_thres, contact_force_metrics
get_m_metric(minval, maxval, thres, metric_original)

print('######Heuristic - Contact Force - Success Metric######')
minval, maxval, thres, metric_original = 0., 1.0, force_thres, contact_force_metrics
get_s_metric(minval, maxval, thres, metric_original)
