
import time
import csv
# from main import *
import os
import numpy as np

from sklearn.metrics import roc_curve, auc, precision_recall_curve, average_precision_score
import matplotlib.pyplot as plt

# Function to calculate AUC and AP and plot ROC and PRC
def plot_metrics_and_curves(labels, predictions):
    # Calculating True Positive Rate (TPR), False Positive Rate (FPR), Precision and Recall for various thresholds
    fpr, tpr, roc_thresholds = roc_curve(labels, predictions)
    roc_auc = auc(fpr, tpr)

    precision, recall, pr_thresholds = precision_recall_curve(labels, predictions)
    average_precision = average_precision_score(labels, predictions)

    # Plotting ROC Curve
    plt.figure(figsize=(12, 5))

    plt.subplot(1, 2, 1)
    plt.plot(fpr, tpr, color='blue', lw=2, label='ROC curve (area = %0.2f)' % roc_auc)
    plt.plot([0, 1], [0, 1], color='navy', lw=2, linestyle='--')
    plt.xlim([0.0, 1.0])
    plt.ylim([0.0, 1.05])
    plt.xlabel('False Positive Rate')
    plt.ylabel('True Positive Rate')
    plt.title('Receiver Operating Characteristic')
    plt.legend(loc="lower right")

    # Plotting Precision-Recall Curve
    plt.subplot(1, 2, 2)
    plt.plot(recall, precision, color='green', lw=2, label='PR curve (AP = %0.2f)' % average_precision)
    plt.fill_between(recall, precision, alpha=0.2, color='green')
    plt.xlim([0.0, 1.0])
    plt.ylim([0.0, 1.05])
    plt.xlabel('Recall')
    plt.ylabel('Precision')
    plt.title('Precision-Recall Curve')
    plt.legend(loc="lower left")

    plt.tight_layout()
    plt.show()

    return roc_auc, average_precision

# plannername = 'ao-rrt' # 'ao-est', 'rrt*', 'ao-rrt'
prname = 'BalanceGrasp' # 'BalanceGrasp', 'PlanePush', 'PlanePushRrtstar', 'BoxPivot', 'WaterSwing', 'Shuffling'
num_via_points = 10
num_trajs = 20

if prname == 'PlanePush':
    # f_states = 'data/evaluation/push_fixture/rand_traj/dataset/scripted_movement_viapoints_PlanePush.csv'
    f_success_labels = 'data/evaluation/push_fixture/rand_traj/dataset/scripted_movement_success_labels_PlanePush.csv'
    f_maneuver_labels = 'data/evaluation/push_fixture/rand_traj/dataset/scripted_movement_maneuver_labels_PlanePush.csv'
    f_aorrt_metrics = 'data/evaluation/push_fixture/rand_traj/approaches/ao-rrt-metrics/ao_rrt.csv'
    f_aoest_metrics = 'data/evaluation/push_fixture/rand_traj/approaches/ao-est-metrics/ao_est.csv'
    f_heuristics = 'data/evaluation/push_fixture/rand_traj/approaches/heuristics/scripted_movement_heuristics_PlanePush.csv'
    f_soft_fixture = 'data/evaluation/push_fixture/rand_traj/approaches/rrtstar-softfixture/rrtstar.csv'
    f_quasistatic = 'data/evaluation/push_fixture/rand_traj/approaches/ao-rrt-quasistatic/ao_rrt.csv'
if prname == 'BalanceGrasp':
    f_success_labels = 'data/evaluation/balance_grasp/comp_contact_score/dataset/scripted_movement_success_labels_BalanceGrasp.csv'
    f_maneuver_labels = 'data/evaluation/balance_grasp/comp_contact_score/dataset/scripted_movement_maneuver_labels_BalanceGrasp.csv'
    f_aorrt_metrics = 'data/evaluation/balance_grasp/comp_contact_score/approaches/prob-aorrt/ao_rrt.csv'
    f_aoest_metrics = ''
    f_soft_fixture = ''
    # f_quasistatic = 'data/evaluation/balance_grasp/rand_traj/approaches/ao-rrt-quasistatic/ao_rrt.csv'
    f_heuristics = 'data/evaluation/balance_grasp/comp_contact_score/dataset/scripted_movement_heuristics_BalanceGrasp.csv'
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

# success_metrics_quasistatic = []
# maneuverability_metrics_quasistatic = []
# with open(f_quasistatic, 'r') as file:
#     csv_reader = csv.reader(file)
#     header = next(csv_reader)
#     for id, row in enumerate(csv_reader):
#         success_metrics_quasistatic.append(float(row[5]))
#         maneuverability_metrics_quasistatic.append(float(row[6]))

closeset_distance_metrics = [] # 500
s_stick_metrics = [] # 500
s_engage_metrics = [] # 500
with open(f_heuristics, 'r') as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)
    for id, row in enumerate(csv_reader):
        closeset_distance_metrics.append(float(row[2]))
        s_stick_metrics.append(float(row[3]))
        s_engage_metrics.append(float(row[4]))

##################################
def get_m_metric(labels, predictions):
    """ maneuverability metric
        metric_original: list of floats, len=num_traj*num_via_points
    """
    # Example usage with the AO-RRT success metrics
    roc_auc, average_precision = plot_metrics_and_curves(labels, predictions)
    print("AUC (Area Under Curve) for ROC: ", roc_auc)
    print("AP (Average Precision) for Precision-Recall Curve: ", average_precision)

def get_s_metric(labels, predictions):
    """ success metric
        metric_original: list of floats, len=num_traj*num_via_points
    """
    metrics_traj = []
    # list of weights of length 10 that add up to 1 and exponentially increase
    weights = [1.5**i for i in range(num_via_points)]
    # weights = [w/sum(weights) for w in weights]
    for i in range(num_trajs):
        m = predictions[i*num_via_points:(i+1)*num_via_points]
        metrics_traj.append(sum([w*m for w,m in zip(weights,m)]))

    # Example usage with the AO-RRT success metrics
    roc_auc, average_precision = plot_metrics_and_curves(labels, metrics_traj)
    print("AUC (Area Under Curve) for ROC: ", roc_auc)
    print("AP (Average Precision) for Precision-Recall Curve: ", average_precision)

##################################
print('######1. AO-RRT Maneuverability Metric######')
# minval, maxval, thres, metric_original = 0.0, 1.0, m_thres, maneuverability_metric_aorrt
get_m_metric(maneuver_labels, maneuverability_metric_aorrt)

print('######AO-RRT Success Metric######')
# minval, maxval, thres, metric_original = 0.0, 1.0, s_thres, success_metrics_aorrt
get_s_metric(success_labels, success_metrics_aorrt)

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

# print('######4. AO-RRT Quasistatic Motion Maneuverability Metric######')
# minval, maxval, thres, metric_original = 0., 1.0, m_thres, maneuverability_metrics_quasistatic
# get_m_metric(minval, maxval, thres, metric_original)

# print('######AO-RRT Quasistatic Motion Success Metric######')
# minval, maxval, thres, metric_original = 0., 1.0, s_thres, success_metrics_quasistatic
# get_s_metric(minval, maxval, thres, metric_original)

# # 5,6 are good metrics but hard to determine the threshold
# print('######5. Heuristic - Closest Distance - Maneuverability Metric######')
# minval, maxval, thres, metric_original = 0., -1.0, dis_thres, closeset_distance_metrics
# get_m_metric(minval, maxval, thres, metric_original)

# print('######Heuristic - Closest Distance - Success Metric######')
# minval, maxval, thres, metric_original = 0., -1.0, dis_thres, closeset_distance_metrics
# get_s_metric(minval, maxval, thres, metric_original)

print('######6. Baseline - Contact Force-related Score - Maneuverability ######')
w0, w1, w2 = -1, 1, 1
hybrid_score = [(w0*d + w1*s + w2*e) for d,s,e in zip(closeset_distance_metrics, s_stick_metrics, s_engage_metrics)]
# closeset_distance_metrics = [m / max(closeset_distance_metrics) for m in closeset_distance_metrics]
# s_stick_metrics = [m / max(s_stick_metrics) for m in s_stick_metrics]
# s_engage_metrics = [m / max(s_engage_metrics) for m in s_engage_metrics]
# hybrid_score = [np.exp(w0*d + w1*s + w2*e) for d,s,e in zip(closeset_distance_metrics, s_stick_metrics, s_engage_metrics)]
# hybrid_score = [s / max(hybrid_score) for s in hybrid_score]

get_m_metric(maneuver_labels, hybrid_score)

print('######Heuristic - Contact Force - Success Metric######')
get_s_metric(success_labels, hybrid_score)



