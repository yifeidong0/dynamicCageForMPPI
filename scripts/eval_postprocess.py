
import time
import csv
# from main import *
import os

plannername = 'ao-rrt' # 'ao-est', 'rrt*', 'ao-rrt'
prname = 'PlanePush' # 'BalanceGrasp', 'PlanePush', 'PlanePushRrtstar', 'BoxPivot', 'WaterSwing', 'Shuffling'
num_via_points = 10
num_trajs = 50

if prname == 'PlanePush' or prname == 'PlanePushRrtstar':
    f_states = 'data/evaluation/push_fixture/rand_traj/dataset/scripted_movement_viapoints_PlanePush.csv'
    f_success_labels = 'data/evaluation/push_fixture/rand_traj/dataset/scripted_movement_success_labels_PlanePush.csv'
    f_maneuver_labels = 'data/evaluation/push_fixture/rand_traj/dataset/scripted_movement_maneuver_labels_PlanePush.csv'
    f_aorrt_metrics = 'data/evaluation/push_fixture/rand_traj/approaches/ao-rrt-metrics/ao_rrt.csv'
    f_aoest_metrics = 'data/evaluation/push_fixture/rand_traj/approaches/ao-est-metrics/ao_est.csv'
    f_heuristics = 'data/evaluation/push_fixture/rand_traj/approaches/heuristics/scripted_movement_heuristics_PlanePush.csv'
    f_soft_fixture = 'data/evaluation/push_fixture/rand_traj/approaches/rrtstar-softfixture/rrtstar.csv'
# if prname == 'BalanceGrasp':
#     filenames = ['data/evaluation/balance_grasp/test_data/scripted_movement_viapoints_BalanceGrasp_fail.csv',
#                  'data/evaluation/balance_grasp/test_data/scripted_movement_viapoints_BalanceGrasp_success.csv']
# if prname == 'WaterSwing':
#     filenames = [
#                 'scripted_movement_viapoints_WaterSwing_t3.5.csv',
#                  ]
# if prname == 'BoxPivot':
#     filenames = [
#                 'data/boxpivot/scripted_movement_viapoints_BoxPivot_k2.0_friction1.0.csv',
#                  ]
# if prname == 'Shuffling':
#     filenames = [
#                 'data/shuffling/scripted_movement_viapoints_Shuffling.csv',
#                  ]
    
# Read from the CSV file
rows = [] # 500
id_trajs = [] # 500
id_data = [] # 500
with open(f_states, 'r') as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)
    for id, row in enumerate(csv_reader):
        id_trajs.append(int(row[0]))
        id_data.append(int(row[1]))
        rows.append([float(d) for d in row[2:]])

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

success_metrics = [] # 500
maneuverability_metrics = [] # 500
with open(f_aorrt_metrics, 'r') as file:
# with open(f_aoest_metrics, 'r') as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)
    for id, row in enumerate(csv_reader):
        success_metrics.append(float(row[5]))
        maneuverability_metrics.append(float(row[6]))

soft_fixture_metrics = [] # 500
with open(f_soft_fixture, 'r') as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)
    for id, row in enumerate(csv_reader):
        d = 100.0 if row[4] == 'inf' else float(row[4])
        soft_fixture_metrics.append(d)

contact_force_metrics = [] # 500
closeset_distance_metrics = [] # 500
with open(f_heuristics, 'r') as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)
    for id, row in enumerate(csv_reader):
        contact_force_metrics.append(float(row[4]))
        closeset_distance_metrics.append(float(row[2]))

##################################
# Get predictions - maneuverability metric
print('###########################')
print('######maneuverability metrics######')
thres = 0.4
maneuver_pred = [m>thres for m in maneuverability_metrics]
tp = sum([(i==1 and j==1) for i,j in zip(maneuver_labels, maneuver_pred)])
tn = sum([(i==0 and j==0) for i,j in zip(maneuver_labels, maneuver_pred)])
fp = sum([(i==0 and j==1) for i,j in zip(maneuver_labels, maneuver_pred)])
fn = sum([(i==1 and j==0) for i,j in zip(maneuver_labels, maneuver_pred)])
print("True positive: {}".format(tp))
print("True negative: {}".format(tn))
print("False positive: {}".format(fp))
print("False negative: {}".format(fn))

# Accuracy (ACC): (TP + TN) / (TP + FP + FN +TN)
tptn = [i==j for i,j in zip(maneuver_labels, maneuver_pred)]
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
# Get predictions - success metric
print('###########################')
print('######success metrics######')
thres = 0.5
success_metrics_traj = []
# list of weights of length 10 that add up to 1 and exponentially increase
weights = [1.5**i for i in range(num_via_points)]
weights = [w/sum(weights) for w in weights]
for i in range(num_trajs):
    m = success_metrics[i*num_via_points:(i+1)*num_via_points]
    success_metrics_traj.append(sum([w*m for w,m in zip(weights,m)]))

success_pred = [m>thres for m in success_metrics_traj]
tp = sum([(i==1 and j==1) for i,j in zip(success_labels, success_pred)])
tn = sum([(i==0 and j==0) for i,j in zip(success_labels, success_pred)])
fp = sum([(i==0 and j==1) for i,j in zip(success_labels, success_pred)])
fn = sum([(i==1 and j==0) for i,j in zip(success_labels, success_pred)])
print("True positive: {}".format(tp))
print("True negative: {}".format(tn))
print("False positive: {}".format(fp))
print("False negative: {}".format(fn))

# Accuracy (ACC): (TP + TN) / (TP + FP + FN +TN)
tptn = [i==j for i,j in zip(success_labels, success_pred)]
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
# Soft Fixture - maneuverability metric
print('###########################')
print('######Soft Fixture maneuverability metrics######')
minval = 0.09
maxval = 3.01
thres = 0.4
soft_fixture_metrics = [(v-minval) / maxval for v in soft_fixture_metrics]
soft_fixture_pred = [m>thres for m in soft_fixture_metrics]
tp = sum([(i==1 and j==1) for i,j in zip(maneuver_labels, soft_fixture_pred)])
tn = sum([(i==0 and j==0) for i,j in zip(maneuver_labels, soft_fixture_pred)])
fp = sum([(i==0 and j==1) for i,j in zip(maneuver_labels, soft_fixture_pred)])
fn = sum([(i==1 and j==0) for i,j in zip(maneuver_labels, soft_fixture_pred)])
print("True positive: {}".format(tp))
print("True negative: {}".format(tn))
print("False positive: {}".format(fp))
print("False negative: {}".format(fn))

# Accuracy (ACC): (TP + TN) / (TP + FP + FN +TN)
tptn = [i==j for i,j in zip(maneuver_labels, soft_fixture_pred)]
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
# Get predictions - success metric
print('###########################')
print('######Soft Fixture success metrics######')
thres = 0.5
success_metrics_traj = []
# list of weights of length 10 that add up to 1 and exponentially increase
weights = [1.5**i for i in range(num_via_points)]
weights = [w/sum(weights) for w in weights]
for i in range(num_trajs):
    m = soft_fixture_metrics[i*num_via_points:(i+1)*num_via_points]
    success_metrics_traj.append(sum([w*m for w,m in zip(weights,m)]))

success_pred = [m>thres for m in success_metrics_traj]
tp = sum([(i==1 and j==1) for i,j in zip(success_labels, success_pred)])
tn = sum([(i==0 and j==0) for i,j in zip(success_labels, success_pred)])
fp = sum([(i==0 and j==1) for i,j in zip(success_labels, success_pred)])
fn = sum([(i==1 and j==0) for i,j in zip(success_labels, success_pred)])
print("True positive: {}".format(tp))
print("True negative: {}".format(tn))
print("False positive: {}".format(fp))
print("False negative: {}".format(fn))

# Accuracy (ACC): (TP + TN) / (TP + FP + FN +TN)
tptn = [i==j for i,j in zip(success_labels, success_pred)]
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
