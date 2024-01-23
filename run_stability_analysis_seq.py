from pomp.bullet.scriptedmovement import *
from pomp.example_problems.cageplanner import *
from pomp.example_problems.waterswing import *
from pomp.example_problems.planepush import *
from pomp.example_problems.planepushrrtstar import *
from pomp.example_problems.balancegrasp import *
from pomp.example_problems.boxpivot import *
from pomp.example_problems.shuffling import *
from pomp.planners import allplanners
from pomp.visualizer import *
import time
import csv
from main import *
import os

# !!! Things remember to do BEFORE running: pruning, quasistatic_motion, pChooseGoal, densityEstimationRadius, goal sets, costs...
# !!! More non-maneuverable states needed in the 50 trajs

plannername = 'ao-rrt' # 'ao-est', 'rrt*', 'ao-rrt'
prname = 'BoxPivot' # 'BalanceGrasp', 'PlanePush', 'PlanePushRrtstar', 'BoxPivot', 'WaterSwing', 'Shuffling'
vis = 0
maxTime = 10 # only used when vis=0

if prname == 'PlanePush' or prname == 'PlanePushRrtstar':
    filenames = [
        'data/evaluation/push_fixture/rand_traj_3/dataset/scripted_movement_viapoints_PlanePush.csv',
        ]
if prname == 'BalanceGrasp':
    filenames = [
        'data/evaluation/balance_grasp/rand_traj_1/dataset/scripted_movement_viapoints_BalanceGrasp.csv',
        # 'data/evaluation/balance_grasp/trial/test_data/scripted_movement_viapoints_BalanceGrasp_fail.csv',
        # 'data/evaluation/balance_grasp/trial/test_data/scripted_movement_viapoints_BalanceGrasp_success.csv'
        ]
if prname == 'BoxPivot':
    filenames = [
                'data/evaluation/box_pivot/rand_fri_coeff/dataset/scripted_movement_viapoints_BoxPivot.csv',
                 ]
    filename_friction = 'data/evaluation/box_pivot/rand_fri_coeff/dataset/scripted_movement_maneuver_labels_BoxPivot.csv'
# if prname == 'Shuffling':
#     filenames = [
#                 'data/shuffling/scripted_movement_viapoints_Shuffling.csv',
#                  ]
    
for filename in filenames:
    # Read from the CSV file
    rows = []
    ids = []
    with open(filename, 'r') as file:
        csv_reader = csv.reader(file)
        header = next(csv_reader)
        for id, row in enumerate(csv_reader):
            rows.append([float(d) for d in row[2:]])
            ids.append(int(row[0]))

    if prname == 'BoxPivot':
        fri_coeffs = []
        with open(filename_friction, 'r') as file:
            csv_reader = csv.reader(file)
            header = next(csv_reader)
            for id, row in enumerate(csv_reader):
                fri_coeffs.append(float(row[3]))
    params = {'maxTime':maxTime}
    if 'maxTime' in params:
        del params['maxTime']

    # maneuver_labelset = []
    # k = 0
    
    for i, data_i in enumerate(rows):
        if prname == 'BalanceGrasp':
            dynamics_sim = forwardSimulationBalanceGrasp(gui=0)
            problem = BalanceGraspTest(dynamics_sim, data_i, save_hyperparams=1)
        if prname == 'PlanePush':
            dynamics_sim = forwardSimulationPlanePush(gui=0)
            problem = planePushTest(dynamics_sim, data_i, save_hyperparams=1,)
        if prname == 'PlanePushRrtstar':
            dynamics_sim = forwardSimulationPlanePushRrtstar(gui=0)
            problem = PlanePushRrtstarTest(dynamics_sim, data_i, save_hyperparams=1)
        if prname == 'BoxPivot':
            dynamics_sim = forwardSimulationBoxPivot(gui=0)
            problem = boxPivotTest(dynamics_sim, data_i, save_hyperparams=1, lateral_friction_coef=fri_coeffs[i])
        if prname == 'WaterSwing':
            dynamics_sim = forwardSimulationWaterSwing(gui=0)
            problem = waterSwingTest(dynamics_sim, data_i)
        if prname == 'Shuffling':
            dynamics_sim = forwardSimulationShuffling(gui=0)
            problem = ShufflingTest(dynamics_sim, data_i)
        if vis:
            runVisualizer(problem, type=plannername, **params)
        else:
            print(ids[i])
            testPlannerDefault(problem, prname, maxTime, plannername, data_id=ids[i], **params)
        dynamics_sim.finish_sim()
        # time.sleep(3.0)