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

plannername = 'ao-rrt' # 'ao-est', 'rrt*'
prname = 'BalanceGrasp' # 'PlanePushRrtstar', 'BalanceGrasp', 'PlanePush', 'BoxPivot', 'WaterSwing', 'Shuffling'
vis = 0
maxTime = 60 # only used when vis=0

if prname == 'BalanceGrasp':
    filenames = ['data/evaluation/balance_grasp/test_data/scripted_movement_viapoints_BalanceGrasp_fail.csv',
                 'data/evaluation/balance_grasp/test_data/scripted_movement_viapoints_BalanceGrasp_success.csv']
if prname == 'PlanePush' or prname == 'PlanePushRrtstar':
    filenames = [
        'data/evaluation/push_fixture/push_point_bias/unbiased/scripted_movement_viapoints_PlanePush.csv',
        'data/evaluation/push_fixture/push_point_bias/biased0.1/scripted_movement_viapoints_PlanePush.csv',
        'data/evaluation/push_fixture/push_point_bias/biased0.3/scripted_movement_viapoints_PlanePush.csv',
        ]
if prname == 'WaterSwing':
    filenames = [
                # 'data/waterswing/scripted_movement_viapoints_WaterSwing_radius3_t5.5.csv',
                # 'data/waterswing/scripted_movement_viapoints_WaterSwing_radius3_t3.5.csv',
                # 'data/waterswing/scripted_movement_viapoints_WaterSwing_t2.2.csv',
                #  'data/waterswing/scripted_movement_viapoints_WaterSwing_t3.5.csv',
                'scripted_movement_viapoints_WaterSwing_t3.5.csv',
                 ]
if prname == 'BoxPivot':
    filenames = [
                # 'data/boxpivot/scripted_movement_viapoints_BoxPivot_k8.0.csv',
                # 'data/boxpivot/scripted_movement_viapoints_BoxPivot_k2.0_friction0.4.csv',
                'data/boxpivot/scripted_movement_viapoints_BoxPivot_k2.0_friction1.0.csv',
                 ]
if prname == 'Shuffling':
    filenames = [
                'data/shuffling/scripted_movement_viapoints_Shuffling.csv',
                 ]
    
for filename in filenames:
    # Read from the CSV file
    rows = []
    ids = []
    with open(filename, 'r') as file:
        csv_reader = csv.reader(file)
        header = next(csv_reader)
        for id, row in enumerate(csv_reader):
            rows.append([float(d) for d in row[1:]])
            ids.append(int(row[0]))

    params = {'maxTime':maxTime}
    if 'maxTime' in params:
        del params['maxTime']

    for i, data_i in enumerate(rows):
        if prname == 'BalanceGrasp':
            dynamics_sim = forwardSimulationBalanceGrasp(gui=0)
            problem = BalanceGraspTest(dynamics_sim, data_i, save_hyperparams=1)
        if prname == 'PlanePush':
            dynamics_sim = forwardSimulationPlanePush(gui=0)
            problem = planePushTest(dynamics_sim, data_i, save_hyperparams=1)
        if prname == 'PlanePushRrtstar':
            dynamics_sim = forwardSimulationPlanePushRrtstar(gui=0)
            problem = PlanePushRrtstarTest(dynamics_sim, data_i, save_hyperparams=1)
        if prname == 'WaterSwing':
            dynamics_sim = forwardSimulationWaterSwing(gui=0)
            problem = waterSwingTest(dynamics_sim, data_i)
        if prname == 'BoxPivot':
            dynamics_sim = forwardSimulationBoxPivot(gui=0)
            problem = boxPivotTest(dynamics_sim, data_i)
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
