from pomp.bullet.scriptedmovement import *
from pomp.example_problems.cageplanner import *
from pomp.example_problems.waterswing import *
from pomp.example_problems.planepush import *
from pomp.example_problems.cageenergylabeler import *
from pomp.example_problems.boxpivot import *
from pomp.planners import allplanners
from pomp.visualizer import *
import time
import csv
from main import *
import os

plannername = 'ao-rrt'
prname = 'BoxPivot' # 'CageEnergyLabeler', 'PlanePush', 'BoxPivot', 'WaterSwing'
vis = 0
maxTime = 60

if prname == 'CageEnergyLabeler':
    # dynamics_sim = forwardSimulationEL(gui=0)
    filename = ''
if prname == 'PlanePush':
    # dynamics_sim = forwardSimulationPlanePush(gui=0)
    filename = ''
if prname == 'WaterSwing':
    # dynamics_sim = forwardSimulationWaterSwing(gui=0)
    filenames = [
                # 'data/waterswing/scripted_movement_viapoints_WaterSwing_radius3_t5.5.csv',
                'data/waterswing/scripted_movement_viapoints_WaterSwing_radius3_t3.5.csv',
                # 'data/waterswing/scripted_movement_viapoints_WaterSwing_t2.2.csv',
                #  'data/waterswing/scripted_movement_viapoints_WaterSwing_t3.5.csv',
                 ]
if prname == 'BoxPivot':
    # dynamics_sim = forwardSimulationBoxPivot(gui=0)
    filenames = [
                'data/boxpivot/scripted_movement_viapoints_BoxPivot_k8.0.csv',
                'data/boxpivot/scripted_movement_viapoints_BoxPivot_k2.0.csv',
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
        if prname == 'CageEnergyLabeler':
            dynamics_sim = forwardSimulationEL(gui=0)
            problem = cageELTest(dynamics_sim, data_i)
        if prname == 'PlanePush':
            dynamics_sim = forwardSimulationPlanePush(gui=0)
            problem = planePushTest(dynamics_sim, data_i)
        if prname == 'WaterSwing':
            dynamics_sim = forwardSimulationWaterSwing(gui=0)
            problem = waterSwingTest(dynamics_sim, data_i)
        if prname == 'BoxPivot':
            dynamics_sim = forwardSimulationBoxPivot(gui=0)
            problem = boxPivotTest(dynamics_sim, data_i)
        if vis:
            runVisualizer(problem, type=plannername, **params)
        else:
            print(ids[i])
            testPlannerDefault(problem, prname, maxTime, plannername, data_id=ids[i], **params)
        dynamics_sim.finish_sim()
        # time.sleep(3.0)
