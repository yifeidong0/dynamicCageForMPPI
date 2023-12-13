from pomp.bullet.scriptedmovement import *
from pomp.example_problems.cageplanner import *
from pomp.example_problems.waterswing import *
from pomp.example_problems.planepush import *
from pomp.example_problems.cageenergylabeler import *
from pomp.planners import allplanners
from pomp.visualizer import *
import time
import csv
from main import *
import os

plannername = 'ao-rrt'
prname = 'WaterSwing' # 'CageEnergyLabeler', 'PlanePush'
vis = 0
maxTime = 60

if prname == 'CageEnergyLabeler':
    dynamics_sim = forwardSimulationEL(gui=0)
    filename = ''
if prname == 'PlanePush':
    dynamics_sim = forwardSimulationPlanePush(gui=0)
    filename = ''
if prname == 'WaterSwing':
    dynamics_sim = forwardSimulationWaterSwing(gui=0)
    filenames = ['scripted_movement_viapoints_WaterSwing_t2.2.csv',
                'scripted_movement_viapoints_WaterSwing_t3.5.csv']

for filename in filenames:
    # Read from the CSV file
    rows = []
    ids = []
    with open(filename, 'r') as file:
        csv_reader = csv.reader(file)
        header = next(csv_reader)
        for id, row in enumerate(csv_reader):
            rows.append([float(d) for d in row[1:]])
            ids.append(int(id))

    params = {'maxTime':maxTime}
    if 'maxTime' in params:
        del params['maxTime']

    for i, data_i in enumerate(rows):
        if prname == 'CageEnergyLabeler':
            problem = cageELTest(dynamics_sim, data_i)
        if prname == 'PlanePush':
            problem = planePushTest(dynamics_sim, data_i)
        if prname == 'WaterSwing':
            problem = waterSwingTest(dynamics_sim, data_i)
        if vis:
            runVisualizer(problem, type=plannername, **params)
        else:
            print(ids[i])
            testPlannerDefault(problem, prname, maxTime, plannername, data_id=ids[i], **params)
