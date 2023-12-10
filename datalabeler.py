from pomp.example_problems.cagedatasetgenerator import *
from pomp.example_problems.cageplanner import *
from pomp.example_problems.cageenergylabeler import *
from pomp.planners import allplanners
from pomp.visualizer import *
import time
import csv
from main import *
import os

# TODO: inf labels: 
# object and gripper small ypos; 
# object in collision with gripper;
# sol: penalize small ypos
# multiple trials might be useful

# data with excessively large escape energy labels affect the training greatly.
# sol: EST, the current RRT version does not expand nodes adequetly, which might be better with a bit hyperparameter tuning.
# Overall, the dataset is not of high quality for now.

vis = 1
maxTime = 12
# filename = 'data/planar-gripper-dynamic-cage-dataset/data_points_O.csv'
# filename = 'data/dataset-loaded-from-mppi-planner/filtered_states_from_mppi.csv'
filename = 'data/9kdataset-from-mppi/states_from_mppi_9k.csv'

# Read from the CSV file
rows = []
ids = []
with open(filename, 'r') as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)
    for id, row in enumerate(csv_reader):
        rows.append([float(d) for d in row[4:]])
        ids.append(int(id))

planner = 'ao-rrt'
prname = 'CageEnergyLabeler'
params = {'maxTime':maxTime,'edgeCheckTolerance':.04}
if 'maxTime' in params:
    del params['maxTime']

print("Parameters:")
for (k,v) in iteritems(params):
    print(" ",k,":",v)

for i, data_i in enumerate(rows):
    problem = cageELTest(data_i)
    if vis:
        runVisualizer(problem,type=planner,**params)
    else:
        print(ids[i])
        testPlannerDefault(problem,prname,maxTime,planner,data_id=ids[i],**params)

    # Save escape energy labels to the original csv file
