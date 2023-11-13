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
maxTime = 20

# Read from the CSV file
rows = []
ids = []
with open('data/planar-gripper-dynamic-cage-dataset/data_points_O.csv', 'r') as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)
    for row in csv_reader:
        rows.append([float(d) for d in row[1:]])
        ids.append(float(row[0]))

planner = 'ao-rrt'
prname = 'CageEnergyLabeler'
params = {'maxTime':maxTime,'edgeCheckTolerance':.1,'selectionRadius':.05,'witnessRadius':.05}
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
