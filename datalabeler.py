from pomp.example_problems.cagedatasetgenerator import *
from pomp.example_problems.cageplanner import *
from pomp.example_problems.cageenergylabeler import *
from pomp.planners import allplanners
from pomp.visualizer import *
import time
import csv
from main import parseParameters
import os

# Read from the CSV file
rows = []
with open('data_points_O.csv', 'r') as file:
    csv_reader = csv.reader(file)
    header = next(csv_reader)
    for row in csv_reader:
        rows.append([float(d) for d in row])

problem = 'CageMovingObstacle'
planner = 'ao-rrt'
planner,params = parseParameters(problem, planner)
if 'maxTime' in params:
    del params['maxTime']

print("Parameters:")
for (k,v) in iteritems(params):
    print(" ",k,":",v)


for i, data_i in enumerate(rows):
    problem = cageELTest(data_i)

    runVisualizer(problem,type=planner,**params)

    # Save escape energy labels to the original csv file
