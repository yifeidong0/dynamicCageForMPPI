from pomp.example_problems.cagedatasetgenerator import *
from pomp.example_problems.cageplanner import *
import time
import csv
import numpy as np

# TODO: dataset is generated in two ways:
# 1. sim in Pybullet (high quality with object on the top of gripper)
# 2. random C-space sampling (low quality)

# 1. Data points generated from Bullet
num_trajs = 500
num_via_points = 10
gui = 0
filename = "data_points_O.csv"

cage_planner = CagePlanner()
params = cage_planner.params
y_range = cage_planner.y_range
x_range = cage_planner.x_range
half_extents_gripper = cage_planner.half_extents_gripper
radius_object = cage_planner.radius_object
g = cage_planner.gravity
cbd = cage_planner.c_space_boundary
ubd = cage_planner.u_boundary

sim = dataGeneratorSim(params, gui=gui)

data_points_O = [] # in OpenGL coordinates (x) not in Bullet coordinates (q)
k = 0
for i in range(num_trajs):
    # time.sleep(2)
    # Generate random state and input as initialization in OpenGL coordinates
    x_rand = generate_random_point(cbd)
    x_rand[0] = (x_range-1)*random.random() + 0.5 # xo
    x_rand[1] = y_range*random.random() # yo
    x_rand[2] = 0.4*random.random() - 0.2 # vxo
    x_rand[3] = 0.4*random.random() - 0.2 # vyo
    x_rand[4] = x_rand[0] + random.random() - 0.5 # xg
    x_rand[5] = x_rand[1] + radius_object + half_extents_gripper[1] # yg
    x_rand[6] = 0.0 # thetag

    t = np.random.uniform(ubd[0][0],ubd[1][0])
    ax = np.random.uniform(ubd[0][1],ubd[1][1])
    ay = np.random.uniform(ubd[0][2],ubd[1][2])
    alpha = np.random.uniform(ubd[0][3],ubd[1][3])
    u_rand = [t, ax, ay, alpha]

    # print("x_rand", x_rand)
    # print("u_rand", u_rand)
    # x_rand = [2, 4, 0, 0, 2, 4.11, 0, 0, 0, 0]
    # u_rand = [.1,1,0,0]

    # Pass to Bullet and retrieve a data point
    q, mu = toBulletStateInput(x_rand, u_rand, y_range)
    sim.reset_states(q)
    q_news = sim.run_forward_sim(mu, num_via_points)
    for q_new in q_news:
        x_new = toOpenglStateInput(q_new, y_range)

        # Check if inside C-space boundaries
        is_valid = check_bounds(x_new, cbd)
        if is_valid:
            data = [k,] + x_new # data_i
            data_points_O.append(data)
            k += 1

# print("data_points_O",len(data_points_O))
sim.finish_sim()

# Save data to a CSV file with headers
headers = ['data_id', 'xo', 'yo', 'vxo', 'vyo', 'xg', 'yg', 'thetag', 'vxg', 'vyg', 'omegag']
with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    # Write the headers first
    writer.writerow(headers)
    # Write the data
    writer.writerows(data_points_O)


# 2. Random sampling in the C-space
num_rand_data = 1000
data_points_rand_O = []

for i in range(num_rand_data):
    x_rand = generate_random_point(cbd)
    controlSpace = cage_planner.controlSpace()
    is_feasible = controlSpace.check_state_feasibility(x_rand) # check collision
    if is_feasible:
        data = [k,] + x_rand
        data_points_rand_O.append(data)
        k += 1
# print("data_points_rand_O",len(data_points_rand_O))

with open(filename, mode='a', newline='') as file:
    writer = csv.writer(file)
    writer.writerows(data_points_rand_O)
