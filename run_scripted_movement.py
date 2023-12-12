from pomp.example_problems.cageenergylabeler import *
from pomp.example_problems.planepush import *
from pomp.example_problems.waterswing import *
from pomp.bullet.scriptedmovement import *
import time
import csv

problem_name = "WaterSwing"
total_time = 1.0
gui = 1
num_via_points = 10
num_trajs = 1
filename = "scripted_movement_viapoints_{}.csv".format(problem_name)

# cage_planner = CagePlanner()
# params = cage_planner.params
# y_range = cage_planner.y_range
# x_range = cage_planner.x_range
# half_extents_gripper = cage_planner.half_extents_gripper
# radius_object = cage_planner.radius_object
# g = cage_planner.gravity
# cbd = cage_planner.c_space_boundary
# ubd = cage_planner.u_boundary

if problem_name == 'CageEnergyLabeler':
    dynamics_sim = forwardSimulationEL(gui=0)
if problem_name == 'PlanePush':
    dynamics_sim = forwardSimulationPlanePush(gui=0)
if problem_name == 'WaterSwing':
    headers = ['data_id', 'xo', 'yo', 'thetao', 'vxo', 'vyo', 'omegao', 'xg', 'yg', 'thetag', 'vxg', 'vyg', 'omegag']
    fake_data = [3.0, 5.5, 0.0, 0.0, 0.0, 0,
                 3.0, 4.3, 0.0, 0.0, 0.0, 0.0]
    dynamics_sim = forwardSimulationWaterSwing(gui=0)
    cage = WaterSwing(fake_data, dynamics_sim)
    x_init = [0.2*cage.x_range/2, cage.y_range/3, 0, 0, 0, 0,
              cage.x_range/2, cage.y_range/7, 0, 0, 0, 0]
    dynamics_sim.finish_sim()
    sim = scriptedMovementSimWaterSwing(cage, gui=gui)
    
dataset = []
k = 0
for i in range(num_trajs):
    sim.reset_states(x_init)
    time.sleep(3)
    x_news = sim.run_forward_sim(total_time, num_via_points)
    for x_new in x_news:
        # Check if inside C-space boundaries
        # is_valid = check_bounds(x_new, cbd)
        # if is_valid:
        data = [k,] + x_new # data_i
        dataset.append(data)
        k += 1

sim.finish_sim()

# Save data to a CSV file with headers
with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(headers)
    writer.writerows(dataset)
