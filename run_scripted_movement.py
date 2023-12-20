from pomp.example_problems.cageenergylabeler import *
from pomp.example_problems.planepush import *
from pomp.example_problems.waterswing import *
from pomp.example_problems.boxpivot import *
from pomp.bullet.scriptedmovement import *
import time
import csv

problem_name = "BoxPivot"
total_time = 2.5
gui = 1
num_via_points = 20
num_trajs = 1
filename = "scripted_movement_viapoints_{}.csv".format(problem_name)

# cage_planner = CagePlanner()
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
    x_init = [cage.x_range/2, cage.y_range/3-0.3, 0, 0, 0, 0,
              cage.x_range/2, cage.y_range/3, 0, 0, 0, 0]
    dynamics_sim.finish_sim()
    sim = scriptedMovementSimWaterSwing(cage, gui=gui)
if problem_name == 'BoxPivot':
    total_time = 2.
    headers = ['data_id', 'xo', 'yo', 'thetao', 'vxo', 'vyo', 'omegao', 'xg1', 'xg2', 'vxg1', 'vxg2']
    fake_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, ]
    dynamics_sim = forwardSimulationBoxPivot(gui=0)
    cage = BoxPivot(fake_data, dynamics_sim)
    x_init = [6, 2, 0, 0, 0, 0,
              0, 3, 0, 0]
    dynamics_sim.finish_sim()
    sim = scriptedMovementSimBoxPivot(cage, gui=gui)

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
