from pomp.example_problems.balancegrasp import *
from pomp.example_problems.planepush import *
from pomp.example_problems.waterswing import *
from pomp.example_problems.boxpivot import *
from pomp.example_problems.shuffling import *
from pomp.bullet.scriptedmovement import *
import time
import csv

problem_name = "PlanePush" # "Shuffling", "BoxPivot", "WaterSwing", "PlanePush", "BalanceGrasp"
total_time = 2.4
gui = 0
num_via_points = 10
num_trajs = 50
filename = "scripted_movement_viapoints_{}.csv".format(problem_name)
filename_metric = "scripted_movement_heuristics_{}.csv".format(problem_name)
filename_label = "scripted_movement_labels_{}.csv".format(problem_name)

if problem_name == 'BalanceGrasp':
    total_time = 2.5
    headers = ['num_traj', 'data_id', 'xo', 'yo', 'thetao', 'vxo', 'vyo', 'omegao', 'xg', 'yg', 'thetag', 'vxg', 'vyg', 'omegag']
    headers_metric = ['num_traj', 'data_id', 'shortest_distance', 'com_distance', 'max_contact_normal_force']
    fake_data = [5.0, 4.3, 0.0, 0.0, 0.0, 0.0, 
                 5.0, 4.0, 0.0, 0.0, 0.0, 0.0]
    dynamics_sim = forwardSimulationBalanceGrasp(gui=0)
    cage = BalanceGrasp(fake_data, dynamics_sim)
    x_init = fake_data
    dynamics_sim.finish_sim()
    sim = scriptedMovementSimBalanceGrasp(cage, gui=gui)
if problem_name == 'PlanePush':
    total_time = 2.3
    headers = ['num_traj', 'data_id', 'xo', 'yo', 'thetao', 'vxo', 'vyo', 'omegao', 'xg', 'yg', 'thetag', 'vxg', 'vyg', 'omegag']
    headers_metric = ['num_traj', 'data_id', 'shortest_distance', 'com_distance', 'max_contact_normal_force']
    fake_data = [5.0, 4.3, 0.0, 0.0, 0.0, 0.0, 
                 5.3, 4.0, 0.0, 0.0, 1.0, 0.0]
    dynamics_sim = forwardSimulationPlanePush(gui=0)
    cage = PlanePush(fake_data, dynamics_sim)
    x_init = fake_data
    dynamics_sim.finish_sim()
    sim = scriptedMovementSimPlanePush(cage, gui=gui)
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
if problem_name == 'Shuffling':
    total_time = 7.5
    headers = ['data_id', 'y', 'z', 'x', 'thetax', 'thetay', 'thetaz', 
               'vx', 'vy', 'vz', 'omegax', 'omegay', 'omegaz', 
               'j0', 'j1', 'j2', 'j3', 'jv0', 'jv1', 'jv2', 'jv3',
               'zg', 'vzg']
    num_state = len(headers)-1
    fake_data = [0.0, ]*num_state
    dynamics_sim = forwardSimulationShuffling(gui=0)
    cage = Shuffling(fake_data, dynamics_sim)
    x_init = [0, 4.55, 0, 0, np.pi/2, 0,
              0, 0, 0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0,
              5.2, 0
              ]
    dynamics_sim.finish_sim()
    sim = scriptedMovementSimShuffling(cage, gui=gui)

dataset = []
heuriset = []
labelset = []
for i in range(num_trajs):
    x_init = sim.sample_init_state()
    sim.reset_states(x_init)
    # time.sleep(2)
    _ = sim.run_forward_sim(num_via_points=1, do_cutdown_test=1) # get cutdown time
    sim.reset_states(x_init)
    x_news = sim.run_forward_sim(3, num_via_points)
    heuristics = sim.heuristics_traj
    for k in range(len(x_news)):
        # Check if inside C-space boundaries
        # is_valid = check_bounds(x_new, cbd)
        # if is_valid:
        dataset.append([i, k,] + x_news[k])
        heuriset.append([i, k,] + heuristics[k])
        k += 1
    labelset.append([i, sim.task_success_label])
sim.finish_sim()

# Save data to a CSV file with headers
with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(headers)
    writer.writerows(dataset)

# Save heuristics to a CSV file with headers
with open(filename_metric, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(headers_metric)
    writer.writerows(heuriset)

# Save labels to a CSV file with headers
with open(filename_label, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['num_traj', 'label'])
    writer.writerows(labelset)