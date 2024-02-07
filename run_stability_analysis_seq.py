from pomp.bullet.scriptedmovement import *
from pomp.example_problems.cageplanner import *
from pomp.example_problems.waterswing import *
from pomp.example_problems.planepush import *
from pomp.example_problems.planepushrrtstar import *
from pomp.example_problems.balancegrasp import *
from pomp.example_problems.boxpivot import *
from pomp.example_problems.shuffling import *
from pomp.example_problems.gripper import *
from pomp.planners import allplanners
from pomp.visualizer import *
import time
import csv
from main import *
import os

# !!! Things remember to do BEFORE running: pruning, quasistatic_motion, pChooseGoal, densityEstimationRadius, max_dimensions (ESTprojection), goal sets, costs...
# !!! More non-maneuverable states needed in the 50 trajs

plannername = 'ao-est' # 'ao-est', 'rrt*', 'ao-rrt'
prname = 'PlanePush' # 'PlanePush', 'PlanePushRrtstar', 'BalanceGrasp', 'BoxPivot', 'Gripper', 'WaterSwing', 'Shuffling'
traj_type = 'scripted' # 'mppi', "scripted"
vis = 0
maxTime = 10000 # only used when vis=0
maxIters = 500
init_id = 4 if traj_type == 'mppi' else 2 # 0 for scripted, 2 for mppi

# Randomize the velocity and position of the objects and the friction coefficient - perturbation in estimated state
noise = 0.1
randomize_friction = 0
randomize_velocity = 0
randomize_position = 0
randomize_all = 1
fri_ratio, vel_ratio, pos_ratio = 1.0, 4.0, 2.0 # noise in [0,1]

if prname == 'PlanePush' or prname == 'PlanePushRrtstar':
    filenames = ['data/evaluation/perturbed_state_estimation/push_fixture/dataset/scripted_movement_viapoints_PlanePush.csv',]
    filename_friction = 'data/evaluation/perturbed_state_estimation/push_fixture/dataset/scripted_movement_maneuver_labels_PlanePush.csv'
    # filenames = ['data/18k_dataset_from_mppi/states_from_mppi.csv',]
    # filenames = ['data/evaluation/push_fixture/rand_traj_3/dataset/scripted_movement_viapoints_PlanePush.csv',]
if prname == 'BalanceGrasp':
    filenames = ['data/evaluation/balance_grasp/rand_traj_1/dataset/scripted_movement_viapoints_BalanceGrasp.csv',]
if prname == 'BoxPivot':
    filenames = ['data/evaluation/box_pivot/rand_fri_coeff/dataset/scripted_movement_viapoints_BoxPivot.csv',]
    filename_friction = 'data/evaluation/box_pivot/rand_fri_coeff/dataset/scripted_movement_maneuver_labels_BoxPivot.csv'
if prname == 'Gripper':
    filenames = ['data/evaluation/gripper/rand_objmass_fri/dataset/scripted_movement_viapoints_Gripper.csv',]
    filename_hyperparams = 'data/evaluation/gripper/rand_objmass_fri/dataset/scripted_movement_maneuver_labels_Gripper.csv'
# if prname == 'Shuffling':
#     filenames = ['data/shuffling/scripted_movement_viapoints_Shuffling.csv',]

def move_along_longer_side(pose, d):
    xo, yo, thetao, xg, yg = pose
    
    # Calculate the direction of the longer side of the box
    box_direction = np.array([np.cos(thetao), np.sin(thetao)])
    
    # Add offset d along the longer side of the box to the gripper's position
    new_xg = xg + d * box_direction[0]
    new_yg = yg + d * box_direction[1]
    
    return [xo, yo, thetao, new_xg, new_yg]

for filename in filenames:
    # Read from the CSV file
    rows = []
    ids = []
    with open(filename, 'r') as file:
        csv_reader = csv.reader(file)
        header = next(csv_reader)
        for id, row in enumerate(csv_reader):
            if traj_type == 'mppi':
                rows.append([float(d) for d in row[init_id:init_id+12]])
            else:
                rows.append([float(d) for d in row[init_id:]])
            ids.append(int(row[0]))

    if prname == 'PlanePush':
        fri_coeffs = []
        with open(filename_friction, 'r') as file:
            csv_reader = csv.reader(file)
            header = next(csv_reader)
            for id, row in enumerate(csv_reader):
                fri_coeffs.append(float(row[3]))
    if prname == 'BoxPivot':
        fri_coeffs = []
        with open(filename_friction, 'r') as file:
            csv_reader = csv.reader(file)
            header = next(csv_reader)
            for id, row in enumerate(csv_reader):
                fri_coeffs.append(float(row[3]))
    if prname == 'Gripper':
        fri_coeffs = []
        obj_mass = []
        with open(filename_hyperparams, 'r') as file:
            csv_reader = csv.reader(file)
            header = next(csv_reader)
            for id, row in enumerate(csv_reader):
                fri_coeffs.append(float(row[3]))
                obj_mass.append(float(row[4]))

    params = {'maxTime': maxTime}
    if 'maxTime' in params:
        del params['maxTime']

    t0 = time.time()

    for i, data_i in enumerate(rows):
        # Add velocity noise
        if randomize_velocity:
            data_i[3:6] = [d + random.uniform(-noise, noise) for d in data_i[3:6]]
            data_i[9:12] = [d + random.uniform(-noise, noise) for d in data_i[9:12]]
        # Add push point bias noise
        if randomize_position:
            pose_5d = data_i[:3] + data_i[6:8]
            new_pose_5d = move_along_longer_side(pose_5d, random.uniform(-noise, noise))
            data_i = new_pose_5d[:3] + data_i[3:6] + new_pose_5d[3:] + data_i[8:12]
        if randomize_all:
            fri_coeffs[i] = max(0.0, fri_coeffs[i] + random.uniform(-noise*fri_ratio, noise*fri_ratio))
            data_i[3:6] = [d + random.uniform(-noise*vel_ratio, noise*vel_ratio) for d in data_i[3:6]]
            data_i[9:12] = [d + random.uniform(-noise*vel_ratio, noise*vel_ratio) for d in data_i[9:12]]
            pose_5d = data_i[:3] + data_i[6:8]
            new_pose_5d = move_along_longer_side(pose_5d, random.uniform(-noise*pos_ratio, noise*pos_ratio))
            data_i = new_pose_5d[:3] + data_i[3:6] + new_pose_5d[3:] + data_i[8:12]
        if prname == 'PlanePush':
            dynamics_sim = forwardSimulationPlanePush(gui=0)
            problem = PlanePushTest(dynamics_sim, data_i, save_hyperparams=1, lateral_friction_coef=fri_coeffs[i])
        if prname == 'PlanePushRrtstar':
            dynamics_sim = forwardSimulationPlanePushRrtstar(gui=0)
            problem = PlanePushRrtstarTest(dynamics_sim, data_i, save_hyperparams=1)
        if prname == 'BalanceGrasp':
            dynamics_sim = forwardSimulationBalanceGrasp(gui=0)
            problem = BalanceGraspTest(dynamics_sim, data_i, save_hyperparams=1)
        if prname == 'BoxPivot':
            dynamics_sim = forwardSimulationBoxPivot(gui=0)
            problem = BoxPivotTest(dynamics_sim, data_i, save_hyperparams=1, lateral_friction_coef=fri_coeffs[i])
        if prname == 'Gripper':
            dynamics_sim = forwardSimulationGripper(gui=0)
            problem = GripperTest(dynamics_sim, data_i, save_hyperparams=1, lateral_friction_coef=fri_coeffs[i], mass_object=obj_mass[i])
        # if prname == 'WaterSwing':
        #     dynamics_sim = forwardSimulationWaterSwing(gui=0)
        #     problem = waterSwingTest(dynamics_sim, data_i)
        # if prname == 'Shuffling':
        #     dynamics_sim = forwardSimulationShuffling(gui=0)
        #     problem = ShufflingTest(dynamics_sim, data_i)
        if vis:
            runVisualizer(problem, type=plannername, **params)
        else:
            print(ids[i])
            testPlannerDefault(problem, prname, maxTime, maxIters, plannername, data_id=ids[i], **params)
        dynamics_sim.finish_sim()
    t_elapsed = time.time() - t0
    print('Time elapsed: ', t_elapsed)