import sys
sys.path.append('/home/yif/Documents/KTH/git/dynamicCageForMPPI/mppi')
import numpy as np
import torch
import logging
import math
from pomp.mppi.mppi import *
import random
import csv

if __name__ == "__main__":
    problem_name = 'PlanePush'
    if problem_name == 'PlanePush':
        fake_data = [0.0,]*12
        dynamics_sim = forwardSimulationPlanePushPlanner(gui=0)
        cage = PlanePush(fake_data, dynamics_sim)
        goal_threshold = 0.3 # half of the width of the box
        init_state = [5.0, 4.3, 0.0, 0.0, 0.0, 0.0, 
                          5.0, 4.0, 0.0, 0.0, 0.0, 0.0]
    elif problem_name == 'BalanceGrasp':
        pass

    N_EPISODE = 100
    N_ITER = 25 # max no. of iterations
    N_SAMPLE = 150 # 1000  # K
    N_HORIZON = 20  # T, MPPI horizon
    nx = len(fake_data)
    nu = cage.nu - 1 # except time as the first element of action
    dt = .2 # fixed time step
    num_vis_samples = 1
    lambda_ = 1.
    gravity = 9.81
    d = "cuda"
    dtype = torch.double
    u_init = torch.tensor([0.0,]*3, device=d, dtype=dtype) # in OpenGL frame
    noise_mu = torch.zeros_like(u_init, device=d, dtype=dtype)
    noise_sigma = 1 * torch.eye(nu, device=d, dtype=dtype)
    do_bullet_vis = 0 if N_EPISODE > 1 else 1
    randomize = 1 if N_EPISODE > 1 else 1
    # noise_sigma[2,2] = 0.5

    # # For reproducibility
    # randseed = 5
    # if randseed is None:
    #     randseed = random.randint(0, 1000000)
    # random.seed(randseed)
    # np.random.seed(randseed)
    # torch.manual_seed(randseed)
    # print("random seed %d", randseed)

    def running_cost(state, action, w0=0.1, w1=0.02, w2=0.02, w3=0.01):
        '''state and state_goal: torch.tensor()'''
        cost = (w0 * (state[1]-cage.y_obstacle)**2 
                + w1 * (action[0]**2 + action[1]**2 + action[2]**2)
                + w2 * (state[3]**2 + state[4]**2 + state[5]**2 + state[9]**2 + state[10]**2 + state[11]**2)
                + w3 * (state[2]**2 + state[8]**2)) # orientation
        # cost = torch.Tensor([1e-9,])
        return cost

    def terminal_state_cost(state, weight=10.):
        '''state and state_goal: torch.tensor()'''
        cost_goal = weight * (state[1]-cage.y_obstacle)**2
        # cost_goal = torch.tensor(0.0, device='cuda:0')
        return cost_goal
    
    mppi_gym = MPPI(nx, 
                    nu, 
                    cage, 
                    K=N_SAMPLE, 
                    T=N_HORIZON, 
                    running_cost=running_cost, 
                    terminal_state_cost=terminal_state_cost, 
                    lambda_=lambda_, 
                    goal_thres=goal_threshold,
                    num_vis_samples=num_vis_samples, 
                    noise_mu=noise_mu, 
                    noise_sigma=noise_sigma, 
                    u_init=u_init, 
                    dt=dt, 
                    device=d)

    # Save data to a CSV file with headers
    filename = "states_from_mppi.csv"
    headers = ['n_episode', 'n_iteration', 'n_sample', 'n_horizon', 'xo', 'yo',  'thetao', 'vxo', 'vyo',  'omegao',
               'xg', 'yg', 'thetag', 'vxg', 'vyg', 'omegag', 'shortest_distance', 'S_stick', 'S_engage']
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(headers)
        
    # Randomize start and goal
    for e in range(N_EPISODE):
        print('##############iter#############', e)
        data = []
        if randomize and problem_name == 'PlanePush':
            xo = random.uniform(4,6)
            yo = random.uniform(5,7)
            # yo = random.uniform(3,5)
            thetao = random.uniform(-math.pi/6, math.pi/6)
            vxo = random.uniform(-0.0, 0.0)
            vyo = random.uniform(-0.0, 0.0)
            omegao = random.uniform(-0.0, 0.0)
            xg = xo + random.uniform(-0.6, 0.6)
            yg = yo + random.uniform(-1, -0.5)
            vxg = random.uniform(-0.0, 0.0)
            vyg = random.uniform(0, 0.2)
            init_state = [xo, yo, thetao, vxo, vyo, omegao,
                          xg, yg, 0, vxg, vyg, 0]
        mppi_gym._reset_start_goal(init_state)
        is_in_collsion = mppi_gym._check_collision()
        if is_in_collsion:
            continue

        rollouts_hist, cutdown_hist, cutdown_iter, rollouts_quality_hist = run_mppi(mppi_gym, iter=N_ITER, episode=e, do_bullet_vis=do_bullet_vis)

        # Process data to save
        for i in range(cutdown_iter):
            for s in range(mppi_gym.num_vis_samples):
                cutoff = int(cutdown_hist[i, s].item()) # cutoff in the horizon
                selected_data = rollouts_hist[i, s, 1:cutoff, :]
                heuristics = rollouts_quality_hist[i, s, 1:cutoff, :]
                for h in range(cutoff-1):
                    selected_data_list = selected_data[h, :].tolist()
                    heuristics_list = heuristics[h, :].tolist()
                    data.append([e, i, s, h,] + selected_data_list + heuristics_list)
        
        # Save data to a CSV file
        with open(filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(data)