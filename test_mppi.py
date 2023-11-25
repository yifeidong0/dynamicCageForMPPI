import sys
sys.path.append('/home/yif/Documents/KTH/git/dynamicCageForMPPI/mppi')
# import gym
import numpy as np
import torch
import logging
import math
from pomp.mppi.mppi import *
import random
import csv

# gym_log.set_level(gym_log.INFO)
# logger = logging.getLogger(__name__)
# logging.basicConfig(level=logging.DEBUG,
#                     format='[%(levelname)s %(asctime)s %(pathname)s:%(lineno)d] %(message)s',
#                     datefmt='%m-%d %H:%M:%S')

if __name__ == "__main__":
    N_EPISODE = 1000
    N_ITER = 30 # max no. of iterations
    N_SAMPLE = 1000  # K
    N_HORIZON = 15  # T, MPPI horizon
    nx = 10
    nu = 4
    dt = 0.15
    lambda_ = 1.
    gravity = 9.81
    d = "cuda"
    dtype = torch.double
    u_init = torch.tensor([0.5, 0.0, -1.1*gravity, 0.0], device=d, dtype=dtype) # in OpenGL frame
    noise_mu = torch.zeros_like(u_init, device=d, dtype=dtype)
    noise_sigma = .3 * torch.eye(nu, device=d, dtype=dtype)
    noise_sigma[2,2] = 0.5

    # For reproducibility
    # randseed = 5
    # if randseed is None:
    #     randseed = random.randint(0, 1000000)
    # random.seed(randseed)
    # np.random.seed(randseed)
    # torch.manual_seed(randseed)
    # print("random seed %d", randseed)

    def running_cost(state, action, state_goal):
        '''state and state_goal: torch.tensor()'''
        # weight = 1.
        cost = (state_goal[0]-state[0])**2 + (state_goal[1]-state[1])**2
        # cost = angle_normalize(theta) ** 2 + 0.1 * theta_dt ** 2 + 0.001 * action ** 2
        return cost

    mppi_gym = MPPI(nx, nu, K=N_SAMPLE, T=N_HORIZON, running_cost=running_cost, lambda_=lambda_,
                    noise_mu=noise_mu, noise_sigma=noise_sigma, u_init=u_init, dt=dt, device=d)

    # Save data to a CSV file with headers
    filename = "states_from_mppi.csv"
    headers = ['n_episode', 'n_iteration', 'n_sample', 'n_horizon', 'xo', 'yo', 'vxo', 'vyo', 'xg', 'yg', 'thetag', 'vxg', 'vyg', 'omegag']
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(headers)
        
    # Randomize start and goal
    x_range = 10
    y_range = 10
    randomize = 1
    for e in range(N_EPISODE):
        data = []
        if randomize:
            params = [(x_range-2)*random.random() + 1, # xo_init
                    (y_range-2)*random.random() + 1, # yo_init
                    (x_range-2)*random.random() + 1, # xo_goal
                    (y_range-2)*random.random() + 1, # yo_goal
                    ]
        else:
            params = [7,1,2,9]
        mppi_gym._reset_start_goal(params)

        rollouts_hist, cutdown_hist, cutdown_iter = run_mppi(mppi_gym, iter=N_ITER, episode=e)
        # rollouts_hist = torch.tensor((iter, mppi.num_vis_samples, mppi.T+1, mppi.nx))
        # cutdown_hist = torch.tensor((iter, mppi.num_vis_samples))

        # Process data to save
        for i in range(cutdown_iter):
            for s in range(mppi_gym.num_vis_samples):
                cutoff = int(cutdown_hist[i, s].item()) # cutoff in the horizon
                # print('cutoff',cutoff)
                selected_data = rollouts_hist[i, s, 1:cutoff, :]
                
                for h in range(cutoff-1):
                    selected_data_list = selected_data[h, :].tolist()
                    data.append([e, i, s, h,] + selected_data_list)
        
        # Save data to a CSV file
        with open(filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(data)