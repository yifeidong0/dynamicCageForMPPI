import sys
sys.path.append('/home/yif/Documents/KTH/git/dynamicCageForMPPI/mppi')
# import gym
import numpy as np
import torch
import logging
import math
from pomp.mppi.mppi import *
import random
# from gym import wrappers, logger as gym_log

# gym_log.set_level(gym_log.INFO)
# logger = logging.getLogger(__name__)
# logging.basicConfig(level=logging.DEBUG,
#                     format='[%(levelname)s %(asctime)s %(pathname)s:%(lineno)d] %(message)s',
#                     datefmt='%m-%d %H:%M:%S')

if __name__ == "__main__":
    # ENV_NAME = "Pendulum-v0"
    TIMESTEPS = 5  # T, MPPI horizon
    N_SAMPLES = 100 # 1000  # K
    N_ITER = 1000 # max no. of iterations
    nx = 10
    nu = 4
    lambda_ = 1.
    gravity = 9.81
    d = "cuda"
    dtype = torch.double
    u_init = torch.tensor([0.5, 0.0, -1.1*gravity, 0.0], device=d, dtype=dtype) # in OpenGL frame
    noise_mu = torch.zeros_like(u_init, device=d, dtype=dtype)
    noise_sigma = .03 * torch.eye(nu, device=d, dtype=dtype)
    noise_sigma[2,2] = 0.2

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
        weight = 1.
        cost = (state_goal[0]-state[0])**2 + (state_goal[1]-state[1])**2
        # cost = angle_normalize(theta) ** 2 + 0.1 * theta_dt ** 2 + 0.001 * action ** 2
        return weight * cost

    mppi_gym = MPPI(nx, nu, K=N_SAMPLES, T=TIMESTEPS, running_cost=running_cost, lambda_=lambda_,
                    noise_mu=noise_mu, noise_sigma=noise_sigma, u_init=u_init)
    # init_state = mppi_gym.state_start
    run_mppi(mppi_gym, iter=N_ITER)



