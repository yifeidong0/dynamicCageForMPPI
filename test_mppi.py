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
    N_SAMPLES = 1000  # K
    N_ITER = 1000 # max no. of iterations
    nx = 10
    nu = 4
    lambda_ = 1.

    # ACTION_LOW = -2.0
    # ACTION_HIGH = 2.0
    gravity = 9.81
    d = "cuda"
    dtype = torch.double

    # noise_sigma = torch.tensor(10, device=d, dtype=dtype)
    # noise_mu = torch.tensor([0, 0], device=d, dtype=dtype)
    # u_init = torch.zeros_like(noise_mu)

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


    # # new hyperparmaeters for approximate dynamics
    # H_UNITS = 16
    # TRAIN_EPOCH = 150
    # BOOT_STRAP_ITER = 100

    # # network output is state residual
    # network = torch.nn.Sequential(
    #     torch.nn.Linear(nx + nu, H_UNITS),
    #     torch.nn.Tanh(),
    #     torch.nn.Linear(H_UNITS, H_UNITS),
    #     torch.nn.Tanh(),
    #     torch.nn.Linear(H_UNITS, nx)
    # ).double()

    # def dynamics(state, perturbed_action):
    #     u = torch.tensor([perturbed_action], dtype=torch.double, device=d)
    #     u = torch.clamp(u, ACTION_LOW, ACTION_HIGH)
    #     xu = torch.cat((state, u))
    #     state_residual = network(xu)
    #     next_state = state + state_residual
    #     next_state[0] = angle_normalize(next_state[0])
    #     return next_state


    # def angular_diff_batch(a, b):
    #     """Angle difference from b to a (a - b)"""
    #     d = a - b
    #     d[d > math.pi] -= 2 * math.pi
    #     d[d < -math.pi] += 2 * math.pi
    #     return d


    # def angle_normalize(x):
    #     return (((x + math.pi) % (2 * math.pi)) - math.pi)


    # dataset = None

    # def train(new_data):
    #     global dataset
    #     # not normalized inside the simulator
    #     new_data[:, 0] = angle_normalize(new_data[:, 0])
    #     if not torch.is_tensor(new_data):
    #         new_data = torch.from_numpy(new_data)
    #     # clamp actions
    #     new_data[:, -1] = torch.clamp(new_data[:, -1], ACTION_LOW, ACTION_HIGH)
    #     # append data to whole dataset
    #     if dataset is None:
    #         dataset = new_data
    #     else:
    #         dataset = torch.cat((dataset, new_data), dim=0)

    #     # train on the whole dataset (assume small enough we can train on all together)
    #     XU = dataset
    #     dtheta = angular_diff_batch(XU[1:, 0], XU[:-1, 0])
    #     dtheta_dt = XU[1:, 1] - XU[:-1, 1]
    #     Y = torch.cat((dtheta.view(-1, 1), dtheta_dt.view(-1, 1)), dim=1)  # x' - x residual
    #     XU = XU[:-1]  # make same size as Y

    #     # thaw network
    #     for param in network.parameters():
    #         param.requires_grad = True

    #     optimizer = torch.optim.Adam(network.parameters())
    #     for epoch in range(TRAIN_EPOCH):
    #         optimizer.zero_grad()
    #         # MSE loss
    #         Yhat = network(XU)
    #         loss = (Y - Yhat).norm(2, dim=1) ** 2
    #         loss.mean().backward()
    #         optimizer.step()
    #         logger.info("ds %d epoch %d loss %f", dataset.shape[0], epoch, loss.mean().item())

    #     # freeze network
    #     for param in network.parameters():
    #         param.requires_grad = False


    # downward_start = True
    # env = gym.make(ENV_NAME).env  # bypass the default TimeLimit wrapper
    # env.reset()
    # if downward_start:
    #     env.state = [np.pi, 1]

    # # bootstrap network with random actions
    # if BOOT_STRAP_ITER:
    #     logger.info("bootstrapping with random action for %d actions", BOOT_STRAP_ITER)
    #     new_data = np.zeros((BOOT_STRAP_ITER, nx + nu))
    #     for i in range(BOOT_STRAP_ITER):
    #         pre_action_state = env.state
    #         action = np.random.uniform(low=ACTION_LOW, high=ACTION_HIGH)
    #         env.step([action])
    #         # env.render()
    #         new_data[i, :nx] = pre_action_state
    #         new_data[i, nx:] = action

    #     train(new_data)
    #     logger.info("bootstrapping finished")

    # env = wrappers.Monitor(env, '/tmp/mppi/', force=True)
    # env.reset()
    # if downward_start:
    #     env.env.state = [np.pi, 1]


