import sys
sys.path.append('/home/yif/Documents/KTH/git/dynamicCageForMPPI/pomp')
import torch
import time
import logging
from torch.distributions.multivariate_normal import MultivariateNormal
from ..example_problems.cageplanner import CagePlannerControlSpace, CagePlanner
import copy
import math 
import matplotlib.pyplot as plt

logger = logging.getLogger(__name__)

# Function to check if x is within boundaries
def is_within_boundaries(x, x_boundary):
    for i in range(len(x)):
        if x[i] < x_boundary[i][0] or x[i] > x_boundary[i][1]:
            return False
    return True

def get_gripper_corners(pose, half_extents_gripper):
    # Calculate the corner points of the rotated box
    half_length, half_height = half_extents_gripper
    center_x, center_y, theta = pose
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)

    # TODO: Why it is wrong??
    # p1 = (center_x - cos_theta * half_length + sin_theta * half_height,
    #         center_y - sin_theta * half_length - cos_theta * half_height)
    
    # p2 = (center_x + cos_theta * half_length + sin_theta * half_height,
    #         center_y + sin_theta * half_length - cos_theta * half_height)

    # p3 = (center_x + cos_theta * half_length - sin_theta * half_height,
    #         center_y + sin_theta * half_length + cos_theta * half_height)

    # p4 = (center_x - cos_theta * half_length - sin_theta * half_height,
    #         center_y - sin_theta * half_length + cos_theta * half_height)

    p1 = (center_x - cos_theta * half_length + sin_theta * half_height,
            center_y + sin_theta * half_length + cos_theta * half_height)
    
    p2 = (center_x + cos_theta * half_length + sin_theta * half_height,
            center_y - sin_theta * half_length + cos_theta * half_height)

    p3 = (center_x + cos_theta * half_length - sin_theta * half_height,
            center_y - sin_theta * half_length - cos_theta * half_height)

    p4 = (center_x - cos_theta * half_length - sin_theta * half_height,
            center_y + sin_theta * half_length - cos_theta * half_height)
    
    return torch.tensor([p1, p2, p3, p4]) # 4x2

def draw_gripper(corners, ax, alpha=0.5):
    points = corners.numpy() # 4x2
    x, y = points[:, 0], points[:, 1]
    for i in range(len(points)):
        next_i = (i + 1) % len(points)  # To loop back to the first point
        ax.plot([x[i], x[next_i]], [y[i], y[next_i]], 'g-', alpha=alpha)  # 'b-' for blue lines

class MPPI():
    """ MMPI according to algorithm 2 in Williams et al., 2017
        'Information Theoretic MPC for Model-Based Reinforcement Learning' """

    def __init__(self, nx, nu, K, T, running_cost, device="cpu", terminal_state_cost=None,
                 lambda_=1.,
                 noise_mu=torch.tensor(0., dtype=torch.double),
                 noise_sigma=torch.tensor(1., dtype=torch.double),
                 u_init=torch.tensor(1., dtype=torch.double),):
                #  U_init=None):
        self.d = device
        self.K = K  # N_SAMPLES
        self.T = T  # TIMESTEPS

        # dimensions of state and control
        self.nx = nx
        self.nu = nu
        self.lambda_ = lambda_
        # self.nu = 1 if len(u_init.shape) == 0 else u_init.shape[0]

        # # handle 1D edge case
        # if self.nu == 1:
        #     noise_mu = noise_mu.view(-1)
        #     noise_sigma = noise_sigma.view(-1, 1)

        self.noise_mu = noise_mu.to(self.d)
        self.noise_sigma = noise_sigma.to(self.d)
        self.noise_sigma_inv = torch.inverse(self.noise_sigma)
        self.noise_dist = MultivariateNormal(self.noise_mu, covariance_matrix=self.noise_sigma)
        # T x nu control sequence
        # self.U = U_init # initial control sequence
        self.u_init = u_init
        # if self.U is None:
        self.U = u_init.repeat(self.T,1).to(self.d) + self.noise_dist.sample((self.T,)) # T x nu, initial control sequence

        # self.F = dynamics
        self.running_cost = running_cost
        self.terminal_state_cost = terminal_state_cost
        self.state = None

        self.cage = CagePlanner()
        self.control_space = self.cage.controlSpace()
        self.state_start = torch.tensor(self.cage.start_state)
        self.state_goal = torch.tensor(self.cage.goal_state)
        self.goal_radius = self.cage.goal_radius
        self.u_boundary = self.cage.u_boundary
        self.c_space_boundary = self.cage.c_space_boundary
        self.half_extents_gripper = self.cage.half_extents_gripper

    def _start_action_consideration(self):
        # reseample noise each time we take an action; these can be done at the start
        self.cost_total = torch.zeros(self.K, device=self.d)
        self.noise = self.noise_dist.sample((self.K, self.T))
        # cache action cost
        self.action_cost = self.lambda_ * self.noise @ self.noise_sigma_inv

    def _compute_total_cost(self, k):
        state = self.state.clone()
        for t in range(self.T):
            perturbed_action_t = self.U[t] + self.noise[k, t]
            # Clamping each element in u to its respective boundary
            for i in range(len(perturbed_action_t)):
                perturbed_action_t[i] = torch.clamp(perturbed_action_t[i], min=self.u_boundary[0][i], max=self.u_boundary[1][i])
            # print('state', state)
            # print('perturbed_action_t', perturbed_action_t)
            state = self.control_space.nextState(state, perturbed_action_t)
            state = torch.tensor(state)
            # is_valid_state = is_within_boundaries(state, self.c_space_boundary)
            # if not is_valid_state:
            #     # print('is_valid_state NO')
            #     self.cost_total[k] += 1e8
            #     break

            # # cage cost TODO
            # self.model = load_model('/home/yif/Documents/KTH/research/dynamicCaging/cage_metric_model.h5')
            # self.scaler = joblib.load('/home/yif/Documents/KTH/research/dynamicCaging/cage_metric_scaler.pkl')
            # xnext = self.space.nextState(x,u)
            # x_tran = self.scaler.transform([xnext])
            # cage_metric = self.model.predict(x_tran, verbose=0)

            self.cost_total[k] += self.running_cost(state, perturbed_action_t, self.state_goal).item()
            # add action perturbation cost
            # self.cost_total[k] += perturbed_action_t @ self.action_cost[k, t]

        # this is the additional terminal cost (running state cost at T already accounted for)
        if self.terminal_state_cost:
            self.cost_total[k] += self.terminal_state_cost(state)

    def _ensure_non_zero(self, cost, beta, factor):
        return torch.exp(-factor * (cost - beta))

    def command(self, state):
        """MPPI algorithm block II in the paper."""
        if not torch.is_tensor(state):
            state = torch.tensor(state)
        self.state = state.to(dtype=self.U.dtype, device=self.d) # get state estimate

        self._start_action_consideration()
        # TODO dynamics not easily parallelizable - refer to this https://openreview.net/pdf?id=fvfZKL1hCx for robot simulation
        for k in range(self.K):
            # print('sample id', k)
            self._compute_total_cost(k) # rollout dynamics and cost

        beta = torch.min(self.cost_total) # min cost
        cost_total_non_zero = self._ensure_non_zero(self.cost_total, beta, 1 / self.lambda_)

        eta = torch.sum(cost_total_non_zero) # normalizer
        omega = (1. / eta) * cost_total_non_zero # weights of each sample
        for t in range(self.T): # retrieve the control sequence by importance sampling - 
            self.U[t] += torch.sum(omega.view(-1, 1) * self.noise[:, t], dim=0)
        action = self.U[0] # send to actuator

        # shift command 1 time step
        self.U = torch.roll(self.U, -1, dims=0)
        self.U[-1] = self.u_init

        return action


def run_mppi(mppi, iter=10):
    # dataset = torch.zeros((retrain_after_iter, mppi.nx + mppi.nu), dtype=mppi.U.dtype, device=mppi.d)
    state = mppi.state_start
    xhist = torch.zeros((iter+1, mppi.nx))
    uhist = torch.zeros((iter, mppi.nu))
    gripperhist = torch.zeros((iter, 4, 2)) # 4 corners, 2d points
    state_q, _ = mppi.control_space.toBulletStateInput(state)
    xhist[0] = torch.tensor(state_q)
    gripperhist[0] = torch.tensor(get_gripper_corners(state_q[4:7], mppi.half_extents_gripper))
    for t in range(iter):
        print('----------iter----------', t)
        command_start = time.perf_counter()
        action = mppi.command(state)
        elapsed = time.perf_counter() - command_start
        # s, r, _, _ = env.step(action.numpy())
        state = mppi.control_space.nextState(state, action)
        state = torch.tensor(state)
        print('state x',state)
        print('action x',action)
        cost = 0. # TODO
        # logger.debug("action taken: %.4f cost received: %.4f time taken: %.5fs", action, cost, elapsed)
        # env.render()

        # Log and visualize
        state_q, action_q = mppi.control_space.toBulletStateInput(state, action)
        print('state q',state_q)
        print('action q',action_q)
        xhist[t+1] = torch.tensor(state_q)
        gripperhist[t+1] = torch.tensor(get_gripper_corners(state_q[4:7], mppi.half_extents_gripper))
        uhist[t] = torch.tensor(action_q)
        if t % 5 == 4:
            visualize_mppi(mppi, xhist, gripperhist, t)

        # Check if goal is reached
        curr = state[:2] # object position
        goal = mppi.state_goal[:2].clone().detach()
        dist_to_goal = torch.norm(goal-curr, p=2)
        print('dist_to_goal',dist_to_goal)
        if dist_to_goal < mppi.goal_radius:
            print('REACHED!')
            visualize_mppi(mppi, xhist, gripperhist, t)
            break

        # di = i % retrain_after_iter
        # if di == 0 and i > 0:
        #     retrain_dynamics(dataset)
        #     # don't have to clear dataset since it'll be overridden, but useful for debugging
        #     dataset.zero_()
        # dataset[di, :mppi.nx] = torch.tensor(state, dtype=mppi.U.dtype)
        # dataset[di, mppi.nx:] = action

def visualize_mppi(mppi, xhist, gripperhist, t):
    # print('xhist',xhist[t+1])
    starto = copy.deepcopy(mppi.state_start[:2])
    goalo = copy.deepcopy(mppi.state_goal[:2]) # opengl frame
    goalo[1] = 10-goalo[1]
    goalo = goalo.tolist() # bullet frame
    startg = mppi.state_start[4:6]
    goal_rad = mppi.goal_radius
    
    # Visualize the basic set up
    fig, ax = plt.subplots()
    ax.plot([starto[0]], [10-starto[1]], 'ro', markersize=5, markerfacecolor='none', label="init obj")
    # ax.plot([startg[0]], [10-startg[1]], 'go', markersize=5, markerfacecolor='none', label="StartG")
    ax.plot([xhist[t+1, 0]], [xhist[t+1, 1]], 'ko', markersize=3, markerfacecolor='none', label="curr obj", zorder=5)
    # ax.plot([xhist[t+1, 4]], [xhist[t+1, 5]], 'go', markersize=8, label="Curr. StateG", zorder=5)
    c1 = plt.Circle(goalo, goal_rad, color='b', linewidth=1, fill=False, label="goal", zorder=7)
    ax.add_patch(c1)

    # # # Show obstacles
    # # for obs_pos, obs_r in zip(obstacle_positions, obstacle_radius):
    # #   obs = plt.Circle(obs_pos, obs_r, color='k', fill=True, zorder=6)
    # #   ax.add_patch(obs)

    # # Get rollout states from subset of maps for visualization? (e.g., 50)
    # rollout_states_vis = mppi.get_state_rollout()
    
    ax.plot(xhist[:t+2,0], xhist[:t+2,1], 'ro', markersize=3)
    # ax.plot(xhist[:t+2,4], xhist[:t+2,5], 'go', markersize=3, label="Past StateG")
    for i in range(t+2):
        draw_gripper(gripperhist[i], ax, alpha=float((i+1)/(t+2)))
    # # ax.plot(rollout_states_vis[:,-1,0].T, rollout_states_vis[:,-1,1].T, 'r.', zorder=4)
    # ax.plot(rollout_states_vis[:,:,0].T, rollout_states_vis[:,:,1].T, 'k', alpha=0.5, zorder=3)
    # ax.plot(rollout_states_vis[0,:,0], rollout_states_vis[0,:,1], 'k', alpha=0.5, label="Rollouts")
    # ax.set_xlim(vis_xlim)
    # ax.set_ylim(vis_ylim)

    ax.set_xlim([0.,10.0])
    ax.set_ylim([0.,10.0])
    ax.legend(loc="upper right")
    ax.set_aspect("equal")
    plt.tight_layout()
    plt.show()