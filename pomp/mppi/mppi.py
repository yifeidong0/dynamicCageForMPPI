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
import joblib
from tensorflow.keras.models import load_model
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
                 u_init=torch.tensor(1., dtype=torch.double),
                 dt=0.5):
                #  U_init=None):
        self.d = device
        self.K = K  # N_SAMPLES
        self.T = T  # HORIZON
        # self.I = 10 # N_ITERATIONS
        self.dt = dt
        self.num_vis_samples = 8 # nu. of rollouts

        # dimensions of state and control
        self.nx = nx
        self.nu = nu
        self.lambda_ = lambda_

        # # handle 1D edge case
        # if self.nu == 1:
        #     noise_mu = noise_mu.view(-1)
        #     noise_sigma = noise_sigma.view(-1, 1)

        self.noise_mu = noise_mu.to(self.d)
        self.noise_sigma = noise_sigma.to(self.d)
        self.noise_sigma_inv = torch.inverse(self.noise_sigma)
        self.noise_dist = MultivariateNormal(self.noise_mu, covariance_matrix=self.noise_sigma)
        # T x nu control sequence
        self.u_init = u_init
        self.U = u_init.repeat(self.T,1).to(self.d) + self.noise_dist.sample((self.T,)) # T x nu, initial control sequence

        # self.F = dynamics
        self.running_cost = running_cost
        self.terminal_state_cost = terminal_state_cost
        self.state = None
        self.rollout_state = None
        self.model = load_model('/home/yif/Documents/KTH/research/dynamicCaging/cage_metric_model.h5')
        self.scaler = joblib.load('/home/yif/Documents/KTH/research/dynamicCaging/cage_metric_scaler.pkl')

        self.cage = CagePlanner()
        self.control_space = self.cage.controlSpace()
        self.state_start = torch.tensor(self.cage.start_state)
        self.state_goal = torch.tensor(self.cage.goal_state)
        self.obj_goal_pos = self.state_goal[:2].clone().detach()
        self.goal_radius = self.cage.goal_radius
        self.u_boundary = self.cage.u_boundary
        self.c_space_boundary = self.cage.c_space_boundary
        self.radius_object = self.cage.radius_object
        self.half_extents_gripper = self.cage.half_extents_gripper

    def _reset_start_goal(self, params):
        xo_init, yo_init, xo_goal, yo_goal = params
        self.state_start = torch.tensor([xo_init,yo_init,0,0,xo_init,yo_init+self.radius_object+self.half_extents_gripper[1],0,0,0,0])
        self.state_goal = torch.tensor([xo_goal,yo_goal,0,0,0,0,0,0,0,0])
        self.obj_goal_pos = self.state_goal[:2].clone().detach()

    def _initialize_rollout_container(self):
        self.rollout_state = torch.zeros((self.num_vis_samples, self.T+1, self.nx))

    def _start_action_consideration(self):
        # reseample noise each time we take an action; these can be done at the start
        self.cost_total = torch.zeros(self.K, device=self.d)
        self.noise = self.noise_dist.sample((self.K, self.T))
        # cache action cost
        self.action_cost = self.lambda_ * self.noise @ self.noise_sigma_inv

    def _compute_total_cost(self, k, weight=1.0):
        save_rollouts = False
        if k < self.num_vis_samples: save_rollouts = True
        state = self.state.clone()
        if save_rollouts: 
            state_q, _ = self.control_space.toBulletStateInput(state)
            self.rollout_state[k,0,:] = torch.tensor(state_q) # (self.num_vis_samples, self.T+1, self.nx)

        for t in range(self.T):
            perturbed_action_t = self.U[t] + self.noise[k, t]

            # Clamping each element in u to its respective boundary
            for i in range(len(perturbed_action_t)): 
                perturbed_action_t[i] = torch.clamp(perturbed_action_t[i], min=self.u_boundary[0][i], max=self.u_boundary[1][i])
            
            perturbed_action_t[0] = self.dt # fixed time step

            # Rollout
            state = self.control_space.nextState(state, perturbed_action_t)
            
            # State space boundary
            # is_valid_state = is_within_boundaries(state, self.c_space_boundary)
            # if not is_valid_state:
            #     # print('is_valid_state NO')
            #     self.cost_total[k] += 1e8
            #     break

            # Cage cost TODO: the inference can be done in parallel once every K*T times
            # x_tran = self.scaler.transform([state])
            # cage_metric = self.model.predict(x_tran, verbose=0)
            # c_cage = 1/(1+max(cage_metric[0,0],1e-4))
            c_cage = 0.0
            state = torch.tensor(state)
            c_dis_to_goal = self.running_cost(state, perturbed_action_t, self.state_goal).item()
            decayed_weight = (t+1) / ((1+self.T)*self.T/2)
            c_goal = weight*decayed_weight*c_dis_to_goal
            self.cost_total[k] += (c_goal + c_cage)
            # print('c_goal', c_goal)

            # Add action perturbation cost
            # c_action = perturbed_action_t @ self.action_cost[k, t]
            # self.cost_total[k] += c_action
            # print('c_action', c_action)

            if save_rollouts: 
                state_q, _ = self.control_space.toBulletStateInput(state)
                self.rollout_state[k,t+1,:] = torch.tensor(state_q) # save rollouts

            # Break the for loop if current state is in goal
            if math.sqrt(c_dis_to_goal) < self.goal_radius:
                if save_rollouts and t+2 <= self.T: 
                    stacked_state_q = torch.tensor(state_q).unsqueeze(0).expand(self.T-(t+2)+1, -1)
                    self.rollout_state[k,t+2:,:] = stacked_state_q # save rollouts
                break

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
            self._compute_total_cost(k) # rollout dynamics and cost

        beta = torch.min(self.cost_total) # min cost
        cost_total_non_zero = self._ensure_non_zero(self.cost_total, beta, 1/self.lambda_)

        eta = torch.sum(cost_total_non_zero) # normalizer
        omega = (1. / eta) * cost_total_non_zero # weights of each sample
        for t in range(self.T): # retrieve the control sequence by importance sampling
            self.U[t] += torch.sum(omega.view(-1, 1) * self.noise[:, t], dim=0)
        action = self.U[0] # send to actuator

        # Shift command 1 time step
        self.U = torch.roll(self.U, -1, dims=0)
        self.U[-1] = self.u_init

        return action


def run_mppi(mppi, iter=10):
    state = mppi.state_start
    # mppi.I = iter
    mppi._initialize_rollout_container()
    xhist = torch.zeros((iter+1, mppi.nx))
    uhist = torch.zeros((iter, mppi.nu))
    gripperhist = torch.zeros((iter, 4, 2)) # 4 corners, 2d points
    state_q, _ = mppi.control_space.toBulletStateInput(state)
    xhist[0] = torch.tensor(state_q)
    gripperhist[0] = torch.tensor(get_gripper_corners(state_q[4:7], mppi.half_extents_gripper))
    goal = mppi.state_goal[:2].clone().detach()
    for t in range(iter):
        print('----------iter----------', t)
        action = mppi.command(state)
        action[0] = mppi.dt # fixed time step
        state = mppi.control_space.nextState(state, action)
        state = torch.tensor(state)
        # print('state x',state)
        # print('action x',action)
        cost = 0. # TODO: print cost

        # Log and visualize
        state_q, action_q = mppi.control_space.toBulletStateInput(state, action)
        print('state q',state_q)
        print('action q',action_q)
        xhist[t+1] = torch.tensor(state_q)
        gripperhist[t+1] = torch.tensor(get_gripper_corners(state_q[4:7], mppi.half_extents_gripper))
        uhist[t] = torch.tensor(action_q)
        if t % 1 == 0:
            visualize_mppi(mppi, xhist, gripperhist, t)

        # Check if goal is reached
        curr = state[:2] # object position
        dist_to_goal = torch.norm(goal-curr, p=2)
        print('dist_to_goal',dist_to_goal)
        if dist_to_goal < mppi.goal_radius:
            print('REACHED!')
            reached_goal = True
            visualize_mppi(mppi, xhist, gripperhist, t, reached_goal)
            break

        # Clean up the container
        mppi._initialize_rollout_container()


def visualize_mppi(mppi, xhist, gripperhist, t, reached_goal=False):
    if reached_goal: t += 1
    starto = copy.deepcopy(mppi.state_start[:2])
    goalo = copy.deepcopy(mppi.state_goal[:2]) # opengl frame
    goalo[1] = 10-goalo[1]
    goalo = goalo.tolist() # bullet frame
    goal_rad = mppi.goal_radius
    
    # Visualize the basic setup
    fig, ax = plt.subplots()

    if not reached_goal:
        ax.plot(mppi.rollout_state[:,:,0].T, mppi.rollout_state[:,:,1].T, 'k', alpha=0.3, zorder=3, linewidth=1) # rollout obj states
        ax.plot(mppi.rollout_state[0,:,0], mppi.rollout_state[0,:,1], 'k', alpha=0.02, label="rollouts")

    ax.plot([starto[0]], [10-starto[1]], 'ro', markersize=5, markerfacecolor='none', label="init obj")
    ax.plot([xhist[t, 0]], [xhist[t, 1]], 'ko', markersize=3, markerfacecolor='none', label="curr obj", zorder=5)
    c1 = plt.Circle(goalo, goal_rad, color='b', linewidth=1, fill=False, label="goal", zorder=7)
    ax.add_patch(c1)

    for i in range(t+1):
        draw_gripper(gripperhist[i], ax, alpha=float((i+1)/(t+1))) # gripper past poses
    ax.plot(xhist[:t+1,0], xhist[:t+1,1], 'ro-', markersize=3) # obj past trajectory

    # # # Show obstacles TODO: add obstacles.
    # # for obs_pos, obs_r in zip(obstacle_positions, obstacle_radius):
    # #   obs = plt.Circle(obs_pos, obs_r, color='k', fill=True, zorder=6)
    # #   ax.add_patch(obs)

    ax.set_xlim([-1.,11.0])
    ax.set_ylim([-1.,11.0])
    ax.legend(loc="upper left")
    ax.set_aspect("equal")
    plt.tight_layout()
    # plt.show()
    fig.savefig('mppi-plot-{}.png'.format(t), dpi=300)