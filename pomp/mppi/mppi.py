import sys
sys.path.append('/home/yif/Documents/KTH/git/dynamicCageForMPPI/pomp')
import torch
import time
import logging
from torch.distributions.multivariate_normal import MultivariateNormal
# from ..example_problems.cageplanner import CagePlannerControlSpace, CagePlanner
from ..example_problems.planepush import *
import copy
import math 
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import joblib
import torch.nn as nn
import time 
# from tensorflow.keras.models import load_model

logger = logging.getLogger(__name__)

# Function to check if x is within boundaries
def is_within_boundaries(x, x_boundary):
    for i in range(len(x)):
        if x[i] < x_boundary[i][0] or x[i] > x_boundary[i][1]:
            return False
    return True

# def get_gripper_corners(pose, half_extents_gripper):
#     # Calculate the corner points of the rotated box
#     half_length, half_height = half_extents_gripper
#     center_x, center_y, theta = pose
#     cos_theta = math.cos(theta)
#     sin_theta = math.sin(theta)

#     p1 = (center_x - cos_theta * half_length + sin_theta * half_height,
#             center_y + sin_theta * half_length + cos_theta * half_height)
    
#     p2 = (center_x + cos_theta * half_length + sin_theta * half_height,
#             center_y - sin_theta * half_length + cos_theta * half_height)

#     p3 = (center_x + cos_theta * half_length - sin_theta * half_height,
#             center_y - sin_theta * half_length - cos_theta * half_height)

#     p4 = (center_x - cos_theta * half_length - sin_theta * half_height,
#             center_y + sin_theta * half_length - cos_theta * half_height)
    
#     return torch.tensor([p1, p2, p3, p4]) # 4x2

# def draw_gripper(corners, ax, alpha=0.5, label=False):
#     points = corners.numpy() # 4x2
#     x, y = points[:, 0], points[:, 1]
#     # if label:
#     #     rect = patches.Rectangle((x[-1], y[-1]), 2*0.7, 2*0.4, edgecolor='green', facecolor='green', alpha=alpha, label='gripper', angle=)
#     # else:
#     for i in range(len(points)):
#         next_i = (i + 1) % len(points)  # To loop back to the first point
#         ax.plot([x[i], x[next_i]], [y[i], y[next_i]], 'g-', alpha=alpha)  # 'b-' for blue lines
#     #     rect = patches.Rectangle((x[-1], y[-1]), 2*0.7, 2*0.4, edgecolor='green', facecolor='green', alpha=alpha)
#     # ax.add_patch(rect)

def predict(model, input_data, scale_, min_):
    """
        input_data: torch.tensor(ndata,nx).cuda()
        scale_: torch.tensor(nx,).cuda()
        min_: torch.tensor(nx,).cuda()

        predictions: torch.tensor(ndata,).cuda()
    """
    # Preprocess the input data
    # input_data_scaled = scaler.transform(input_data)
    input_data_scaled = input_data * scale_ + min_
    
    # # Convert to PyTorch tensor
    # input_tensor = torch.tensor(input_data_scaled, dtype=torch.float32)

    # Perform prediction
    with torch.no_grad():
        predictions = model(input_data_scaled)
    
    return predictions


# Define the same model class used for training the cage stability network
class NeuralNetwork(nn.Module):
    def __init__(self):
        super(NeuralNetwork, self).__init__()
        self.linear_relu_stack = nn.Sequential(
            nn.Linear(10, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 1)
        )

    def forward(self, x):
        return self.linear_relu_stack(x)


class MPPI():
    """ MMPI according to algorithm 2 in Williams et al., 2017
        'Information Theoretic MPC for Model-Based Reinforcement Learning' """

    def __init__(self, 
                 nx, 
                 nu, 
                 cage,
                 K, 
                 T, 
                 running_cost, 
                 device="cpu", 
                 terminal_state_cost=None,
                 lambda_=1.,
                 goal_thres=0.1,
                 noise_mu=torch.tensor(0., dtype=torch.double),
                 noise_sigma=torch.tensor(1., dtype=torch.double),
                 u_init=torch.tensor(1., dtype=torch.double),
                 num_vis_samples=5,
                 dt=0.5):
        self.goal_thres = goal_thres
        self.cage = cage
        self.d = device
        self.K = K  # N_SAMPLES
        self.T = T  # HORIZON
        self.dt = dt
        self.num_vis_samples = num_vis_samples # nu. of rollouts
        self.theta_max = math.pi/2
        self.theta_min = -math.pi/2

        # dimensions of state and control
        self.nx = nx
        self.nu = nu
        self.lambda_ = lambda_

        self.noise_mu = noise_mu.to(self.d)
        self.noise_sigma = noise_sigma.to(self.d)
        self.noise_sigma_inv = torch.inverse(self.noise_sigma)
        self.noise_dist = MultivariateNormal(self.noise_mu, covariance_matrix=self.noise_sigma)
        # T x nu control sequence
        self.u_init = u_init
        self.U = u_init.repeat(self.T,1).to(self.d) + self.noise_dist.sample((self.T,)) # T x nu, initial control sequence

        self.running_cost = running_cost
        self.terminal_state_cost = terminal_state_cost
        self.state = None
        self.rollout_state = None
        # self.rollout_state_q = None
        self.rollout_cutdown_id = None

        # self._prepare_nn()

        # self.cage = CagePlanner()
        self.g = self.cage.gravity
        # self.time_range = self.cage.time_range
        self.control_space = self.cage.controlSpace()
        # self.state_start = torch.tensor(self.cage.start_state, device=self.d)
        # self.state_goal = torch.tensor(self.cage.goal_state, device=self.d)
        # self.obj_goal_pos = self.state_goal[:2].clone().detach().to(self.d)
        # self.goal_radius = self.cage.goal_radius
        self.u_boundary = [
                           [-self.cage.max_acceleration, self.cage.max_acceleration],
                           [-self.cage.max_acceleration, self.cage.max_acceleration],
                           [-self.cage.max_ang_acceleration, self.cage.max_ang_acceleration],
                           ]
        self.c_space_boundary = self.cage.c_space_boundary
        # self.c_space_boundary = [
        #     [-self.offset, 10.0],
        #     [0.0, 10.0],
        #     [-10.0, 10.0],
        #     [-10.0, 10.0],
        #     [-20.0, 20.0],
        #     [-20.0, 20.0],
        #     [-math.pi/2, math.pi/2],
        #     [-15.0, 15.0],
        #     [-15.0, 15.0],
        #     [-math.pi/2, math.pi/2],
        #     ]
        # self.radius_object = self.cage.radius_object
        # self.half_extents_gripper = self.cage.half_extents_gripper

    def _prepare_nn(self):
        scaler = joblib.load('data/9kdataset-from-mppi/scaler_minmax.pkl')

        # Convert the scaler parameters to PyTorch tensors
        self.scaler_scale = torch.tensor(scaler.scale_, dtype=torch.float32, device=self.d)
        self.scaler_min = torch.tensor(scaler.min_, dtype=torch.float32, device=self.d)

        self.model = NeuralNetwork()
        self.model.load_state_dict(torch.load('data/9kdataset-from-mppi/model_9k_1000epoch.pth'))
        self.model.to(self.d)
        self.model.eval()

    def _reset_start_goal(self, params):
        xo_init, yo_init, thetao_init, xg_init, yg_init = params
        self.state_start = torch.tensor([xo_init,yo_init,thetao_init,0,0,0,xg_init, yg_init,0,0,0,0], device=self.d) # dim=12
        # self.state_goal = torch.tensor([xo_goal,yo_goal,0,0,0,0,0,0,0,0], device=self.d)
        # self.obj_goal_pos = self.state_goal[:2].clone().detach()

    def _initialize_rollout_container(self):
        self.rollout_state = torch.zeros((self.K, self.T+1, self.nx), device=self.d)
        # self.rollout_state_q = torch.zeros((self.num_vis_samples, self.T+1, self.nx), device=self.d)
        self.rollout_cutdown_id = torch.zeros((self.K), device=self.d).fill_(self.T+1)

    def _start_action_consideration(self):
        # reseample noise each time we take an action; these can be done at the start
        self.cost_total = torch.zeros(self.K, device=self.d)
        self.noise = self.noise_dist.sample((self.K, self.T))
        # cache action cost
        self.action_cost = self.lambda_ * self.noise @ self.noise_sigma_inv

    def _compute_total_cost(self, k):
        vis_rollouts = False
        if k < self.num_vis_samples: vis_rollouts = True
        state = self.state.clone()
        # if vis_rollouts: # initialize
        #     state_q, _ = self.control_space.toBulletStateInput(state)
        #     self.rollout_state_q[k,0,:] = torch.tensor(state_q) # (self.num_vis_samples, self.T+1, self.nx)
        self.rollout_state[k,0,:] = state.clone().detach() # save rollouts for dataset generation

        for t in range(self.T):
            perturbed_action_t = self.U[t] + self.noise[k, t]

            # Clamping each element in u to its respective boundary
            for i in range(len(perturbed_action_t)): 
                perturbed_action_t[i] = torch.clamp(perturbed_action_t[i], min=self.u_boundary[i][0], max=self.u_boundary[i][1])
            
            # perturbed_action_t[0] = self.dt # fixed time step

            # Rollout
            state = self.control_space.nextState(state.tolist(), [self.dt,]+perturbed_action_t.tolist(), is_planner=True) # TODO: dynamics take 95% of the runtime
            # State space boundary
            is_valid_state = is_within_boundaries(state, self.c_space_boundary)
            if not is_valid_state:
                # print('is_valid_state NO')
                self.cost_total[k] += 1e5
                if vis_rollouts and t+1 <= self.T: 
                    # state_q, _ = self.control_space.toBulletStateInput(state)
                    # stacked_state_q = torch.tensor(state_q).unsqueeze(0).expand(self.T-(t+1)+1, -1)
                    # self.rollout_state_q[k,t+1:,:] = stacked_state_q # save rollouts for visualization
                    stacked_state = torch.tensor(state).unsqueeze(0).expand(self.T-(t+1)+1, -1)
                    self.rollout_state[k,t+1:,:] = stacked_state # save rollouts for visualization
                self.rollout_cutdown_id[k] = t + 1
                state = torch.tensor(state, device=self.d)
                break

            # Cage cost TODO: the inference can be done in parallel once every K*T times
            # increased_weight = (t+1) / ((1+self.T)*self.T/2)
            state = torch.tensor(state, device=self.d)

            cost_t = self.running_cost(state, perturbed_action_t).item()
            # c_goal = increased_weight * c_dis_to_goal
            # c_goal = c_dis_to_goal
            self.cost_total[k] += cost_t

            # Add action perturbation cost
            # c_action = perturbed_action_t @ self.action_cost[k, t]
            # self.cost_total[k] += c_action

            self.rollout_state[k,t+1,:] = state.clone().detach().to(self.d) # save rollouts for dataset generation
            # if vis_rollouts:
            #     state_q, _ = self.control_space.toBulletStateInput(state)
            #     self.rollout_state_q[k,t+1,:] = torch.tensor(state_q) # save rollouts for visualization
            #     # self.rollout_state[k,t+1,:] = state.clone().detach() # save rollouts for dataset generation

            # Break the for loop if current state is in goal
            # if math.sqrt(c_dis_to_goal) < self.goal_radius:
            if abs(cost_t) < self.goal_thres: # block already against the wall (PlanePush)
                if vis_rollouts and t+2 <= self.T: 
                    # stacked_state_q = torch.tensor(state_q).unsqueeze(0).expand(self.T-(t+2)+1, -1)
                    # self.rollout_state_q[k,t+2:,:] = stacked_state_q # save rollouts for visualization
                    stacked_state = torch.tensor(state).unsqueeze(0).expand(self.T-(t+2)+1, -1)
                    self.rollout_state[k,t+2:,:] = stacked_state # save rollouts for visualization
                if t+2 <= self.T: 
                    self.rollout_cutdown_id[k] = t+2
                break

        # this is the additional terminal cost (running state cost at T already accounted for)
        if self.terminal_state_cost:
            self.cost_total[k] += self.terminal_state_cost(state)

    def _add_cage_cost(self, cage_weight=1.):
        # Inference in batches
        # Create a mask to select the columns based on rollout_cutdown_id
        mask = torch.arange(self.rollout_state.shape[1], device=self.d).expand(self.rollout_state.shape[0], -1) < self.rollout_cutdown_id.view(-1, 1)

        # Apply the mask to state and reshape the result
        stacked_states = self.rollout_state[mask].reshape(-1, self.rollout_state.shape[-1]) # torch.Size([-1,self.nx]), gpu

        # Make a prediction
        stability_cage = predict(self.model, stacked_states, self.scaler_scale, self.scaler_min)
        # print('stability_cage', stability_cage[:int(self.rollout_cutdown_id[0].item())])
        cost_cage = cage_weight / (.01 + 2*torch.max(stability_cage, torch.tensor(1e-3))) # torch.Size([self.nx,1]), gpu

        # Calculate cumulative sum of rollout_cutdown_id
        cumulative_rollout = torch.cumsum(self.rollout_cutdown_id, dim=0)

        # Generate start indices of each group
        start_indices = cumulative_rollout - self.rollout_cutdown_id

        # Create a range tensor for comparison
        range_tensor = torch.arange(cumulative_rollout[-1], device=self.d)

        # Broadcast and create a mask for grouping
        group_mask = (range_tensor >= start_indices[:, None]) & (range_tensor < cumulative_rollout[:, None]) # torch.Size([nsample,ndata])

        # Sum the elements of cost_cage for each group
        cost_cage_total = torch.sum(cost_cage.reshape(1,-1) * group_mask, dim=1) # (self.K,)
        self.cost_total = self.cost_total + cost_cage_total # (self.K,)

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
            self._compute_total_cost(k) # rollout dynamics and cost TODO:95% of total runtime

        # Add penalization against unstable configurations
        # self._add_cage_cost()
        
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


def run_mppi(mppi, iter=10, episode=0):
    xhist = torch.zeros((iter+1, mppi.nx), device=mppi.d)
    uhist = torch.zeros((iter, mppi.nu), device=mppi.d)
    # gripperhist = torch.zeros((iter+1, 4, 2), device=mppi.d) # 4 corners, 2d points
    rollouts_hist = torch.zeros((iter, mppi.num_vis_samples, mppi.T+1, mppi.nx), device=mppi.d)
    cutdown_hist = torch.zeros((iter, mppi.num_vis_samples), device=mppi.d).fill_(mppi.T+1) # cutdown of #step in the horizon because reaching goals

    cutdown_iter = iter
    state = mppi.state_start
    mppi._initialize_rollout_container()
    # state_q, _ = mppi.control_space.toBulletStateInput(state)
    # xhist[0] = torch.tensor(state_q)
    xhist[0] = torch.tensor(state)
    # gripperhist[0] = get_gripper_corners(state_q[4:7], mppi.half_extents_gripper).clone().detach()
    # goal = mppi.state_goal[:2].clone().detach().to(mppi.d)
    for t in range(iter):
        print('----------iter----------', t)
        action = mppi.command(state)
        # action[0] = mppi.dt # fixed time step
        # state = mppi.control_space.nextState(state, action, is_planner=True)
        state = mppi.control_space.nextState(state.tolist(), [mppi.dt,]+action.tolist(), is_planner=True)

        # Keep theta within [-pi/2,pi/2]
        # if state[6] > mppi.theta_max:
        #     state[6] = (state[6] - mppi.theta_max) % math.pi + mppi.theta_min
        # elif state[6] < mppi.theta_min:
        #     state[6] = mppi.theta_max - (mppi.theta_min - state[6]) % math.pi
        state = torch.tensor(state, device=mppi.d)

        # stability_cage = predict(mppi.model, state.reshape(-1, mppi.nx), mppi.scaler_scale, mppi.scaler_min)[0,0]
        # print('ITER stability_cage', stability_cage)
        # cost = 0. # TODO: print cost

        # Log and visualize
        # state_q, action_q = mppi.control_space.toBulletStateInput(state, action)
        # xhist[t+1] = torch.tensor(state_q, device=mppi.d)
        # gripperhist[t+1] = get_gripper_corners(state_q[4:7], mppi.half_extents_gripper).clone().detach()
        # uhist[t] = torch.tensor(action_q, device=mppi.d)
        xhist[t+1] = torch.tensor(state, device=mppi.d)
        uhist[t] = torch.tensor(action, device=mppi.d)
        if t % 1 == 0:
            visualize_mppi(mppi, xhist, t, epi=episode)

        # Check if goal is reached
        # curr = state[:2] # object position
        # dist_to_goal = torch.norm(goal-curr, p=2)
        curr = state[1] # object position
        dist_to_goal = abs(mppi.cage.y_obstacle - curr)
        print('dist_to_goal',dist_to_goal)
        print('')
        print('!!!state',state)
        print('!!!mppi.c_space_boundary',mppi.c_space_boundary)
        print('!!!action',action)
        if dist_to_goal < mppi.goal_thres:
            print('REACHED!')
            cutdown_iter = t
            reached_goal = True
            visualize_mppi(mppi, xhist, t, reached_goal, epi=episode)
            break
        if not is_within_boundaries(state, mppi.c_space_boundary):
            print('OUT OF LIMIT!')
            cutdown_iter = t
            break

        # Save rollouts and clean up the container
        rollouts_hist[t,:,:,:] = mppi.rollout_state[:mppi.num_vis_samples,:,:]
        cutdown_hist[t,:] = mppi.rollout_cutdown_id[:mppi.num_vis_samples]
        mppi._initialize_rollout_container()

    return rollouts_hist, cutdown_hist, cutdown_iter


def visualize_mppi(mppi, xhist, t, reached_goal=False, epi=0):
    if reached_goal: t += 1
    # starto = copy.deepcopy(mppi.state_start[:2])
    # goalo = copy.deepcopy(mppi.state_goal[:2]) # opengl frame
    # goalo[1] = 10-goalo[1]
    # goalo = goalo.tolist() # bullet frame
    # goal_rad = mppi.goal_radius
    # Visualize the basic setup
    fig, ax = plt.subplots()

    if not reached_goal:
        ax.plot(mppi.rollout_state[:mppi.num_vis_samples,:,0].T.cpu(), 
                mppi.rollout_state[:mppi.num_vis_samples,:,1].T.cpu(), 
                'k', alpha=0.2, zorder=3, linewidth=1) # rollout obj states
        ax.plot(mppi.rollout_state[0,:,0].cpu(), mppi.rollout_state[0,:,1].cpu(), 'k', alpha=0.2, label="rollout")

    # ax.plot([starto[0].cpu()], [10-starto[1].cpu()], 'bo', markersize=5, markerfacecolor='none', label="object")
    # ax.plot([xhist[t, 0].cpu()], [xhist[t, 1].cpu()], 'bo', markersize=5, markerfacecolor='none', label="curr obj", zorder=5)
    # c1 = plt.Circle(goalo, goal_rad, color='orange', linewidth=1, fill=False, label="goal", zorder=7)
    # ax.add_patch(c1)

    # rect = patches.Rectangle((100, 100), 2*0.7, 2*0.4, edgecolor='green', linewidth=1, facecolor='none', label='gripper')
    # ax.add_patch(rect)
    # for i in range(t+1):
    #     label = True if i == 0 else False
    #     draw_gripper(gripperhist[i].cpu(), ax, alpha=float((i+1)/(t+1)), label=label) # gripper past poses
    ax.plot(xhist[:t+1,0].cpu().tolist(), xhist[:t+1,1].cpu().tolist(), 'b-', linewidth=1) # obj past trajectory
    ax.scatter(xhist[:t+1,0].cpu().tolist(), xhist[:t+1,1].cpu().tolist(), c='r', s=11, label="object") # obj past positions
    ax.scatter(xhist[:t+1,6].cpu().tolist(), xhist[:t+1,7].cpu().tolist(), c='g', s=8, label="gripper") # gripper past positions

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
    fig.savefig('mppi-plot-{}-{}.png'.format(epi, t), dpi=100)
    plt.close()