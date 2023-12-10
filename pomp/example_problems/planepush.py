from OpenGL.GL import *
from .geometric import *
from ..spaces.objective import *
from ..spaces.statespace import *
from ..spaces.configurationspace import *
from ..spaces.edgechecker import *
from ..spaces.metric import *
from ..planners.problem import PlanningProblem
from ..bullet.forwardsimulator import *
import math

def limit_angle_to_pi(angle, theta_min=-math.pi, theta_max=math.pi):
    # Make input angle fall in [-pi, pi]
    if angle > theta_max:
        theta = (angle - theta_max) % (2*math.pi) + theta_min
    elif angle < theta_min:
        theta = theta_max - (theta_min - angle) % (2*math.pi)
    return theta

class PlanePushControlSpace(ControlSpace):
    def __init__(self, cage):
        self.cage = cage
        self.dynamics_sim = cage.dynamics_sim
        self.dynamics_sim.set_params(cage.params)
        self.dynamics_sim.create_shapes()
        # self.is_energy_labeler = True
        self.half_extents_gripper = cage.half_extents_gripper # [x,z]
        self.obstacles = self.cage.obstacles[0]

    def configurationSpace(self):
        return self.cage.configurationSpace()
    
    def controlSet(self, x):
        return MultiSet(TimeBiasSet(self.cage.time_range,self.cage.controlSet()),self.cage.controlSet())
    
    def nextState(self, x, u):
        return self.eval(x, u, 1.0)
    
    def eval(self, x, u, amount, theta_min=-math.pi, theta_max=math.pi, print_via_points=False):
        x_i, y_i, theta_i, vx_i, vy_i, alpha_i, xr_i, yr_i, thetar_i = x # state space, 7D (3+3: cage, 3: robot gripper)
        t, ax, ay, alpha = u # control space
        tc = t * amount
        mu = [tc, ax, ay, alpha]

        xaug = x + self.cage.gripper_vel
        self.dynamics_sim.reset_states(xaug)
        x_new, xo_via_points = self.dynamics_sim.run_forward_sim_labeler(mu, 1)

        # Make theta fall in [-pi, pi]
        x_new[2] = limit_angle_to_pi(x_new[2])
        x_new[8] = limit_angle_to_pi(x_new[8])
        # thetar_i = x_new[8]
        # if thetar_i > theta_max:
        #     thetar_i = (thetar_i - theta_max) % (2*math.pi) + theta_min
        # elif thetar_i < theta_min:
        #     thetar_i = theta_max - (theta_min - thetar_i) % (2*math.pi)
        # x_new[8] = thetar_i

        if print_via_points: # TODO
            self.xo_via_points = [[q[0], q[1]] for q in xo_via_points]

        return x_new[:9]
    
    def interpolator(self, x, u, xnext=None):
        return LambdaInterpolator(lambda s:self.eval(x,u,s), self.configurationSpace(), 10, xnext=xnext)

class PlanePush:
    def __init__(self, data, dynamics_sim):
        self.dynamics_sim = dynamics_sim
        self.x_range = 10
        self.y_range = 10
        self.max_velocity = 5
        self.max_angle_velocity = 1
        self.max_acceleration = 2
        # self.half_extents_gripper = [.7, .4] # movement on x-z plane
        # self.half_gripper_l = self.half_extents_gripper[0]
        # self.half_gripper_w = self.half_extents_gripper[1]
        # self.radius_object = 0.01
        self.mass_object = .3
        self.mass_gripper = 1
        self.moment_object = 1 # moment of inertia
        self.moment_gripper = 1
        self.params = [self.mass_object, self.moment_object, self.mass_gripper, self.moment_gripper]

        # Gripper moving velocity (constant)
        self.gripper_vel = data[9:]
        self.gripper_vel_x = data[9]
        self.gripper_vel_y = data[10]
        self.gripper_vel_theta = data[11]

        self.start_state = data[:9]
        self.goal_state = [5, data[1]-1.5, 0, 0, 0, 0, 0, 0, 0] # varying goal region # TODO
        self.goal_radius = .2 # MPPI goal radius
        self.time_range = 1

        # self.obstacles = [
        #      (data[4], data[5], data[6], self.half_gripper_l, self.half_gripper_w), # centerx,centery,length,width,orientation 
        #      ]
        self.obstacles = []
        self.gravity = -9.81

    # def checkStartFeasibility(self): # TODO; bullet collision checking
    #     gripper = AxisNotAlignedBox(self.obstacles[0][:3], self.obstacles[0][3:])
    #     contains = gripper.contains(self.start_state[:2])
    #     return contains

    def controlSet(self):
        return BoxSet([-self.max_acceleration, -self.max_acceleration, -self.max_acceleration], 
                      [self.max_acceleration, self.max_acceleration, self.max_acceleration])

    def controlSpace(self):
        # System dynamics
        return PlanePushControlSpace(self)

    def workspace(self, offset=2):
        # For visualization
        wspace = Geometric2DCSpace()
        wspace.box.bmin = [-offset,-offset]
        wspace.box.bmax = [self.x_range+offset, self.y_range+offset]
        return wspace
    
    def configurationSpace(self, offset=2):
        wspace = Geometric2DCSpace()
        wspace.box.bmin = [-offset,-offset]
        wspace.box.bmax = [self.x_range+offset, self.y_range+offset]
        wspace.addObstacleParam(self.obstacles)
        res =  MultiConfigurationSpace(wspace,
                                       BoxConfigurationSpace([-math.pi],[math.pi]),
                                       BoxConfigurationSpace([-self.max_velocity],[self.max_velocity]), 
                                       BoxConfigurationSpace([-self.max_velocity],[self.max_velocity]),
                                       BoxConfigurationSpace([-self.max_angle_velocity],[self.max_angle_velocity]),
                                       BoxConfigurationSpace([-2.5*self.x_range],[2.5*self.x_range]),
                                       BoxConfigurationSpace([-2.5*self.y_range],[2.5*self.y_range]),
                                       BoxConfigurationSpace([-math.pi],[math.pi])
                                       ) # this c-space has to cover the state constraint in MPPI, better with some more margins
        return res

    def startState(self):
        return self.start_state

    def goalSet(self, offset=2):
        return BoxSet([-offset, -offset, -math.pi,
                       -self.max_velocity, -self.max_velocity, -self.max_angle_velocity,
                       -2.5*self.x_range, -2.5*self.y_range, -math.pi],
                      [self.x_range+offset, self.goal_state[1], math.pi,
                       self.max_velocity, self.max_velocity, -self.max_angle_velocity,
                       2.5*self.x_range, 2.5*self.y_range, math.pi])


class PlanePushObjectiveFunction(ObjectiveFunction):
    """Given a function pointwise(x,u), produces the incremental cost
    by incrementing over the interpolator's length.
    """
    def __init__(self, cage, timestep=0.2):
        self.cage = cage
        self.space = cage.controlSpace()
        self.timestep = timestep
        self.xnext = None

    def incremental(self, x, u, uparent=None):
        m = self.cage.mass_object
        I = self.cage.moment_object

        xnext = self.space.nextState(x,u)
        self.xnext = xnext
        E = 0.5*m*(x[3]**2+x[4]**2) + 0.5*I*x[5]**2
        Enext = 0.5*m*(xnext[3]**2+xnext[4]**2) + 0.5*I*xnext[5]**2
        
        # c = max((Enext-E), 1e-3) + (2e-2)*(abs(u[0]) + abs(u[1]) + abs(u[2]))
        c = max((Enext-E), 1e-3)
        return c


def planePushTest(dynamics_sim):
    data = [1.02, 5.11, 0, 0.00, 2, 0,
            1.01, 4.70, -0.00, 0.00, 2, 0.0]
    p = PlanePush(data, dynamics_sim)

    # if p.checkStartFeasibility():
    #     print('In collision!')
    #     return False
    objective = PlanePushObjectiveFunction(p)
    return PlanningProblem(objective.space,p.startState(),p.goalSet(),
                           objective=objective,
                           visualizer=p.workspace(),
                           euclidean = True)


