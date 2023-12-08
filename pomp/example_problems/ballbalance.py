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

class BallBalanceControlSpace(ControlSpace):
    def __init__(self,cage):
        self.cage = cage
        self.dynamics_sim = forwardSimulation(cage.params, gui=0)
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
        xo, yo, xr, yr, thetar = x # state space, 5D (2: cage, 3: robot gripper)
        t, vx, vy = u # control space
        tc = t*amount

        # Make theta fall in [-pi, pi]
        thetar = thetar + self.cage.gripper_vel_theta*tc
        if thetar > theta_max:
            thetar = (thetar - theta_max) % (2*math.pi) + theta_min
        elif thetar < theta_min:
            thetar = theta_max - (theta_min - thetar) % (2*math.pi)

        self.dynamics_sim.reset_states(x[:2]+u[1:]+x[2:]+self.cage.gripper_vel)
        x_new, xo_via_points = self.dynamics_sim.run_forward_sim_ball_balance(tc, print_via_points)

        if print_via_points:
            self.xo_via_points = [[q[0], q[1]] for q in xo_via_points]
        
        return x_new
    
    def interpolator(self, x, u, xnext=None):
        return LambdaInterpolator(lambda s:self.eval(x,u,s), self.configurationSpace(), 10, xnext=xnext)

class BallBalance:
    def __init__(self, data):
        self.x_range = 10
        self.y_range = 10
        self.max_velocity = 5
        self.max_acceleration = .7
        self.mass_object = 1 # import params from cageplanner
        self.half_extents_gripper = [.7, .4] # movement on x-z plane
        self.half_gripper_l = self.half_extents_gripper[0]
        self.half_gripper_w = self.half_extents_gripper[1]
        self.mass_gripper = 10
        self.moment_gripper = 1 # moment of inertia
        self.radius_object = 0.01
        self.params = [self.mass_object, self.mass_gripper, self.moment_gripper, 
                       self.half_extents_gripper, self.radius_object]

        # Gripper and object moving velocity
        self.object_vel_x = data[2]
        self.object_vel_y = data[3]
        self.gripper_vel = data[7:]
        self.gripper_vel_x = data[7]
        self.gripper_vel_y = data[8]
        self.gripper_vel_theta = data[9]

        self.start_state = data[:2] + data[4:7]
        self.start_vo = data[2:4]
        self.goal_state = [5, data[1]-1.5, 0, 0, 0] # varying goal region
        self.goal_radius = .2
        self.time_range = .7

        self.obstacles = [
             (data[4], data[5], data[6], self.half_gripper_l, self.half_gripper_w), # centerx,centery,length,width,orientation 
             ]
        self.gravity = 9.81

    def checkStartFeasibility(self):
        gripper = AxisNotAlignedBox(self.obstacles[0][:3], self.obstacles[0][3:])
        contains = gripper.contains(self.start_state[:2])
        return contains

    def controlSet(self):
        return BoxSet([-self.max_velocity, -self.max_velocity], 
                      [self.max_velocity, self.max_velocity])

    def controlSpace(self):
        # System dynamics
        return BallBalanceControlSpace(self)

    def workspace(self, offset=2):
        # For visualization
        wspace = Geometric2DCSpace()
        wspace.box.bmin = [0-offset,0-offset]
        wspace.box.bmax = [self.x_range+offset, self.y_range+offset]
        return wspace
    
    def configurationSpace(self, offset=2):
        wspace = Geometric2DCSpace()
        wspace.box.bmin = [-offset,-offset]
        wspace.box.bmax = [self.x_range+offset, self.y_range+offset]
        wspace.addObstacleParam(self.obstacles)
        # for o in self.obstacles:
        #     wspace.addObstacle(Box(o[0],o[1],o[0]+o[2],o[1]+o[3]))
        res =  MultiConfigurationSpace(wspace,
                                       BoxConfigurationSpace([-2.5*self.x_range],[2.5*self.x_range]),
                                       BoxConfigurationSpace([-2.5*self.y_range],[2.5*self.x_range]),
                                       BoxConfigurationSpace([-math.pi/2],[math.pi/2])
                                       ) # this c-space has to cover the state constraint in MPPI, better with some more margins
        return res

    def startState(self):
        return self.start_state

    def goalSet(self, offset=2):
        return BoxSet([-offset, -offset,
                       -2.5*self.x_range, -2.5*self.y_range, -math.pi/2],
                      [self.x_range+offset, self.goal_state[1],
                       2.5*self.x_range, 2.5*self.y_range, math.pi/2])


class BallBalanceObjectiveFunction(ObjectiveFunction):
    """Given a function pointwise(x,u), produces the incremental cost
    by incrementing over the interpolator's length.
    """
    def __init__(self,cage,timestep=0.2):
        self.cage = cage
        self.space = cage.controlSpace()
        self.timestep = timestep

    def incremental(self, x, u, uparent=None): # Node.uparent
        m = self.cage.mass_object
        g = self.cage.gravity
        xnext = self.space.nextState(x,u)
        self.xnext = xnext

        E = m*g*x[1] + 0.5*(uparent[1]**2+uparent[2]**2)
        Enext = m*g*xnext[1] + 0.5*(u[1]**2+u[2]**2) # TODO

        # Energy E_k+E_g total increase cost (BUG: root node is asked to be pruned without max)
        # c = max((Enext-E), 0.001)
        # c = max((Enext-E), 1e-5) + 1/(1+xnext[1])
        c = max((Enext-E), 1e-3)
        # print('c1',max((Enext-E), 1e-3))
        # print('c2',(1e-1)*(abs(u[0]) + abs(u[1]) + abs(u[2])))
        # c = max((Enext-E), 1e-3) + (2e-2)*(abs(u[0]) + abs(u[1]) + abs(u[2]))
        # c = abs(u[0]) + abs(u[1]) + abs(u[2])

        return c


def ballBalanceTest():
    data = [1.02, 5.11, 0.0, 1,
            1.01, 4.70, 0.0, 0.0, 1, 0]
    p = BallBalance(data)
    # if p.checkStartFeasibility():
    #     print('In collision!')
    #     return False
    objective = BallBalanceObjectiveFunction(p)
    return PlanningProblem(objective.space,p.startState(),p.goalSet(),
                           objective=objective,
                           visualizer=p.workspace(),
                           euclidean = True)


