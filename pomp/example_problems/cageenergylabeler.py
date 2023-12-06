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

class CageELControlSpace(ControlSpace):
    def __init__(self,cage):
        self.cage = cage
        self.dynamics_sim = forwardSimulation(cage.params, gui=0)
        self.is_energy_labeler = True
        self.half_extents_gripper = cage.half_extents_gripper # [x,z]
        self.obstacles = self.cage.obstacles[0]

    def configurationSpace(self):
        return self.cage.configurationSpace()
    
    def controlSet(self, x):
        return MultiSet(TimeBiasSet(self.cage.time_range,self.cage.controlSet()),self.cage.controlSet())
    
    def nextState(self, x, u):
        return self.eval(x, u, 1.0)
    
    def eval(self, x, u, amount, theta_min=-math.pi, theta_max=math.pi, print_via_points=False):
        # x_i,y_i,vx_i,vy_i,xr_i,yr_i,thetar_i = x # state space, 7D (4: cage, 3: robot gripper)
        print('==x', x)
        print('==u', u)
        print('==amount', amount)
        t, ax, ay = u # control space
        tc = t * amount
        mu = [tc, ax, ay]
        print('==mu', mu)
        # net_acceler_x = thrust_x
        # net_acceler_y = self.cage.gravity + thrust_y

        # print('mu', mu)
        # print('amount', amount)
        xaug = x + self.cage.gripper_vel
        print('==xaug', xaug)
        self.dynamics_sim.reset_states(xaug)
        x_new, xo_via_points = self.dynamics_sim.run_forward_sim_labeler(mu, print_via_points)
        # print('xnew', x_new)

        # Make theta fall in [-pi/2, pi/2]
        # thetar_i = x_new[-1]
        # if thetar_i > theta_max:
        #     thetar_i = (thetar_i - theta_max) % (2*math.pi) + theta_min
        # elif thetar_i < theta_min:
        #     thetar_i = theta_max - (theta_min - thetar_i) % (2*math.pi)
        # x_new[-1] = thetar_i

        if print_via_points:
            self.xo_via_points = [[q[0], q[1]] for q in xo_via_points]

        return x_new[:7]

        # return [x_i+vx_i*tc+0.5*net_acceler_x*(tc**2), 
        #         y_i+vy_i*tc+0.5*net_acceler_y*(tc**2), 
        #         vx_i+net_acceler_x*tc,
        #         vy_i+net_acceler_y*tc,
        #         xr_i+self.cage.gripper_vel_x*tc,
        #         yr_i+self.cage.gripper_vel_y*tc,
        #         thetar_i
        #         ]
    
    def interpolator(self,x,u):
        print("cagelabeler interpolator")
        return LambdaInterpolator(lambda s:self.eval(x,u,s),self.configurationSpace(),10)

class CageEL:
    def __init__(self, data):
        self.x_range = 10
        self.y_range = 10
        self.max_velocity = 10
        self.max_acceleration = 2
        self.mass_object = .1 # import params from cageplanner
        self.half_extents_gripper = [.7, .4] # movement on x-z plane
        self.half_gripper_l = self.half_extents_gripper[0]
        self.half_gripper_w = self.half_extents_gripper[1]
        self.mass_gripper = 10
        self.moment_gripper = 1 # moment of inertia
        self.radius_object = 0.01
        self.params = [self.mass_object, self.mass_gripper, self.moment_gripper, 
                       self.half_extents_gripper, self.radius_object]

        # Gripper moving velocity (constant)
        self.gripper_vel = data[7:]
        self.gripper_vel_x = data[7]
        self.gripper_vel_y = data[8]
        self.gripper_vel_theta = data[9]

        self.start_state = data[:7]
        self.goal_state = [5, data[1]-1.5, 0, 0, 0, 0, 0] # varying goal region
        self.goal_radius = .2
        self.time_range = 1

        self.obstacles = [
             (data[4], data[5], data[6], self.half_gripper_l, self.half_gripper_w), # centerx,centery,length,width,orientation 
             ]
        self.gravity = -9.81

    # def checkStartFeasibility(self): # TODO; bullet collision checking
    #     gripper = AxisNotAlignedBox(self.obstacles[0][:3], self.obstacles[0][3:])
    #     contains = gripper.contains(self.start_state[:2])
    #     return contains

    def controlSet(self):
        return BoxSet([-self.max_acceleration, -self.max_acceleration], 
                      [self.max_acceleration, self.max_acceleration])

    def controlSpace(self):
        # System dynamics
        return CageELControlSpace(self)

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
                                       BoxConfigurationSpace([-self.max_velocity],[self.max_velocity]), 
                                       BoxConfigurationSpace([-self.max_velocity],[self.max_velocity]),
                                       BoxConfigurationSpace([-2.5*self.x_range],[2.5*self.x_range]),
                                       BoxConfigurationSpace([-2.5*self.y_range],[2.5*self.y_range]),
                                       BoxConfigurationSpace([-math.pi],[math.pi])
                                       ) # this c-space has to cover the state constraint in MPPI, better with some more margins
        return res

    def startState(self):
        return self.start_state

    def goalSet(self, offset=2):
        return BoxSet([-offset, -offset,
                       -self.max_velocity, -self.max_velocity, 
                       -2.5*self.x_range, -2.5*self.y_range, -math.pi],
                      [self.x_range+offset, self.goal_state[1],
                       self.max_velocity, self.max_velocity, 
                       2.5*self.x_range, 2.5*self.y_range, math.pi])


class CageELObjectiveFunction(ObjectiveFunction):
    """Given a function pointwise(x,u), produces the incremental cost
    by incrementing over the interpolator's length.
    """
    def __init__(self,cage,timestep=0.2):
        self.cage = cage
        self.space = cage.controlSpace()
        self.timestep = timestep

    def incremental(self,x,u):
        print("CageELObjectiveFunction incremental")
        m = self.cage.mass_object
        g = abs(self.cage.gravity)

        # Energy E_k+E_g total increase cost (BUG: root node is asked to be pruned without max)
        xnext = self.space.nextState(x,u)
        E = m*g*x[1] + 0.5*m*(x[2]**2+x[3]**2)
        Enext = m*g*xnext[1] + 0.5*m*(xnext[2]**2+xnext[3]**2)
        # c = max((Enext-E), 1e-3) + (2e-2)*(abs(u[0]) + abs(u[1]) + abs(u[2]))
        c = max((Enext-E), 1e-3)
        return c


def cageELTest():
    data = [1.02, 5.11, 0.00, 0,
            1.01, 4.70, -0.00, 0.00, 1, -0.50]
    p = CageEL(data)

    # if p.checkStartFeasibility():
    #     print('In collision!')
    #     return False
    objective = CageELObjectiveFunction(p)
    return PlanningProblem(p.controlSpace(),p.startState(),p.goalSet(),
                           objective=objective,
                           visualizer=p.workspace(),
                           euclidean = True)


