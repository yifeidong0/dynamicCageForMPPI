from OpenGL.GL import *
from .geometric import *
from ..spaces.objective import *
from ..spaces.statespace import *
from ..spaces.configurationspace import *
from ..spaces.edgechecker import *
from ..spaces.metric import *
from ..planners.problem import PlanningProblem
import math

class BallBalanceControlSpace(ControlSpace):
    def __init__(self,cage):
        self.cage = cage
        self.is_energy_labeler = True
        self.half_extents_gripper = cage.half_extents_gripper # [x,z]
        self.obstacles = self.cage.obstacles[0]

    def configurationSpace(self):
        return self.cage.configurationSpace()
    
    def controlSet(self, x):
        return MultiSet(TimeBiasSet(self.cage.time_range,self.cage.controlSet()),self.cage.controlSet())
    
    def nextState(self, x, u):
        return self.eval(x, u, 1.0)
    
    def eval(self, x, u, amount, theta_min=-math.pi/2, theta_max=math.pi/2):
        x, y, xr, yr, thetar = x # state space, 5D (2: cage, 3: robot gripper)
        t, vx, vy = u # control space
        tc = t*amount

        # Make theta fall in [-pi/2, pi/2]
        thetar = thetar + self.cage.gripper_vel_theta*tc
        if thetar > theta_max:
            thetar = (thetar - theta_max) % math.pi + theta_min
        elif thetar < theta_min:
            thetar = theta_max - (theta_min - thetar) % math.pi
        
        return [x + vx*tc, 
                y + vy*tc, 
                xr + self.cage.gripper_vel_x*tc,
                yr + self.cage.gripper_vel_y*tc,
                thetar
                ]
    
    def interpolator(self,x,u):
        return LambdaInterpolator(lambda s:self.eval(x,u,s),self.configurationSpace(),10)

class BallBalance:
    def __init__(self, data):
        self.x_range = 10
        self.y_range = 10
        self.max_velocity = 20
        self.max_acceleration = 2
        self.mass_object = 1 # import params from cageplanner
        self.half_extents_gripper = [.7, .4] # movement on x-z plane
        self.half_gripper_l = self.half_extents_gripper[0]
        self.half_gripper_w = self.half_extents_gripper[1]

        # Gripper and object moving velocity
        self.object_vel_x = data[2]
        self.object_vel_y = data[3]
        self.gripper_vel_x = data[7]
        self.gripper_vel_y = data[8]
        self.gripper_vel_theta = data[9]

        self.start_state = data[:2] + data[4:7]
        self.goal_state = [5, data[1]+1.5, 0, 0, 0] # varying goal region
        self.goal_radius = .2
        self.time_range = .5

        self.obstacles = [
             (data[4], data[5], data[6], self.half_gripper_l, self.half_gripper_w), # centerx,centery,length,width,orientation 
             ]
        self.gravity = 9.81

    def checkStartFeasibility(self):
        gripper = AxisNotAlignedBox(self.obstacles[0][:3], self.obstacles[0][3:])
        contains = gripper.contains(self.start_state[:2])
        return contains

    def controlSet(self):
        return BoxSet([-self.max_acceleration, -self.max_acceleration], 
                      [self.max_acceleration, self.max_acceleration])

    def controlSpace(self):
        # System dynamics
        return BallBalanceControlSpace(self)

    def workspace(self, offset=1):
        # For visualization
        wspace = Geometric2DCSpace()
        wspace.box.bmin = [0-offset,0-offset]
        wspace.box.bmax = [self.x_range+offset, self.y_range+offset]
        return wspace
    
    def configurationSpace(self, offset=1):
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

    def goalSet(self, offset=1):
        return BoxSet([0.0-offset, self.goal_state[1],
                       -2.5*self.x_range, -2.5*self.y_range, -math.pi/2],
                      [self.x_range+offset, self.y_range+offset,
                       2.5*self.x_range, 2.5*self.y_range, math.pi/2])


class BallBalanceObjectiveFunction(ObjectiveFunction):
    """Given a function pointwise(x,u), produces the incremental cost
    by incrementing over the interpolator's length.
    """
    def __init__(self,cage,timestep=0.2):
        self.cage = cage
        self.space = cage.controlSpace()
        self.timestep = timestep

    def incremental(self,x,u): # uprev
        m = self.cage.mass_object
        g = self.cage.gravity
        y_range = self.cage.y_range
        # Energy E_k+E_g total increase cost (BUG: root node is asked to be pruned without max)
        xnext = self.space.nextState(x,u)
        # E = m*g*(y_range-x[1]) + 0.5*(uprev[1]**2+uprev[2]**2)
        E = m*g*(y_range-x[1]) + 0.5*(u[1]**2+u[2]**2)
        Enext = m*g*(y_range-xnext[1]) + 0.5*(u[1]**2+u[2]**2) # TODO

        # c = max((Enext-E), 0.001)
        # c = max((Enext-E), 1e-5) + 1/(1+xnext[1])
        # c = max((Enext-E), 0.001) + u[0]
        # print('c1',max((Enext-E), 1e-3))
        # print('c2',(1e-1)*(abs(u[0]) + abs(u[1]) + abs(u[2])))
        c = max((Enext-E), 1e-3) + (2e-2)*(abs(u[0]) + abs(u[1]) + abs(u[2]))

        return c


def ballBalanceTest():
    data = [1.0236332416534424,4.702088832855225,0.0011420216178521514,-0.2462615966796875,1.0122747421264648,
            5.112055778503418,-0.0020964997820556164,-0.14621029794216156,-0.2468520551919937,-0.028510550037026405]
    p = BallBalance(data)
    if p.checkStartFeasibility():
        print('In collision!')
        return False
    objective = BallBalanceObjectiveFunction(p)
    return PlanningProblem(p.controlSpace(),p.startState(),p.goalSet(),
                           objective=objective,
                           visualizer=p.workspace(),
                           euclidean = True)


