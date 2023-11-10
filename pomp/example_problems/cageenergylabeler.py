from OpenGL.GL import *
from .geometric import *
from ..spaces.objective import *
from ..spaces.statespace import *
from ..spaces.configurationspace import *
from ..spaces.edgechecker import *
from ..spaces.metric import *
from ..planners.problem import PlanningProblem
import math

class CageELControlSpace(ControlSpace):
    def __init__(self,cage):
        self.cage = cage
        self.is_energy_labeler = True
        self.half_extents_gripper = cage.half_extents_gripper # [x,z]
        self.obstacles = self.cage.obstacles[0]
    def configurationSpace(self):
        return self.cage.configurationSpace()
    def controlSet(self,x):
        return MultiSet(TimeBiasSet(self.cage.time_range,self.cage.controlSet()),self.cage.controlSet())
    def nextState(self,x,u):
        return self.eval(x,u,1.0)
    def eval(self,x,u,amount):
        x_i,y_i,vx_i,vy_i,xr_i,yr_i,thetar_i = x # state space, 7D (4: cage, 3: robot gripper)
        t,thrust_x,thrust_y = u # control space
        tc = t*amount
        net_acceler_x = thrust_x
        net_acceler_y = self.cage.gravity + thrust_y
        return [x_i+vx_i*tc+0.5*net_acceler_x*(tc**2), 
                y_i+vy_i*tc+0.5*net_acceler_y*(tc**2), 
                vx_i+net_acceler_x*tc,
                vy_i+net_acceler_y*tc,
                xr_i+self.cage.gripper_vel_x*tc,
                yr_i+self.cage.gripper_vel_y*tc,
                thetar_i+self.cage.gripper_vel_theta*tc,
                ]
    
    def interpolator(self,x,u):
        return LambdaInterpolator(lambda s:self.eval(x,u,s),self.configurationSpace(),10)

class CageEL:
    def __init__(self, data):
        self.x_range = 10
        self.y_range = 10
        self.max_velocity = 10
        self.max_acceleration = 20
        self.mass_object = 1 # import params from cageplanner
        self.half_extents_gripper = [.5, .1] # movement on x-z plane
        self.half_gripper_l = self.half_extents_gripper[0]
        self.half_gripper_w = self.half_extents_gripper[1]

        # Gripper moving velocity (constant)
        self.gripper_vel_x = data[7]
        self.gripper_vel_y = data[8]
        self.gripper_vel_theta = data[9]

        self.start_state = data[:7]
        self.goal_state = [5, 9.8, 0, 0, 0, 0, 0]
        self.goal_radius = .2
        self.time_range = 1

        self.obstacles = [
             (data[4], data[5], data[6], self.half_gripper_l, self.half_gripper_w), # centerx,centery,length,width,orientation 
             ]
        self.gravity = 9.81

    def controlSet(self):
        return BoxSet([-self.max_acceleration, -self.max_acceleration], 
                      [self.max_acceleration, self.max_acceleration])

    def controlSpace(self):
        # System dynamics
        return CageELControlSpace(self)

    def workspace(self):
        # For visualization
        wspace = Geometric2DCSpace()
        wspace.box.bmin = [0,0]
        wspace.box.bmax = [self.x_range,self.y_range]
        return wspace
    
    def configurationSpace(self):
        wspace = Geometric2DCSpace()
        wspace.box.bmin = [0,0]
        wspace.box.bmax = [self.x_range, self.y_range]
        wspace.addObstacleParam(self.obstacles)
        # for o in self.obstacles:
        #     wspace.addObstacle(Box(o[0],o[1],o[0]+o[2],o[1]+o[3]))
        res =  MultiConfigurationSpace(wspace,
                                       BoxConfigurationSpace([-self.max_velocity],[self.max_velocity]), 
                                       BoxConfigurationSpace([-self.max_velocity],[self.max_velocity]),
                                       BoxConfigurationSpace([0.0],[self.x_range]),
                                       BoxConfigurationSpace([0.0],[self.y_range]),
                                       BoxConfigurationSpace([-math.pi/2],[math.pi/2])
                                       )
        return res

    def startState(self):
        return self.start_state

    def goalSet(self):
        r = self.goal_radius
        return BoxSet([0,self.goal_state[1]-r,
                       self.goal_state[2]-self.max_velocity,self.goal_state[3]-self.max_velocity, 
                       0.0, 0.0, -math.pi/2],
                      [self.x_range,self.goal_state[1]+r,
                       self.goal_state[2]+self.max_velocity,self.goal_state[3]+self.max_velocity, 
                       self.x_range, self.y_range, math.pi/2])


class CageELObjectiveFunction(ObjectiveFunction):
    """Given a function pointwise(x,u), produces the incremental cost
    by incrementing over the interpolator's length.
    """
    def __init__(self,cage,timestep=0.2):
        self.cage = cage
        self.space = cage.controlSpace()
        self.timestep = timestep

    def incremental(self,x,u):
        m = self.cage.mass_object
        g = self.cage.gravity
        y_range = self.cage.y_range
        # Energy E_k+E_g total increase cost (BUG: root node is asked to be pruned without max)
        xnext = self.space.nextState(x,u)
        E = m*g*(y_range-x[1]) + 0.5*(x[2]**2+x[3]**2)
        Enext = m*g*(y_range-xnext[1]) + 0.5*(xnext[2]**2+xnext[3]**2)

        # c = max((Enext-E), 0.001)
        c = max((Enext-E), 0.001) + u[0]

        return c


def cageELTest(data):
    p = CageEL(data)
    objective = CageELObjectiveFunction(p)
    return PlanningProblem(p.controlSpace(),p.startState(),p.goalSet(),
                           objective=objective,
                           visualizer=p.workspace(),
                           euclidean = True)


