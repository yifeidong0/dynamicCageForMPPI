from OpenGL.GL import *
from .geometric import *
from ..spaces.objective import *
from ..spaces.statespace import *
from ..spaces.configurationspace import *
from ..spaces.edgechecker import *
from ..spaces.metric import *
from ..planners.problem import PlanningProblem
from ..bullet.forward_simulator import *
import math

class CagePlannerControlSpace(ControlSpace):
    def __init__(self,cage):
        self.cage = cage
        self.dynamics_sim = forward_simulation()
        # self.movingObstacles = False
    def configurationSpace(self):
        return self.cage.configurationSpace()
    def controlSet(self,x):
        return MultiSet(TimeBiasSet(self.cage.time_range,self.cage.controlSet()),self.cage.controlSet())
        #return MultiSet(BoxSet([0],[self.cage.time_range]),self.cage.controlSet())
    def nextState(self,x,u):
        return self.eval(x,u,1.0)
    def eval(self,x,u,amount):
        xo,yo,vox,voy,xg,yg,thetag,vgx,vgy,omegag = x # state space, 10D (4: cage, 6: gripper)
        t,thrust_x,thrust_y,alpha = u # control space, 4D
        tc = t*amount

        # states = (1,1,0,0,
        #           1,0,0,0,0,0)
        # inputs = (2,0,10,.01)
        self.dynamics_sim.reset_states(x)
        new_states = self.dynamics_sim.run_forward_sim([tc,thrust_x,thrust_y,alpha])
        print(new_states)
        
        # net_acceler_x = thrust_x
        # net_acceler_y = self.cage.gravity + thrust_y
        # return [x_i+vx_i*tc+0.5*net_acceler_x*(tc**2), 
        #         y_i+vy_i*tc+0.5*net_acceler_y*(tc**2), 
        #         vx_i+net_acceler_x*tc,
        #         vy_i+net_acceler_y*tc,
        #         xr_i+self.cage.gripper_vel_x*tc,
        #         yr_i+self.cage.gripper_vel_y*tc,
        #         ]

        # x_new = 
        # return x_new
    
    def interpolator(self,x,u):
        return LambdaInterpolator(lambda s:self.eval(x,u,s),self.configurationSpace(),10)

class CagePlanner:
    def __init__(self):
        self.x_range = 10
        self.y_range = 10
        # self.min_altitude = 300
        self.max_velocity = 20
        self.max_acceleration = 10

        # Gripper moving velocity (constant)
        self.gripper_vel_x = 0.0
        self.gripper_vel_y = 3.0

        self.start_state = [2,2.1,0,0,2,2,0,0,0,0]
        self.goal_state = [8,8.1,0,0,8,8,0,0,0,0]
        self.goal_radius = 5
        self.time_range = 10
        #u = lambda:round(random.random())

        self.obstacles = []
        # self.obstacles = [
        #      (375, 200, 50, 200), 
        #      (575, 200, 50, 200), 
        #      (375, 400, 250, 50), 
        #      ]
        self.gravity = -9.8

    def controlSet(self):
        # return FiniteSet([[0],[1]])
        return BoxSet([-self.max_acceleration, -self.max_acceleration], 
                      [self.max_acceleration, self.max_acceleration],
                      [self.max_acceleration, self.max_acceleration])

    def controlSpace(self):
        # System dynamics
        return CagePlannerControlSpace(self)

    def workspace(self):
        # For visualization
        wspace = Geometric2DCSpace()
        wspace.box.bmin = [0,0]
        wspace.box.bmax = [self.x_range,self.y_range]
        wspace.addObstacleParam(self.obstacles)
        for o in self.obstacles:
            wspace.addObstacle(Box(o[0],o[1],o[0]+o[2],o[1]+o[3]))
        return wspace
    
    def configurationSpace(self):
        wspace = Geometric2DCSpace()
        wspace.box.bmin = [0,0]
        wspace.box.bmax = [self.x_range,self.y_range]
        wspace.addObstacleParam(self.obstacles)
        for o in self.obstacles:
            wspace.addObstacle(Box(o[0],o[1],o[0]+o[2],o[1]+o[3]))
        res =  MultiConfigurationSpace(wspace,
                                       BoxConfigurationSpace([-self.max_velocity],[self.max_velocity]), 
                                       BoxConfigurationSpace([-self.max_velocity],[self.max_velocity]),
                                       wspace,
                                       BoxConfigurationSpace([-math.pi],[math.pi]), 
                                       BoxConfigurationSpace([-self.max_velocity],[self.max_velocity]), 
                                       BoxConfigurationSpace([-self.max_velocity],[self.max_velocity]),
                                       BoxConfigurationSpace([-self.max_velocity],[self.max_velocity]),
                                       )
        return res

    def startState(self):
        return self.start_state

    def goalSet(self):
        r = self.goal_radius
        return BoxSet([self.goal_state[0]-r,self.goal_state[1]-r,
                       self.goal_state[2]-self.max_velocity,self.goal_state[3]-self.max_velocity, 
                       self.goal_state[4]-self.x_range/2, self.goal_state[5]-self.y_range/2],
                      [self.goal_state[0]+r,self.goal_state[1]+r,
                       self.goal_state[2]+self.max_velocity,self.goal_state[3]+self.max_velocity, 
                       self.goal_state[4]+self.x_range/2, self.goal_state[5]+self.y_range/2])


class CagePlannerObjectiveFunction(ObjectiveFunction):
    """Given a function pointwise(x,u), produces the incremental cost
    by incrementing over the interpolator's length.
    """
    def __init__(self,cage,timestep=0.2):
        self.cage = cage
        self.space = cage.controlSpace()
        self.timestep = timestep
    def incremental(self,x,u):
        e = self.space.interpolator(x,u)
        tmax = u[0]
        t = 0
        c = 0
        
        # Energy E_k+E_g total increase cost (BUG: root node is asked to be pruned without max)
        xnext = self.space.nextState(x,u)
        E = -self.cage.gravity*(self.cage.y_range-x[1]) + 0.5*(x[2]**2+x[3]**2)
        Enext = -self.cage.gravity*(self.cage.y_range-xnext[1]) + 0.5*(xnext[2]**2+xnext[3]**2)
        c = max((Enext-E), 0.0)

        return c


def CagePlannerTest():
    p = CagePlanner()
    objective = CagePlannerObjectiveFunction(p)
    return PlanningProblem(p.controlSpace(),p.startState(),p.goalSet(),
                           objective=objective,
                           visualizer=p.workspace(),
                           euclidean = True)


