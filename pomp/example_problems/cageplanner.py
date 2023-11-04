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
        self.cagePlanner = True
        self.gripper_size = self.dynamics_sim.half_extents_gripper # [x,y,z]
    def configurationSpace(self):
        return self.cage.configurationSpace()
    def controlSet(self,x):
        return MultiSet(TimeBiasSet(self.cage.time_range,self.cage.controlSet()),self.cage.controlSet())
        #return MultiSet(BoxSet([0],[self.cage.time_range]),self.cage.controlSet())
    def nextState(self,x,u):
        return self.eval(x,u,1.0)
    def toBulletStateInput(self, x, u):
        # O--->---------
        # |             | 
        # \/     *      | 
        # |    =====    | 
        # /\            | 
        #  |            | 
        # B--->---------
        q = [x[0],self.cage.y_range-x[1],
             x[2],-x[3],
             x[4],self.cage.y_range-x[5],-x[6],
             x[7],-x[8],-x[9]]
        mu = [u[0],u[1],-u[2],-u[3]]
        return q, mu
    
    def toOpenglStateInput(self, q):
        x = [q[0],self.cage.y_range-q[1],
             q[2],-q[3],
             q[4],self.cage.y_range-q[5],-q[6],
             q[7],-q[8],-q[9]]
        return x
         
    def eval(self,x,u,amount):
        # xo,yo,vox,voy,xg,yg,thetag,vgx,vgy,omegag = x # state space, 10D (4: cage, 6: gripper)
        t,thrust_x,thrust_y,alpha = u # control space, 4D
        tc = t*amount
        u = [tc,thrust_x,thrust_y,alpha]
        q, mu = self.toBulletStateInput(x, u)
        self.dynamics_sim.reset_states(q)
        # print("=========")
        # print("states x", x)
        # print("states q", q)
        # print("inputs u ", u)
        # print("inputs mu ", mu)
        q_new = self.dynamics_sim.run_forward_sim(mu)
        x_new = self.toOpenglStateInput(q_new)
        # print("q_new", q_new)
        # print("x_new", x_new)

        return x_new
    
    def interpolator(self,x,u):
        return LambdaInterpolator(lambda s:self.eval(x,u,s),self.configurationSpace(),10)

class CagePlanner:
    def __init__(self):
        self.x_range = 10
        self.y_range = 10
        self.max_velocity = 5
        self.max_acceleration = .3

        self.start_state = [2,2,0,0,2,2.1,0,0,0,0]
        self.goal_state = [8,8.1,0,0,0,0,0,0,0,0]
        self.goal_radius = .5
        self.time_range = 1
        #u = lambda:round(random.random())

        self.obstacles = [] # bar-gripper size 1.0*0.2
        self.gravity = 9.81 # downward in openGL vis

    def controlSet(self):
        return BoxSet([-self.max_acceleration, -self.gravity-self.max_acceleration, -.1], 
                      [self.max_acceleration, -self.gravity+self.max_acceleration, .1])

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
                                       BoxConfigurationSpace([0],[self.x_range]),
                                       BoxConfigurationSpace([0],[self.y_range]),
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
        return BoxSet([self.goal_state[0]-r, self.goal_state[1]-r,
                       -self.max_velocity, -self.max_velocity, 
                       0.0, 0.0, -math.pi,
                       -self.max_velocity, -self.max_velocity, -self.max_velocity],
                      [self.goal_state[0]+r, self.goal_state[1]+r,
                       self.max_velocity, self.max_velocity, 
                       self.x_range, self.y_range, math.pi,
                       self.max_velocity, self.max_velocity, self.max_velocity])


class CagePlannerObjectiveFunction(ObjectiveFunction):
    """Given a function pointwise(x,u), produces the incremental cost
    by incrementing over the interpolator's length.
    """
    def __init__(self,cage,timestep=0.2):
        self.cage = cage
        self.space = cage.controlSpace()
        self.timestep = timestep
    def incremental(self,x,u):
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


