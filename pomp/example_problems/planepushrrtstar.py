from OpenGL.GL import *
from .geometric import *
from ..spaces.objective import *
from ..spaces.statespace import *
from ..spaces.configurationspace import *
from ..spaces.edgechecker import *
from ..spaces.metric import *
from ..planners.problem import PlanningProblem
from ..bullet.forwardsimulator import *
from ..structures.toolfunc import *
import math

class PlanePushRrtstar:
    def __init__(self, data, dynamics_sim):
        self.dynamics_sim = dynamics_sim
        self.x_range = 10
        self.y_range = 10
        self.offset = 2.0 # extend the landscape
        # self.max_velocity = 10
        # self.max_ang_velocity = 2
        # self.max_acceleration = 10
        # self.max_ang_acceleration = 1
        self.mass_object = 1
        self.mass_gripper = 4
        self.moment_object = self.mass_gripper * 1e-1 # moment of inertia
        self.moment_gripper = self.mass_object * 1e-3
        self.y_obstacle = 7
        self.obstacle_borderline = [[-self.offset,self.y_obstacle], [self.x_range+self.offset, self.y_obstacle]]
        self.start_state = data[:3]
        self.gripper_pose = data[6:9]
        self.goal_state = [0,]*3
        self.params = [self.mass_object, self.moment_object, self.mass_gripper, self.moment_gripper, self.y_obstacle]
        
        self.dynamics_sim.set_params(self.params)
        self.dynamics_sim.create_shapes()
        self.dynamics_sim.set_gripper_pos(self.gripper_pose)

        self.obstacles = []
        self.gravity = -9.81

    def workspace(self):
        # For visualization
        wspace = Geometric2DCSpace()
        wspace.box.bmin = [-self.offset,-self.offset]
        wspace.box.bmax = [self.x_range+self.offset, self.y_range+self.offset]
        return wspace
    
    def configurationSpace(self):
        wspace = Geometric2DCSpace()
        wspace.box.bmin = [-self.offset,-self.offset]
        wspace.box.bmax = [self.x_range+self.offset, self.y_range+self.offset]
        wspace.addObstacleParam(self.obstacles)
        res = RRTstarMultiConfigurationSpace((wspace, BoxConfigurationSpace([-math.pi],[math.pi])), self.dynamics_sim)
        return res

    def startState(self):
        return self.start_state

    def goalSet(self):
        multibmin = [[-self.offset,-self.offset], [self.gripper_pose[0]+2, -self.offset],]
        multibmax = [[self.gripper_pose[0]-2, self.y_obstacle], [self.x_range+self.offset, self.y_obstacle],]
        bmin = [-math.pi,]
        bmax = [math.pi,]

        return MultiSet(UnionBoxSet(multibmin, multibmax),
                        BoxSet(bmin, bmax))


def potentialMetric(a, b):
    # return vectorops.distance(a,b)
    mg = 1
    c = mg * (a[1]-b[1])
    # print("!!!!!!c", b[1]-a[1])
    return max(c, 1e-5)

def PlanePushRrtstarTest(dynamics_sim, 
                  data = [5.0, 4.3, 0.0, 0.0, 0.0, 0, # point gripper with cylinder/box object
                          5.0, 4, 0.0, 0.0, 1.0, 0.0],
                #   data = [5.0, 4, 0.0, 0.0, 0, 0, # bowl gripper with cylinder object
                #           5.0, 4, 0.0, 0.0, 1, 0.0],
                          ):
    p = PlanePushRrtstar(data, dynamics_sim)
    return PlanningProblem(p.configurationSpace(), # for geometric planner - rrt*
                           p.startState(),
                           p.goalSet(),
                           objective=potentialMetric,
                           visualizer=p.workspace(),
                           euclidean=True)


