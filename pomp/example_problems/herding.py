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

class HerdingControlSpace(ControlSpace):
    def __init__(self, cage):
        self.cage = cage
        self.dynamics_sim = cage.dynamics_sim
        self.dynamics_sim.set_params(cage.params)
        self.dynamics_sim.create_shapes()
        self.obstacles = self.cage.obstacles
        self.is_herding = True

    def configurationSpace(self):
        return self.cage.configurationSpace()
    
    def controlSet(self, x):
        return MultiSet(TimeBiasSet(self.cage.time_range, self.cage.controlSet()),
                        self.cage.controlSet())
    
    def nextState(self, x, u):
        return self.eval(x, u, 1.0)
    
    def eval(self, x, u, amount, print_via_points=False):
        # x_i, y_i, vx_i, vy_i, (xr_i, yr_i)*nr = x # state space, (4+2nr) D
        t, ax, ay = u # control space
        tc = t * amount
        mu = [tc, ax, ay]

        xaug = x + self.cage.gripper_vel
        self.dynamics_sim.reset_states(xaug)
        x_new, xo_via_points = self.dynamics_sim.run_forward_sim(mu, 1)

        if print_via_points:
            self.xo_via_points = xo_via_points

        return x_new
    
    def interpolator(self, x, u, xnext=None):
        return LambdaInterpolator(lambda s:self.eval(x,u,s), self.configurationSpace(), 10, xnext=xnext)

class Herding:
    def __init__(self, data, dynamics_sim, num_robots=5):
        self.dynamics_sim = dynamics_sim
        self.num_robot = num_robots
        self.dim_workspace = 2
        self.dim_object = 4
        self.dim_state = self.dim_object + self.dim_workspace * self.num_robot
        self.x_range = 10
        self.y_range = 10
        self.offset = 4.0 # extend the landscape
        self.max_velocity = 10
        self.max_acceleration = 10
        self.mass_object = 1
        self.mass_gripper = 1 # has to be the SAME as the 4face-bottle.urdf file!
        self.params = [self.mass_object, self.mass_gripper, self.num_robot]

        # Gripper moving velocity (constant)
        self.gripper_vel = data[self.dim_state:] # list[10,]

        self.start_state = data[:self.dim_state]
        self.goal_state = [0,] * self.dim_state # varying goal region # TODO
        self.time_range = 0.5

        self.obstacles = []
        self.gravity = -9.81

    def controlSet(self):
        return BoxSet([-self.max_acceleration, -self.max_acceleration], 
                      [self.max_acceleration, self.max_acceleration])

    def controlSpace(self):
        # System dynamics
        return HerdingControlSpace(self)

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

        res =  MultiConfigurationSpace(wspace, 
                                       BoxConfigurationSpace([-self.max_velocity],[self.max_velocity]), 
                                       BoxConfigurationSpace([-self.max_velocity],[self.max_velocity]),
                                       *[BoxConfigurationSpace([-self.offset],[self.x_range+self.offset]),
                                         BoxConfigurationSpace([-self.offset],[self.y_range+self.offset])]*self.num_robot)
        return res

    def startState(self):
        return self.start_state

    def goalSet(self): # TODO: implement a ring set
        bmin = [-self.max_velocity, -self.max_velocity,] + [-self.offset, -self.offset,]*self.num_robot
        bmax = [self.max_velocity, self.max_velocity,] + [self.x_range+self.offset, self.y_range+self.offset,]*self.num_robot

        return MultiSet(RingSet([0.5*self.x_range, 0.5*self.y_range], 0.5*(self.x_range+self.offset), 0.5*self.x_range+self.offset), 
                        BoxSet(bmin, bmax))

class HerdingObjectiveFunction(ObjectiveFunction):
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

        # Roll out
        xnext = self.space.nextState(x,u)
        self.xnext = xnext

        # Energy
        # E = 0.5*m*(x[3]**2+x[4]**2) + 0.5*I*x[5]**2 + m*g*x[1]
        # Enext = 0.5*m*(xnext[3]**2+xnext[4]**2) + 0.5*I*xnext[5]**2 + m*g*xnext[1]
        # # c = max((Enext-E), 1e-3) + (2e-2)*(abs(u[0]) + abs(u[1]) + abs(u[2]))
        # c = max((Enext-E), 1e-5)

        # Work (applied force, torque and friction)
        W = m*u[1]*(xnext[0]-x[0]) + m*u[2]*(xnext[1]-x[1])
        c = max(W, 1e-5)

        return c

def form_a_random_cage(data, num_robot=5, radius=2.0):
    center = data[:2]
    # num_robot = len(data[4:]) // 4
    while True:
        # Generate num_robot equally spaced angles between 0 and 2pi
        angles = np.linspace(0, 2*np.pi, num_robot, endpoint=False)
        angles += np.random.rand(num_robot) * np.pi / num_robot / 1.4

        # Derive positions of robots
        positions = []
        for angle in angles:
            positions.append(center[0] + radius*np.cos(angle))
            positions.append(center[1] + radius*np.sin(angle))

        # Form a new data list
        data_new = data[:4] + positions + data[4+2*num_robot:]

        return data_new

def HerdingTest(dynamics_sim, data=None, num_robots=5):
    data = [5.0, 5.0, 0.0, 0.0,] + [0.0, 0.0,]*num_robots + [0.0, 0,0]*num_robots
    data = form_a_random_cage(data, num_robots)
    p = Herding(data, dynamics_sim, num_robots)

    objective = HerdingObjectiveFunction(p)
    return PlanningProblem(objective.space,
                           p.startState(),
                           p.goalSet(),
                           objective=objective,
                           visualizer=p.workspace(),
                           euclidean=True)

