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

class BoxPivotControlSpace(ControlSpace):
    def __init__(self, cage):
        self.cage = cage
        self.dynamics_sim = cage.dynamics_sim
        self.dynamics_sim.set_params(cage.params)
        self.dynamics_sim.create_shapes()
        self.obstacles = self.cage.obstacles
        self.is_box_pivot = True

    def configurationSpace(self):
        return self.cage.configurationSpace()
    
    def controlSet(self, x):
        return MultiSet(TimeBiasSet(self.cage.time_range,self.cage.controlSet()),self.cage.controlSet())
    
    def nextState(self, x, u):
        return self.eval(x, u, 1.0)
    
    def eval(self, x, u, amount, print_via_points=False):
        t, ax, omega = u # control space
        tc = t * amount
        mu = [tc, ax, omega]

        # xaug = x + self.cage.gripper_vel
        vxg1, vxg2 = self.cage.gripper_vel[:2]
        # omegag = self.cage.gripper_vel_theta
        # vxg, vyg = calculate_new_velocity(vxg_init, vyg_init, omegag, self.cage.start_state[8], x[8])
        xaug = x + [vxg1, vxg2]

        self.dynamics_sim.reset_states(xaug)
        x_new, xo_via_points = self.dynamics_sim.run_forward_sim(mu, 1)
        # print("@@@@x_new",x_new[2])

        # # Make theta fall in [-pi, pi]
        # x_new[2] = limit_angle_to_pi(x_new[2])
        # x_new[8] = limit_angle_to_pi(x_new[8])

        if print_via_points: # TODO
            self.xo_via_points = [[q[0], q[1]] for q in xo_via_points]

        return x_new[:self.cage.num_state]
    
    def interpolator(self, x, u, xnext=None):
        return LambdaInterpolator(lambda s:self.eval(x,u,s), self.configurationSpace(), 10, xnext=xnext)

class BoxPivot:
    def __init__(self, data, dynamics_sim):
        self.dynamics_sim = dynamics_sim
        self.num_state = 8 # box 6 + spring 2
        self.num_input = 2
        self.x_range = 12
        self.y_range = 12
        self.offset = 0.0 # extend the landscape
        self.max_velocity = 1
        self.max_ang_velocity = 3
        self.max_acceleration = 3.0
        self.max_ang_acceleration = 20 # might need to be 100
        self.mass_object = .3
        self.mass_gripper = 1e-1
        self.moment_object = 8e-1 # moment of inertia.  Solid - (I) = (1/12) * m * (a^2 + b^2) - a=b=4
        self.moment_gripper = 3e-3 # (2/5) * mass_ball * radius_ball**2
        self.params = [self.mass_object, self.moment_object, self.mass_gripper, self.moment_gripper]

        # Gripper moving velocity (constant)
        self.gripper_vel = data[self.num_state:] # spring 1,2 velocity
        # self.gripper_vel_x = data[9]
        # self.gripper_vel_y = data[10]
        # self.gripper_vel_theta = data[11]

        self.start_state = data[:self.num_state]
        self.goal_state = [0, 0, 0, 0, 0, 0, 0, 0] # varying goal region # TODO
        # self.goal_radius = .2 # MPPI goal radius
        # self.goal_half_extent = 1.5 # AO-xxx goal region
        self.time_range = .5

        self.obstacles = []
        self.gravity = -9.81

        # self.cspace_bound = [[0, self.x_range], 
        #                      [0, self.y_range], 
        #                      [-math.pi, math.pi], 
        #                      [-self.max_velocity, self.max_velocity],
        #                      [-self.max_velocity, self.max_velocity],
        #                      [-self.max_ang_velocity, self.max_ang_velocity],
        #                      [-self.x_range, 2*self.x_range], 
        #                      [-self.y_range, 2*self.y_range], 
        #                      [-math.pi, math.pi], 
        #                      ]

    def controlSet(self):
        return BoxSet([-self.max_acceleration, -1.0*self.max_ang_acceleration], 
                      [self.max_acceleration, 0.0*self.max_ang_acceleration])

    def controlSpace(self):
        # System dynamics
        return BoxPivotControlSpace(self)

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
                                       BoxConfigurationSpace([0.0*math.pi],[0.5*math.pi]),
                                       BoxConfigurationSpace([-self.max_velocity],[self.max_velocity]), 
                                       BoxConfigurationSpace([-self.max_velocity],[self.max_velocity]), 
                                       BoxConfigurationSpace([-self.max_ang_velocity],[self.max_ang_velocity]),
                                       BoxConfigurationSpace([0.0],[self.x_range]), # spring positions
                                       BoxConfigurationSpace([0.0],[self.y_range]),
                                       ) # this c-space has to cover the state constraint in MPPI, better with some more margins
        return res

    def startState(self):
        return self.start_state

    def goalSet(self):
        return BoxSet([-self.offset, -self.offset, 0.0*math.pi,
                       -self.max_velocity, -self.max_velocity, -self.max_ang_velocity,
                       -2.5*self.x_range, -2.5*self.x_range],
                      [self.x_range+self.offset, self.y_range+self.offset, (3e-2)*math.pi,
                       self.max_velocity, self.max_velocity, self.max_ang_velocity,
                       2.5*self.x_range, 2.5*self.x_range])


class BoxPivotObjectiveFunction(ObjectiveFunction):
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
        I = self.cage.moment_object # TODO
        # g = self.cage.gravity

        xnext = self.space.nextState(x,u)
        self.xnext = xnext

        # Energy
        # E = 0.5*m*(x[3]**2+x[4]**2) + 0.5*I*x[5]**2 + m*g*x[1]
        # Enext = 0.5*m*(xnext[3]**2+xnext[4]**2) + 0.5*I*xnext[5]**2 + m*g*xnext[1]
        # # c = max((Enext-E), 1e-3) + (2e-2)*(abs(u[0]) + abs(u[1]) + abs(u[2]))
        # c = max((Enext-E), 1e-5)

        # Work (applied force, torque and friction)
        delta_alpha = xnext[2] - x[2]
        # if delta_alpha < -math.pi:
        #     delta_alpha = 2*math.pi + delta_alpha
        # if delta_alpha > math.pi:
        #     delta_alpha = -2*math.pi + delta_alpha
        W = m*u[1]*(xnext[0]-x[0]) + I*u[2]*delta_alpha
        c = max(W, 1e-5)
        # c = max(abs(W), 1e-5)

        return c


def boxPivotTest(dynamics_sim,
                #  data=[6.088935001616721,2.085201582357198,0.04356701420972912,0.8179462670670236,0.7514566040947372,0.39286942983033646,
                #        1.2081902424651314,3.857239623005551,3.363011718477068,1.3836635048618797]
                 data=[6.499079904444838,2.397549737084281,0.22622950186326807,1.8207181866422655,1.1495647514494098,0.7609057703831511,
                         1.6157827276435577,4.439045449428635,0.20465756939330715,2.192905112691566]
                #  data=[8.078300255697453,2.827378659621455,0.8128189302617814,3.3118722652718695,-0.07453201727857538,1.1712296629792454,
                #        2.791803974737872,5.655365749729913,0.17408929947791152,1.435692406769087]
                 ):
    p = BoxPivot(data, dynamics_sim)

    # if p.checkStartFeasibility():
    #     print('In collision!')
    #     return False
    objective = BoxPivotObjectiveFunction(p)
    return PlanningProblem(objective.space,p.startState(),p.goalSet(),
                           objective=objective,
                           visualizer=p.workspace(),
                           euclidean = True)


