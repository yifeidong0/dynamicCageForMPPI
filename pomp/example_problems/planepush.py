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

class PlanePushControlSpace(ControlSpace):
    def __init__(self, cage):
        self.cage = cage
        self.dynamics_sim = cage.dynamics_sim
        self.dynamics_sim.set_params(cage.params)
        self.dynamics_sim.create_shapes()
        self.obstacles = self.cage.obstacles
        self.is_plane_push = True

    def configurationSpace(self):
        return self.cage.configurationSpace()
    
    def controlSet(self, x):
        return MultiSet(TimeBiasSet(self.cage.time_range,self.cage.controlSet()),self.cage.controlSet())
    
    def nextState(self, x, u):
        return self.eval(x, u, 1.0)
    
    def eval(self, x, u, amount, print_via_points=False):
        # x_i, y_i, theta_i, vx_i, vy_i, alpha_i, xr_i, yr_i, thetar_i = x # state space, 9D (3+3: cage, 3: robot gripper)
        t, ax, ay, omega = u # control space
        tc = t * amount
        mu = [tc, ax, ay, omega]

        xaug = x + self.cage.gripper_vel
        self.dynamics_sim.reset_states(xaug)
        x_new, xo_via_points = self.dynamics_sim.run_forward_sim(mu, 1)

        # Make theta fall in [-pi, pi]
        x_new[2] = limit_angle_to_pi(x_new[2])
        x_new[8] = limit_angle_to_pi(x_new[8])

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
        self.offset = 2.0 # extend the landscape
        self.max_velocity = 10
        self.max_ang_velocity = 2
        self.max_acceleration = 10
        self.max_ang_acceleration = 1
        self.mass_object = 1
        self.mass_gripper = 4
        self.moment_object = self.mass_gripper * 1e-1 # moment of inertia
        self.moment_gripper = self.mass_object * 1e-3
        self.y_obstacle = 7
        self.obstacle_borderline = [[-self.offset,self.y_obstacle], [self.x_range+self.offset, self.y_obstacle]]
        self.angle_slope = 0.0 * math.pi  # equivalent to on a slope
        self.params = [self.mass_object, self.moment_object, self.mass_gripper, self.moment_gripper, self.y_obstacle, self.angle_slope]

        # Gripper moving velocity (constant)
        self.gripper_vel = data[9:]
        self.gripper_vel_x = data[9]
        self.gripper_vel_y = data[10]
        self.gripper_vel_theta = data[11]

        self.start_state = data[:9]
        self.goal_state = [5, data[1]-1.5, 0, 0, 0, 0, 0, 0, 0] # varying goal region # TODO
        self.goal_radius = .2 # MPPI goal radius
        self.goal_half_extent = 1.5 # AO-xxx goal region
        self.time_range = 1

        self.obstacles = []
        self.gravity = -9.81

    # def checkStartFeasibility(self): # TODO; bullet collision checking
    #     gripper = AxisNotAlignedBox(self.obstacles[0][:3], self.obstacles[0][3:])
    #     contains = gripper.contains(self.start_state[:2])
    #     return contains

    def controlSet(self):
        return BoxSet([-self.max_acceleration, -self.max_acceleration, -self.max_ang_acceleration], 
                      [self.max_acceleration, self.max_acceleration, self.max_ang_acceleration])

    def controlSpace(self):
        # System dynamics
        return PlanePushControlSpace(self)

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
                                       BoxConfigurationSpace([-math.pi],[math.pi]),
                                       BoxConfigurationSpace([-self.max_velocity],[self.max_velocity]), 
                                       BoxConfigurationSpace([-self.max_velocity],[self.max_velocity]),
                                       BoxConfigurationSpace([-self.max_ang_velocity],[self.max_ang_velocity]),
                                       BoxConfigurationSpace([-2.5*self.x_range],[2.5*self.x_range]),
                                       BoxConfigurationSpace([-2.5*self.y_range],[2.5*self.y_range]),
                                       BoxConfigurationSpace([-math.pi],[math.pi])
                                       ) # this c-space has to cover the state constraint in MPPI, better with some more margins
        return res

    def startState(self):
        return self.start_state

    def goalSet(self):
        multibmin = [[-self.offset,-self.offset], [self.start_state[6]+2, -self.offset],]
        multibmax = [[self.start_state[6]-2, self.y_obstacle], [self.x_range+self.offset, self.y_obstacle],]
        bmin = [-math.pi, -self.max_velocity, -self.max_velocity, -self.max_ang_velocity, -2.5*self.x_range, -2.5*self.y_range, -math.pi]
        bmax = [math.pi, self.max_velocity, self.max_velocity, self.max_ang_velocity, 2.5*self.x_range, 2.5*self.y_range, math.pi]

        return MultiSet(UnionBoxSet(multibmin, multibmax),
                        BoxSet(bmin, bmax))


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
        """
        a. Law of Conservation of Energy: Energy cannot be created or destroyed, only transformed into one type or another. 
        (1) If there only conservative forces (such as gravity or elastic) then the total initial energy is equal to the total final energy.
            Ei = Ef
        (2) Non-conservative forces (such as friction, applied force and tension) can be used to add or remove energy to/from an object. 
        Thus, if there is work involved (i.e. a loss of energy or energy added to the system) then
            Ei + W = Ef

        b. Thermal Equilibrium Assumption: we assume that the object is in thermal equilibrium with its surroundings throughout the process. 
        In other words, the object's temperature remains nearly constant throughout the period when you apply a force or a torque.
        Thus, the heat generated by friction or negative work completely dissipated into the environment, and no a little is converted to 
        the internal energy of the object. 

        c. Along the escape path, there is piece-wise constant acceleration (u[1:]) applied to the object with a time duration of u[0]. For
        each segment, from the perspective of the object's energy, 
        (1) if the system energy (sum of kinetic and potential) goes up, positive work is done; 
        (2) if it goes down, negative work is done and the energy loss converts to heat in the environment.

        d. From the perspective of robot gripper's energy, why not think of the example that a human pushes a block on the table.
        (1) When a human applies positive work to accelerate a block on a table, the energy transferred to the block's kinetic energy and 
        the environment's thermal energy primarily comes from the human's metabolic energy.
        (2) When a human applies negative work (opposing force) to stop a moving block, they are essentially converting the block's kinetic 
        energy back into thermal energy due to friction and heat dissipation. (the human is not necessarily losing energy in the sense that 
        their metabolic energy is being depleted. Instead, the human's metabolic energy is converted into other forms ( e.g., thermal energy 
        due to muscle friction), but the total energy within the human system remains relatively constant over short time scales. )

        e. Work done by a virtual human is Wh = Ef_j-Ei_j-Wf. As a result, it seems to make sense to consider the incremental cost as 
        max(Wh, 0.0), Wf is the work done by friction, i.e. the overall cost as 
            c_all = integral_k (max(Wh, 0.0)),
        which means the energy required from a "virtual human"'s metabolic energy to move the object from init to final.
        """
        m = self.cage.mass_object
        I = self.cage.moment_object

        xnext = self.space.nextState(x,u)
        self.xnext = xnext

        # # Energy
        # E = 0.5*m*(x[3]**2+x[4]**2) + 0.5*I*x[5]**2
        # Enext = 0.5*m*(xnext[3]**2+xnext[4]**2) + 0.5*I*xnext[5]**2
        # # c = max((Enext-E), 1e-3) + (2e-2)*(abs(u[0]) + abs(u[1]) + abs(u[2]))
        # c = max((Enext-E), 1e-5)

        # Calculating change in position and orientation
        delta_x = xnext[0] - x[0]
        delta_y = xnext[1] - x[1]
        delta_theta = math.atan2(math.sin(xnext[2] - x[2]), math.cos(xnext[2] - x[2]))  # More robust angle difference

        # Work done by forces and torque
        W = m * u[1] * delta_x + m * u[2] * delta_y + I * u[3] * delta_theta

        # Considering both positive and negative work
        # c = W
        c = abs(W)
        
        # Include a small positive value to avoid zero cost in cases where it's needed
        return max(c, 1e-5)


def planePushTest(dynamics_sim, 
                #   data = [5.0, 4.3, 0.0, 0.0, 0.0, 0, # point gripper with cylinder/box object
                #           5.0, 4, 0.0, 0.0, 1.0, 0.0],
                  data = [5.0, 4, 0.0, 0.0, 0, 0, # bowl gripper with cylinder object
                          5.0, 4, 0.0, 0.0, 3, 0.0],
                ):
    p = PlanePush(data, dynamics_sim)

    # if p.checkStartFeasibility():
    #     print('In collision!')
    #     return False
    objective = PlanePushObjectiveFunction(p)
    return PlanningProblem(objective.space,
    # return PlanningProblem(p.configurationSpace(), # for geometric planner - rrt*
                           p.startState(),
                           p.goalSet(),
                           objective=objective,
                           visualizer=p.workspace(),
                           euclidean=True)


