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
        self.mass_object = 1
        self.mass_gripper = 4
        self.moment_object = self.mass_gripper * 1e-1 # moment of inertia
        self.moment_gripper = self.mass_object * 1e-3
        self.y_obstacle = 8
        self.obstacle_borderline = [[-self.offset,self.y_obstacle], [self.x_range+self.offset, self.y_obstacle]]
        self.start_state = data[:3]
        self.gripper_pose = data[6:9]
        self.gripper_vel = data[9:]
        self.gripper_vel_x = data[9]
        self.gripper_vel_y = data[10]
        self.gripper_vel_theta = data[11]

        self.angle_slope = 1/6 * math.pi  # equivalent to on a slope
        self.params = [self.mass_object, self.moment_object, self.mass_gripper, self.moment_gripper, self.y_obstacle, self.angle_slope]
        
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

    def goalSet(self, goal_margin=1.2, default_radius=1000, t=6.0):
        bmin = [-math.pi,]
        bmax = [math.pi,]
        arcbmin = [-self.offset, -self.offset]
        arcbmax = [self.x_range+self.offset, self.y_range+self.offset]

        # Calculate arc center and radius
        gripper_velocity = np.linalg.norm(np.array([self.gripper_vel_x, self.gripper_vel_y]))
        arc_radius = gripper_velocity / abs(self.gripper_vel_theta) if self.gripper_vel_theta != 0 else default_radius
        if gripper_velocity >= 3e-1:
            # Calculate perpendicular direction to velocity
            perp_direction = [-self.gripper_vel_y, self.gripper_vel_x]
            perp_direction = perp_direction / np.linalg.norm(perp_direction)

            # Calculate arc center
            if self.gripper_vel_theta >= 0:
                arc_center = self.gripper_pose[:2] + perp_direction * arc_radius
            else:
                arc_center = self.gripper_pose[:2] - perp_direction * arc_radius
        else:
            # Handle linear motion case or set a default large radius
            arc_center = self.gripper_pose[:2]
            arc_radius = goal_margin
            arc_angle_range = [0.0, 2*np.pi-1e-9]
            return MultiSet(ArcErasedSet([arcbmin, arcbmax], goal_margin, arc_center, arc_radius, arc_angle_range),
                            BoxSet(bmin, bmax))
        
        # Calculate arc angle range
        angle_to_point = np.arctan2(self.gripper_pose[1] - arc_center[1], self.gripper_pose[0] - arc_center[0])

        # Normalize the angle_to_point within the range [0, 2π)
        initial_pos_angle = (angle_to_point + 2 * np.pi) % (2 * np.pi)

        if self.gripper_vel_theta > 0:
            # If the gripper is rotating counterclockwise
            arc_angle_range = [(initial_pos_angle - goal_margin/arc_radius) % (2*np.pi), 
                               (initial_pos_angle + self.gripper_vel_theta*t) % (2*np.pi)]
        elif self.gripper_vel_theta < 0:
            # If the gripper is rotating clockwise
            arc_angle_range = [(initial_pos_angle + self.gripper_vel_theta*t) % (2*np.pi), 
                               (initial_pos_angle + goal_margin/arc_radius) % (2*np.pi),]
        else:
            # If the gripper is not rotating
            arc_angle_range = [(initial_pos_angle-goal_margin/default_radius) % (2*np.pi), 
                               (initial_pos_angle+gripper_velocity*t/default_radius) % (2*np.pi)]

        print('arc_angle_range', arc_angle_range, 'arc_radius', arc_radius, 'arc_center', arc_center)
        return MultiSet(ArcErasedSet([arcbmin, arcbmax], goal_margin, arc_center, arc_radius, arc_angle_range),
                        BoxSet(bmin, bmax))

    def potentialMetric(self, a, b, is_potential_cost=True):
        """Potential-based metric for RRT* - edge cost from a to b"""
        if is_potential_cost:
            c = self.mass_object * abs(self.gravity) * math.sin(self.angle_slope) * (b[1]-a[1])
        else:
            c = euclideanMetric(a, b) # path length cost
        return max(c, 1e-5)

def PlanePushRrtstarTest(dynamics_sim, 
                #   data = [5.0, 4.3, 0.0, 0.0, 0.0, 0, # point gripper with cylinder/box object
                #           5.0, 4, 0.0, 0.0, 1.0, 0.0],
                  data = [5.0, 4, 0.0, 0.0, 0, 0, # bowl gripper with cylinder object
                          5.0, 4, 0.0, 0.0, 1, 0.0],
                          ):
    p = PlanePushRrtstar(data, dynamics_sim)
    return PlanningProblem(p.configurationSpace(), # for geometric planner - rrt*
                           p.startState(),
                           p.goalSet(),
                           objective=p.potentialMetric,
                           visualizer=p.workspace(),
                           euclidean=True)


