from .forwardsimulator import *
import random
import pybullet as p
import pybullet_data
import time
import math
import numpy as np

class scriptedMovementSimWaterSwing(forwardSimulationWaterSwing):
    def __init__(self, cage, gui=False):
        super().__init__(gui=gui)
        p.setGravity(0, 0, self.g)

        self.set_params(cage.params)
        self.create_shapes()

    def run_forward_sim(self, total_time=10, num_via_points=10):
        num_steps = int(total_time * 240)  # Number of time steps
        radius = 1.0  # Radius of the circular path
        angular_velocity = 2 * np.pi / total_time  # Radians per second for a full circle
        initial_angular_velocity = -2 * np.pi / total_time  # Initial velocity for a full circle
        vel_gripper = [angular_velocity, 0, 0]
        vel_angular_gripper = [0, initial_angular_velocity, 0]
        p.resetBaseVelocity(self.gripperUid, vel_gripper, vel_angular_gripper) # linear and angular vels both in world coordinates

        dt = total_time / num_steps

        interval = int(num_steps/num_via_points)
        interval = 3 if interval==0 else interval

        # Step the simulation
        via_points = []
        
        for t in range(num_steps):
            # Calculate the current angular displacement
            theta = angular_velocity * t * dt
            
            # Calculate centripetal acceleration
            centripetal_acceleration = (angular_velocity ** 2) / radius

            # Calculate force components along x and y axes (centripetal force)
            force_x = -centripetal_acceleration * self.mass_gripper * np.sin(theta)
            force_z = centripetal_acceleration*self.mass_gripper*np.cos(theta) + self.mass_gripper*(-self.g)

            # Apply external force
            self.pos_gripper,_ = p.getBasePositionAndOrientation(self.gripperUid)
            self.pos_object,_ = p.getBasePositionAndOrientation(self.objectUid)
            p.applyExternalForce(self.gripperUid, -1, 
                                [force_x, 0, force_z], 
                                self.pos_gripper, 
                                p.WORLD_FRAME)
            p.applyExternalForce(self.objectUid, -1, # keep object static
                                [0, 0, self.mass_object*(-self.g)], 
                                self.pos_object, 
                                p.WORLD_FRAME)
            # p.applyExternalTorque(self.gripperUid, -1, 
            #                      [0, self.moment_gripper*(-3), 0],
            #                      p.WORLD_FRAME)
            p.stepSimulation()

            # Print object via-points along the trajectory for visualization
            if t % interval == 0 or t == int(t*240)-1:
                # Get the object and gripper states
                self.pos_object, self.quat_object = p.getBasePositionAndOrientation(self.objectUid)
                self.eul_object = p.getEulerFromQuaternion(self.quat_object) # rad
                self.vel_object, self.vel_ang_object = p.getBaseVelocity(self.objectUid)
                self.pos_gripper, self.quat_gripper = p.getBasePositionAndOrientation(self.gripperUid)
                self.eul_gripper = p.getEulerFromQuaternion(self.quat_gripper)
                self.vel_gripper,self.vel_ang_gripper = p.getBaseVelocity(self.gripperUid)

                new_states = [self.pos_object[0], self.pos_object[2], self.eul_object[1],
                            self.vel_object[0], self.vel_object[2], self.vel_ang_object[1],
                            self.pos_gripper[0], self.pos_gripper[2], self.eul_gripper[1], 
                            self.vel_gripper[0], self.vel_gripper[2], self.vel_ang_gripper[1]
                            ]
                print(self.pos_gripper,"self.pos_gripper")
                via_points.append(new_states)

            if self.gui:
                time.sleep(2/240)

        return via_points

def check_bounds(x, bd):
    # Iterate over elements in x and corresponding bounds in bd
    for i, xi in enumerate(x):
        lower_bound, upper_bound = bd[i]
        if xi < lower_bound or xi > upper_bound:
            return False
    return True

def generate_random_point(bd):
    # Generate a random value for each dimension within its bounds
    random_point = []
    for bounds in bd:
        lower_bound, upper_bound = bounds
        random_value = np.random.uniform(lower_bound, upper_bound)
        random_point.append(random_value)
    return random_point
