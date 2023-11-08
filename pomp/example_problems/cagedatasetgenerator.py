from ..bullet.forwardsimulator import forwardSimulation
import random
import pybullet as p
import pybullet_data
import time
import math
import numpy as np

class dataGeneratorSim(forwardSimulation):
    def __init__(self, params, gui=False):
        super().__init__(params, gui=gui)

    def run_forward_sim(self, inputs, num_via_points=10):
        t, ax, az, alpha = inputs

        # Step the simulation
        via_points = []
        
        for i in range(int(t*240)):
            # Apply external force
            p.applyExternalForce(self.gripperUid, -1, 
                                [self.mass_gripper*ax, 0, self.mass_gripper*az], 
                                [self.moment_gripper*alpha, 0, 0], 
                                p.LINK_FRAME)
            p.stepSimulation()

            # Print object via-points along the trajectory for visualization
            interval = int(int(t*240)/num_via_points)
            interval = 3 if interval==0 else interval
            if i % interval == 0 or i == int(t*240)-1:
                # Get the object and gripper states
                pos_object,_ = p.getBasePositionAndOrientation(self.objectUid)
                vel_object,_ = p.getBaseVelocity(self.objectUid)
                pos_gripper, quat_gripper = p.getBasePositionAndOrientation(self.gripperUid)
                eul_gripper = p.getEulerFromQuaternion(quat_gripper)
                vel_gripper, vel_ang_gripper = p.getBaseVelocity(self.gripperUid)

                # Add a new data point
                new_state = [pos_object[0], pos_object[2], vel_object[0], vel_object[2],
                            pos_gripper[0], pos_gripper[2], eul_gripper[1], 
                            vel_gripper[0], vel_gripper[2], vel_ang_gripper[1]
                            ]
                via_points.append(new_state)

            if self.gui:
                time.sleep(20/240)

        return via_points
    
def toBulletStateInput(x, u, y_range):
    q = [x[0], y_range-x[1],
         x[2], -x[3],
         x[4], y_range-x[5], -x[6],
         x[7], -x[8], -x[9]]
    mu = [u[0], u[1], -u[2], -u[3]]
    return q, mu

def toOpenglStateInput(q, y_range):
    x = [q[0], y_range-q[1],
         q[2], -q[3],
         q[4], y_range-q[5], -q[6],
         q[7], -q[8], -q[9]]
    return x

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
