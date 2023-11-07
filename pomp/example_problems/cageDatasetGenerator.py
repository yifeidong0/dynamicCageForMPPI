from ..bullet.forward_simulator import forward_simulation
import random
import pybullet as p
import pybullet_data
import time
import math

class data_generator_sim(forward_simulation):
    def __init__(self, params):
        super().__init__(params)

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
                pos_object,_ = p.getBasePositionAndOrientation(self.objectUid)
                via_points.append([pos_object[0], pos_object[2]])

            if self.gui:
                time.sleep(10/240)

        # Get the object and gripper states
        self.pos_object,_ = p.getBasePositionAndOrientation(self.objectUid)
        self.vel_object,_ = p.getBaseVelocity(self.objectUid)
        self.pos_gripper,self.quat_gripper = p.getBasePositionAndOrientation(self.gripperUid)
        self.eul_gripper = p.getEulerFromQuaternion(self.quat_gripper)
        self.vel_gripper,self.vel_ang_gripper = p.getBaseVelocity(self.gripperUid)

        new_states = [self.pos_object[0], self.pos_object[2], self.vel_object[0], self.vel_object[2],
                      self.pos_gripper[0], self.pos_gripper[2], self.eul_gripper[1], 
                      self.vel_gripper[0], self.vel_gripper[2], self.vel_ang_gripper[1]
                      ]
        return new_states
    
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

