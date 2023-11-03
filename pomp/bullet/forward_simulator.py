import pybullet as p
import pybullet_data
import time
import math

class forward_simulation():
    def __init__(self):
        self.visualShapeId = -1
        physicsClient = p.connect(p.GUI) # p.GUI
        # physicsClient = p.connect(p.DIRECT) # p.GUI
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        self.set_params()
        self.create_shapes()
    
    def set_params(self):
        # Kinodynamics
        self.mass_object = 0.01
        self.pos_object = [0,0,1]
        self.quat_object = p.getQuaternionFromEuler([math.pi/2,0,0])
        self.vel_object = [.1,0,0]
        
        self.mass_gripper = 1
        self.moment_gripper = 1 # moment of inertia
        self.pos_gripper = [0,0,0]
        self.quat_gripper = p.getQuaternionFromEuler([0,0,0])
        self.vel_gripper = [0,0,0]
        self.vel_ang_gripper = [0,0,0]

        # Geometrics
        self.half_extents_gripper = [1, .05, .2]
        self.radius_object = 0.1
        self.radius_height = 0.1

    def create_shapes(self):
        # Create an object
        objectId = p.createCollisionShape(p.GEOM_CYLINDER, radius=.1, height=.1)
        self.objectUid = p.createMultiBody(self.mass_object, 
                                           objectId, 
                                           self.visualShapeId, 
                                           self.pos_object,
                                           self.quat_object)
        p.changeDynamics(self.objectUid, -1, lateralFriction=0, spinningFriction=0, 
                         rollingFriction=0, linearDamping=0, angularDamping=0)
        
        # Create a bar-gripper
        gripperId = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.half_extents_gripper)
        self.gripperUid = p.createMultiBody(self.mass_gripper, 
                                       gripperId, 
                                       self.visualShapeId, 
                                       self.pos_gripper,
                                       self.quat_gripper)
        p.changeDynamics(self.gripperUid, -1, lateralFriction=0, spinningFriction=0, 
                         rollingFriction=0, linearDamping=0, angularDamping=0)

    def reset_states(self, states):
        xo, zo, vxo, vzo, xg, zg, thetag, vxg, vzg, omegag = states # 10 DoF

        # Object kinematics
        self.pos_object = [xo,0.0,zo]
        self.vel_object = [vxo,0.0,vzo]

        # Gripper kinematics
        self.pos_gripper = [xg,0.0,zg]
        self.quat_gripper = p.getQuaternionFromEuler([0,thetag,0])
        self.vel_gripper = [vxg,0.0,vzg]
        self.vel_ang_gripper = [0.0,omegag,0.0]

        # Reset the kinematics
        p.resetBasePositionAndOrientation(self.objectUid, self.pos_object, self.quat_object)
        p.resetBasePositionAndOrientation(self.gripperUid, self.pos_gripper, self.quat_gripper)
        p.resetBaseVelocity(self.objectUid, self.vel_object)
        p.resetBaseVelocity(self.gripperUid, self.vel_gripper, self.vel_ang_gripper) # linear and angular vels both in world coordinates

    def run_forward_sim(self, inputs):
        t, ax, az, alpha = inputs

        # Reset execution duration
        # p.setTimeStep(t)

        # Step the simulation
        for _ in range(int(t*240)):
            # Apply external force
            p.applyExternalForce(self.gripperUid, -1, 
                                [self.mass_gripper*ax, 0, self.mass_gripper*az], 
                                [self.moment_gripper*alpha, 0, 0], 
                                p.LINK_FRAME)
            p.stepSimulation()
            time.sleep(1/240)

        # Get the object and gripper states
        self.pos_object,_ = p.getBasePositionAndOrientation(self.objectUid)
        self.vel_object,_ = p.getBaseVelocity(self.objectUid)
        self.pos_gripper,self.quat_gripper = p.getBasePositionAndOrientation(self.gripperUid)
        self.eul_gripper = p.getEulerFromQuaternion(self.quat_gripper)
        self.vel_gripper,self.vel_ang_gripper = p.getBaseVelocity(self.gripperUid)

        # Print the positions and velocities
        # print(f"Gripper Pos: {gripper_pos}, Gripper Vel: {gripper_vel}, Gripper Ang Vel: {gripper_ang_vel}, Object Pos: {object_pos}, Object Vel: {object_vel}")
        new_states = (self.pos_object[0], self.pos_object[2], self.vel_object[0], self.vel_object[2],
                      self.pos_gripper[0], self.pos_gripper[2], self.eul_gripper[1], 
                      self.vel_gripper[0], self.vel_gripper[2], self.vel_ang_gripper[1]
                      )
        return new_states
    
    def finish_sim(self):
        # Clean up and close the simulation
        p.disconnect()


sim = forward_simulation()
states = (1,1,0,0,
          1,0,0,0,0,0)
inputs = (2,0,10,.001)
sim.reset_states(states)
new_states = sim.run_forward_sim(inputs)
print(new_states)
sim.finish_sim()
