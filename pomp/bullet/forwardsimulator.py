import pybullet as p
import pybullet_data
import time
import math

class forwardSimulation():
    def __init__(self, params, gui=False):
        self.visualShapeId = -1
        self.gui = gui
        if self.gui:
            p.connect(p.GUI) # p.GUI
        else:
            p.connect(p.DIRECT) # p.GUI
        self.g = -9.81
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # p.setGravity(0, 0, self.g)

        self.set_params(params)
        self.create_shapes()
    
    def set_params(self, params):
        # Kinodynamics
        self.mass_object = params[0]
        self.pos_object = [0,0,0]
        self.quat_object = p.getQuaternionFromEuler([math.pi/2,0,0])
        self.vel_object = [.1,0,0]
        
        self.mass_gripper = params[1]
        self.moment_gripper = params[2] # moment of inertia
        self.pos_gripper = [0,0,2]
        self.quat_gripper = p.getQuaternionFromEuler([0,0,0])
        self.vel_gripper = [0,0,0]
        self.vel_ang_gripper = [0,0,0]

        # Geometrics
        ydepth = 0.1
        self.half_extents_gripper = [params[3][0], ydepth/2, params[3][1]] # movement on x-z plane
        self.radius_object = params[4]
        self.height_object = ydepth

    def create_shapes(self):
        # Create an object
        objectId = p.createCollisionShape(p.GEOM_CYLINDER, radius=self.radius_object, height=self.height_object)
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


    def run_forward_sim_labeler(self, inputs, print_via_points=False):
        # print('run_forward_sim')
        t, ax, az = inputs
        print('input t', t, ax, az)

        # Step the simulation
        via_points = []
        num_via_points = 10
        for i in range(int(t*240)):
            # Apply external force on object
            p.applyExternalForce(self.objectUid, -1, 
                                # [self.mass_object*ax, self.mass_object*(az-0.0), 0], # gravity compensated 
                                [self.mass_object*ax, 0, self.mass_object*(az-0.0)], # gravity compensated 
                                self.pos_object, 
                                p.WORLD_FRAME)
            p.stepSimulation()
            self.pos_object,_ = p.getBasePositionAndOrientation(self.objectUid)

            # Print object via-points along the trajectory for visualization
            interval = int(int(t*240)/num_via_points)
            interval = 3 if interval==0 else interval
            if print_via_points and (i % interval == 0 or i == int(t*240)-1):
                # pos_object,_ = p.getBasePositionAndOrientation(self.objectUid)
                via_points.append([self.pos_object[0], self.pos_object[2]])

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
        print('via_points', via_points)
        print('self.pos_object', self.pos_object)
        
        return new_states, via_points

    def run_forward_sim_ball_balance(self, tc, print_via_points=False):
        via_points = []
        num_via_points = 10
        for i in range(int(tc*240)):
            p.stepSimulation()

            # Print object via-points along the trajectory for visualization
            interval = int(int(tc*240)/num_via_points)
            interval = 3 if interval==0 else interval
            if print_via_points and (i % interval == 0 or i == int(tc*240)-1):
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

        # new_states = [self.pos_object[0], self.pos_object[2], self.vel_object[0], self.vel_object[2],
        #               self.pos_gripper[0], self.pos_gripper[2], self.eul_gripper[1], 
        #               self.vel_gripper[0], self.vel_gripper[2], self.vel_ang_gripper[1]
        #               ]
        new_states = [self.pos_object[0], self.pos_object[2],
                      self.pos_gripper[0], self.pos_gripper[2], self.eul_gripper[1], 
                      ]
        return new_states, via_points
    
    def run_forward_sim(self, inputs, print_via_points=False):
        # print('run_forward_sim')
        t, ax, az, alpha = inputs

        # Step the simulation
        via_points = []
        num_via_points = 10
        for i in range(int(t*240)):
            # Apply external force on gripper
            p.applyExternalForce(self.gripperUid, -1, 
                                [self.mass_gripper*ax, 0, self.mass_gripper*az], 
                                [0, 0, 0], 
                                p.LINK_FRAME)
            p.applyExternalTorque(self.gripperUid, -1, 
                                 [0, self.moment_gripper*alpha, 0],
                                 p.LINK_FRAME)
            p.stepSimulation()

            # Print object via-points along the trajectory for visualization
            interval = int(int(t*240)/num_via_points)
            interval = 3 if interval==0 else interval
            if print_via_points and (i % interval == 0 or i == int(t*240)-1):
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
        return new_states, via_points
    
    def finish_sim(self):
        # Clean up and close the simulation
        p.disconnect()


# # Test
# mass_object = .1
# mass_gripper = 10
# moment_gripper = 1 # moment of inertia
# half_extents_gripper = [.7, .4] # movement on x-z plane
# radius_object = 0.01
# params = [mass_object, mass_gripper, moment_gripper, half_extents_gripper, radius_object]
# sim = forwardSimulation(params, gui=1)

# # From high to low
# data = [1.02, 5.11, 0.00, 0,
#             1.01, 4.70, -0.00, 0.00, 1, -0.50]
# gipper_vel = data[-3:]
# states = [[1.02, 5.11, 1.01, 4.7, -0.0], [4.304206592094783, 5.195005075715069, 1.0126748209851473, 4.754665910584282, -0.3832874049208745], [4.589857682473735, 1.4220963753814768, 1.0126748209851473, 5.1671659105842584, -0.58691567122751]]
# inputs = [[0.4341884048103374, 7.5790919267402685, -8.60582670121947], [0.41600639801144806, 0.6924874918276647, -9.146445334142033]]
# time.sleep(2.5)
# # states = [[1.02, 5.11, 0.0, 0, 1.01, 4.7, -0.0]]
# # inputs = [[0.8648671674748561, -1.5185060290787007, -1.7180355776782608]]

# # for i in range(len(inputs)):
# #     sim.reset_states(states[i]+gipper_vel)
# #     # sim.reset_states(states[i]+gipper_vel)
# #     new_states = sim.run_forward_sim_labeler(inputs[i])
# #     print('new_states', new_states)

# for i in range(len(inputs)):
#     state = states[i][:2]+inputs[i][1:]+states[i][2:]+gipper_vel
#     sim.reset_states(state)
#     # sim.reset_states(states[i]+gipper_vel)
#     new_states = sim.run_forward_sim_ball_balance(inputs[i][0])
#     print('new_states', new_states)

# sim.finish_sim()
