import pybullet as p
import pybullet_data
import time
import math

class forwardSimulationEL():
    def __init__(self, gui=False):
        self.visualShapeId = -1
        self.gui = gui
        if self.gui:
            p.connect(p.GUI) # p.GUI
        else:
            p.connect(p.DIRECT) # p.GUI
        self.g = -9.81
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # p.setGravity(0, 0, self.g)

        # self.set_params(params)
        # self.create_shapes()
    
    def set_params(self, params):
        # Kinodynamics
        self.mass_object = params[0]
        self.pos_object = [1000,0,0]
        self.quat_object = p.getQuaternionFromEuler([math.pi/2,0,0])
        self.vel_object = [.1,0,0]
        
        self.mass_gripper = params[1]
        self.moment_gripper = params[2] # moment of inertia
        self.pos_gripper = [1000,0,2]
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

    def run_forward_sim_labeler(self, inputs, print_via_points=True, num_via_points=10):
        t, ax, az = inputs
        interval = int(int(t*240)/num_via_points)
        interval = 3 if interval==0 else interval
        
        # Step the simulation
        via_points = []
        force_on_object = [self.mass_object*ax, 0, self.mass_object*(az-0.0)]
        for i in range(int(t*240)):
            # Apply external force on object
            p.applyExternalForce(self.objectUid, -1, 
                                # [self.mass_object*ax, self.mass_object*(az-0.0), 0], # gravity compensated 
                                force_on_object, # gravity compensated 
                                self.pos_object, 
                                p.WORLD_FRAME)
            p.stepSimulation()
            self.pos_object,_ = p.getBasePositionAndOrientation(self.objectUid)

            # Print object via-points along the trajectory for visualization
            if print_via_points and (i % interval == 0 or i == int(t*240)-1):
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

        return new_states, via_points

    def finish_sim(self):
        # Clean up and close the simulation
        p.disconnect()


class forwardSimulationPlanePush():
    def __init__(self, gui=False):
        self.visualShapeId = -1
        self.gui = gui
        if self.gui:
            p.connect(p.GUI) # p.GUI
        else:
            p.connect(p.DIRECT) # p.GUI
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) # optionally
        self.g = -9.81
        p.setGravity(0, 0, self.g)

        # self.set_params(params)
        # self.create_shapes()
    
    def set_params(self, params):
        # Kinodynamics
        self.mass_object = params[0]
        self.moment_object = params[1] # moment of inertia
        self.pos_object = [1000,0,0]
        self.quat_object = p.getQuaternionFromEuler([math.pi/2,0,0])
        self.vel_object = [.1,0,0]
        
        self.mass_gripper = params[2]
        self.moment_gripper = params[3]
        self.pos_gripper = [1000,0,2]
        self.quat_gripper = p.getQuaternionFromEuler([0,0,0])
        self.vel_gripper = [0,0,0]
        self.vel_ang_gripper = [0,0,0]

        self.lateralFriction = 0.0

        # Geometrics
        self.z_bodies = .1

    def create_shapes(self):
        # Create a plane
        self.planeId = p.loadURDF("plane.urdf", basePosition=[0,0,-self.z_bodies])
        p.changeDynamics(self.planeId, -1, lateralFriction=self.lateralFriction, spinningFriction=0, 
                         rollingFriction=0, linearDamping=0, angularDamping=0)
        
        # Create an object # TODO import complex shapes from files
        objectId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[.2, .6, self.z_bodies])
        self.objectUid = p.createMultiBody(self.mass_object, 
                                           objectId, 
                                           self.visualShapeId, 
                                           self.pos_object,
                                           self.quat_object)
        p.changeDynamics(self.objectUid, -1, lateralFriction=self.lateralFriction, spinningFriction=0, 
                         rollingFriction=0, linearDamping=0, angularDamping=0)
        
        # Create a robot
        gripperId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[.5, .5, self.z_bodies])
        self.gripperUid = p.createMultiBody(self.mass_gripper, 
                                       gripperId, 
                                       self.visualShapeId, 
                                       self.pos_gripper,
                                       self.quat_gripper)
        p.changeDynamics(self.gripperUid, -1, lateralFriction=self.lateralFriction, spinningFriction=0, 
                         rollingFriction=0, linearDamping=0, angularDamping=0)

    def reset_states(self, states):
        xo, yo, thetao, vxo, vyo, omegao, xg, yg, thetag, vxg, vyg, omegag = states # 12 DoF

        # Object kinematics
        self.pos_object = [xo, yo, 0.0]
        self.quat_object = p.getQuaternionFromEuler([0.0, 0.0, thetao])
        self.vel_object = [vxo, vyo, 0.0]
        self.vel_ang_object = [0.0, 0.0, omegao]

        # Gripper kinematics
        self.pos_gripper = [xg, yg, 0.0]
        self.quat_gripper = p.getQuaternionFromEuler([0.0, 0.0, thetag])
        self.vel_gripper = [vxg, vyg, 0.0]
        self.vel_ang_gripper = [0.0, 0.0, omegag]

        # Reset the kinematics
        p.resetBasePositionAndOrientation(self.objectUid, self.pos_object, self.quat_object)
        p.resetBasePositionAndOrientation(self.gripperUid, self.pos_gripper, self.quat_gripper)
        p.resetBaseVelocity(self.objectUid, self.vel_object, self.vel_ang_object)
        p.resetBaseVelocity(self.gripperUid, self.vel_gripper, self.vel_ang_gripper) # linear and angular vels both in world coordinates

    def run_forward_sim_labeler(self, inputs, print_via_points=True, num_via_points=10):
        t, ax, ay, omega = inputs
        interval = int(int(t*240)/num_via_points)
        interval = 3 if interval==0 else interval

        # Step the simulation
        via_points = []
        force_on_object = [self.mass_object*ax, self.mass_object*ay, 0.0]
        torque_on_object = [0.0, 0.0, self.moment_object*omega]
        for i in range(int(t*240)):
            # Apply external force on object
            p.applyExternalForce(self.objectUid, -1, 
                                force_on_object, # gravity compensated 
                                self.pos_object, 
                                p.WORLD_FRAME)
            p.applyExternalTorque(self.objectUid, -1, 
                                torque_on_object,
                                p.WORLD_FRAME)
            p.stepSimulation()
            self.pos_object,_ = p.getBasePositionAndOrientation(self.objectUid)

            # Print object via-points along the trajectory for visualization

            if print_via_points and (i % interval == 0 or i == int(t*240)-1):
                via_points.append([self.pos_object[0], self.pos_object[1]])

            if self.gui:
                time.sleep(10/240)

        # Get the object and gripper states
        self.pos_object, self.quat_object = p.getBasePositionAndOrientation(self.objectUid)
        self.eul_object = p.getEulerFromQuaternion(self.quat_object) # rad
        self.vel_object, self.vel_ang_object = p.getBaseVelocity(self.objectUid)
        self.pos_gripper, self.quat_gripper = p.getBasePositionAndOrientation(self.gripperUid)
        self.eul_gripper = p.getEulerFromQuaternion(self.quat_gripper)
        self.vel_gripper,self.vel_ang_gripper = p.getBaseVelocity(self.gripperUid)

        new_states = [self.pos_object[0], self.pos_object[1], self.eul_object[2],
                      self.vel_object[0], self.vel_object[1], self.vel_ang_object[2],
                      self.pos_gripper[0], self.pos_gripper[1], self.eul_gripper[2], 
                      self.vel_gripper[0], self.vel_gripper[1], self.vel_ang_gripper[2]
                      ]

        return new_states, via_points

    def finish_sim(self):
        # Clean up and close the simulation
        p.disconnect()


class forwardSimulationWaterSwing():
    def __init__(self, gui=False):
        self.visualShapeId = -1
        self.gui = gui
        if self.gui:
            p.connect(p.GUI) # p.GUI
        else:
            p.connect(p.DIRECT) # p.GUI
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) # optionally
        self.g = -9.81
        # p.setGravity(0, 0, self.g)

        # self.set_params(params)
        # self.create_shapes()
    
    def set_params(self, params):
        # Kinodynamics
        self.mass_object = params[0]
        self.moment_object = params[1] # moment of inertia
        self.pos_object = [1000,0,0]
        self.quat_object = p.getQuaternionFromEuler([0,0,0])
        self.vel_object = [.1,0,0]
        
        self.mass_gripper = params[2]
        self.moment_gripper = params[3]
        self.pos_gripper = [1000,0,2]
        self.quat_gripper = p.getQuaternionFromEuler([0,0,0])
        self.vel_gripper = [0,0,0]
        self.vel_ang_gripper = [0,0,0]

        self.lateralFriction = 0.5

        # Geometrics
        self.y_bodies = .3

    def create_shapes(self):
        # Create a plane
        # self.planeId = p.loadURDF("plane.urdf", basePosition=[0,0,-self.z_bodies])
        # p.changeDynamics(self.planeId, -1, lateralFriction=self.lateralFriction, spinningFriction=0, 
        #                  rollingFriction=0, linearDamping=0, angularDamping=0)
        
        # Create an object # TODO import complex shapes from files
        objectId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[.3, self.y_bodies, .3])
        self.objectUid = p.createMultiBody(self.mass_object, 
                                           objectId, 
                                           self.visualShapeId, 
                                           self.pos_object,
                                           self.quat_object)
        p.changeDynamics(self.objectUid, -1, lateralFriction=self.lateralFriction, spinningFriction=0, 
                         rollingFriction=0, linearDamping=0, angularDamping=0)
        
        # Create a robot
        # gripperId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[.5, self.y_bodies, .5])
        # self.gripperUid = p.createMultiBody(self.mass_gripper, 
        #                                gripperId, 
        #                                self.visualShapeId, 
        #                                self.pos_gripper,
        #                                self.quat_gripper)
        self.gripperUid = p.loadURDF("asset/4face-bottle.urdf", self.pos_gripper, self.quat_gripper)
        p.changeDynamics(self.gripperUid, -1, lateralFriction=self.lateralFriction, spinningFriction=0, 
                         rollingFriction=0, linearDamping=0, angularDamping=0)

    def reset_states(self, states):
        xo, zo, thetao, vxo, vzo, omegao, xg, zg, thetag, vxg, vzg, omegag = states # 12 DoF

        # Object kinematics
        self.pos_object = [xo, 0.0, zo]
        self.quat_object = p.getQuaternionFromEuler([0.0, thetao, 0.0])
        self.vel_object = [vxo, 0.0, vzo]
        self.vel_ang_object = [0.0, omegao, 0.0]

        # Gripper kinematics
        self.pos_gripper = [xg, 0.0, zg]
        self.quat_gripper = p.getQuaternionFromEuler([0.0, thetag, 0.0])
        self.vel_gripper = [vxg, 0.0, vzg]
        self.vel_ang_gripper = [0.0, omegag, 0.0]

        # Reset the kinematics
        p.resetBasePositionAndOrientation(self.objectUid, self.pos_object, self.quat_object)
        p.resetBasePositionAndOrientation(self.gripperUid, self.pos_gripper, self.quat_gripper)
        p.resetBaseVelocity(self.objectUid, self.vel_object, self.vel_ang_object)
        p.resetBaseVelocity(self.gripperUid, self.vel_gripper, self.vel_ang_gripper) # linear and angular vels both in world coordinates

    def run_forward_sim(self, inputs, print_via_points=True, num_via_points=10):
        t, ax, az, omega = inputs
        interval = int(int(t*240)/num_via_points)
        interval = 3 if interval==0 else interval

        # Step the simulation
        via_points = []
        force_on_object = [self.mass_object*ax, 0.0, self.mass_object*az]
        torque_on_object = [0.0, self.moment_object*omega, 0.0]
        for i in range(int(t*240)):
            # Apply external force on object
            p.applyExternalForce(self.objectUid, -1, 
                                force_on_object, # gravity compensated 
                                self.pos_object, 
                                p.WORLD_FRAME)
            p.applyExternalTorque(self.objectUid, -1, 
                                torque_on_object,
                                p.WORLD_FRAME)
            p.stepSimulation()
            self.pos_object,_ = p.getBasePositionAndOrientation(self.objectUid)

            # Print object via-points along the trajectory for visualization
            if print_via_points and (i % interval == 0 or i == int(t*240)-1):
                via_points.append([self.pos_object[0], self.pos_object[2]])

            if self.gui:
                time.sleep(1/240)

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
# time.sleep(2.5)

# # energy labeler
# states = [
#     [1.02, 5.11, 0.0, 1, 1.01, 4.7, -0.0], 
#     [1.02, 5.11, 0.0, 1, 1.01, 4.7, -0.0], 
#           [0.7100127117984576, 5.387613813939294, -0.4604435562589945, -0.5801269016460175, 1.0099999973140912, 5.648274952381763, 2.2790271618888782e-05], 
#           [0.0471691448743874, 4.42801122570194, -1.2927751208622842, -1.9570681321835652, 1.0099999973140912, 6.402441619048386, 0.00044360889042296335], 
#           [-1.0554520549137338, 3.3182704622122805, -1.264165870195518, -0.6226900540635716, 1.0099999973140912, 7.264941619048337, 0.0004436088904229634]
#           ]
# inputs = [
#     [0.9519515951370671, -0.28284521572525545, -1.8662179375099317], 
#     [0.9519515951370671, -0.28284521572525545, -1.8662179375099317], 
#           [0.7549019843490768, -1.10364406356239, -1.8257784272303628], 
#           [0.8662928020120086, 0.0331701457006246, 1.5471050181101496]
#           ]
# sim.reset_states(states[0]+gipper_vel)
# for i in range(len(inputs)):
#     # sim.reset_states(states[i]+gipperc_vel)
#     new_states = sim.run_forward_sim_labeler(inputs[i])
#     print('new_states', new_states)

# # # ball balance
# # states = [[1.02, 5.11, 1.01, 4.7, -0.0], [4.304206592094783, 5.195005075715069, 1.0126748209851473, 4.754665910584282, -0.3832874049208745], [4.589857682473735, 1.4220963753814768, 1.0126748209851473, 5.1671659105842584, -0.58691567122751]]
# # inputs = [[0.4341884048103374, 7.5790919267402685, -8.60582670121947], [0.41600639801144806, 0.6924874918276647, -9.146445334142033]]
# # for i in range(len(inputs)):
# #     state = states[i][:2]+inputs[i][1:]+states[i][2:]+gipper_vel
# #     sim.reset_states(state)
# #     new_states = sim.run_forward_sim_ball_balance(inputs[i][0])
# #     print('new_states', new_states)

# sim.finish_sim()
