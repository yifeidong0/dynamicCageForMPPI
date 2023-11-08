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
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        self.set_params(params)
        self.create_shapes()
    
    def set_params(self, params):
        # Kinodynamics
        self.mass_object = params[0]
        self.pos_object = [0,0,1]
        self.quat_object = p.getQuaternionFromEuler([math.pi/2,0,0])
        self.vel_object = [.1,0,0]
        
        self.mass_gripper = params[1]
        self.moment_gripper = params[2] # moment of inertia
        self.pos_gripper = [0,0,0]
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

    def run_forward_sim(self, inputs, print_via_points=False):
        t, ax, az, alpha = inputs

        # Step the simulation
        via_points = []
        num_via_points = 10
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
# def toBulletStateInput(x, u, y_range=10):
#     q = [x[0],y_range-x[1],
#             x[2],-x[3],
#             x[4],y_range-x[5],-x[6],
#             x[7],-x[8],-x[9]]
#     mu = [u[0],u[1],-u[2],-u[3]]
#     return q, mu

# mass_object = 1
# mass_gripper = 10
# moment_gripper = 1 # moment of inertia
# half_extents_gripper = [.5, .1] # movement on x-z plane
# radius_object = 0.01
# params = [mass_object, mass_gripper, moment_gripper, 
#                 half_extents_gripper, radius_object]
# y_range = 10

# sim = forwardSimulation(params)

# # From high to low
# states = [[2, 4, 0, 0, 2, 4.109999999999999, 0, 0, 0, 0], [1.995292964091273, 3.7873304581176566, -0.07073937686370206, -1.3932966860397489, 1.7861555313614932, 3.919789446208374, 0.1222387605911052, -1.645760621037054, -0.8461392367269411, 2.065928698884493], [1.7247009152577664, 2.3742129228669606, 1.2746528189695867, -3.0425517881933244, 1.5578530308862355, 2.233373881443944, -1.1961127188185685, 2.5214314026839735, -3.262241107766775, -6.317331079693599], [3.179732650547473, 2.1414253501816454, 2.8215238073188087, 2.2604978605019603, 4.132588022546566, 2.508852531622207, 1.12363614851984, 4.796076891251007, 5.801466561062297, -4.694413137977301], [4.472931062235257, 4.2172446820783795, 2.8215238073188087, 6.756747860501954, 5.418581593848224, 5.248893880422402, 0.2126903021881291, 1.5345973763356378, 5.588080833655447, 0.6706038360387792], [5.970340226987923, 7.580523727119891, 2.196582709678756, 2.995547085711504, 6.037533971612834, 7.678680732039295, 0.12330537875166867, 0.8749980294504657, 3.1985447844520474, -0.4997994198852321], [8.276138235635273, 7.874828501455653, 5.984207367618207, -2.2760302493510784, 8.388550760182172, 8.148470644774143, -0.8234013649078289, 6.8724932137641765, -0.8275367866594386, -1.2167603816236015]]
# inputs = [[0.26041288977332316, -6.204326694977351, -17.95028134004808, -0.008637703247645373], [0.621012591023977, 6.167644611467075, -16.909586903518125, 0.040735829405351975], [0.5486304428090828, -3.1699196089356647, -12.10962471364506, -0.025317885652593616], [0.46232962928606147, -1.8319936310210636, -12.87665958888018, -0.07878409224865462], [0.5683599539302843, 0.9952630897403942, -15.721107994487806, 0.05158356465838723], [0.8073719433361644, 3.7559906446292928, -18.690164318047753, 0.0009171984901206881]]
# # time.sleep(3.5)

# for i in range(len(inputs)):
#     q, mu = toBulletStateInput(states[i],inputs[i],y_range)
#     sim.reset_states(q)

#     obj = sim.objectUid
#     grip = sim.gripperUid

#     new_states = sim.run_forward_sim(mu)
#     # print(new_states)
# sim.finish_sim()
