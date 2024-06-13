import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# Physics simulation setup
simple_scene = 1
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD) # enable Finite Element Method (FEM, mass-spring systems) rather than position based dynamics (PBD)
p.setGravity(0, 0, -10)
planeUid = p.loadURDF("plane.urdf", basePosition=[0,0,0])
g = 9.81
# m_box = .04

# for i in range(2):
#     boxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[.1,.1,.1,])
#     boxId = p.createMultiBody(m_box, boxId, -1, 0.5*np.random.rand(3) + np.array([0, 0, 0.3]))    

gripperId = p.loadURDF(fileName='asset/lc_soft_enable_wide_grip/lc_soft_enable_wide_grip.urdf', 
                        basePosition=[0,0,2.6], 
                        baseOrientation=p.getQuaternionFromEuler([1*math.pi,0,0]),
                        globalScaling=10,
                        )
# p.changeDynamics(gripperId, -1, mass=0)

tex = p.loadTexture("uvmap.png")
bread = p.loadSoftBody("asset/deformable/bread/bread.vtk", basePosition = [0,0.1,0.7], baseOrientation = p.getQuaternionFromEuler([1.57,0,0]),
                    #    scale = 0.5, mass = .01, useMassSpring = 1, springElasticStiffness = 0.2, useSelfCollision = 1, frictionCoeff = .5, collisionMargin = 0.001)
                       scale = 0.5, mass = .005, useNeoHookean = 1,  useSelfCollision = 1, frictionCoeff = .5, collisionMargin = 0.001) # clearly deformable
p.changeVisualShape(bread, -1, rgbaColor=[1,1,1,1], textureUniqueId=tex, flags=0)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(0)

if not simple_scene: 
    ############################# A #############################
    pivotInBox = [0, 0, 0] 
    pivotInWorld = [0, 0, 0]
    constraint1 = p.createConstraint(gripperId, -1, -1, -1, p.JOINT_PRISMATIC, [0, 0, 1], pivotInBox, pivotInWorld, p.getQuaternionFromEuler([1*math.pi,0,0]))

    # Define the initial positions for the joints and the stiffness (P) and damping (D) coefficients
    jointIds = [0,1,2,3]
    initial_positions = [0.5, 1.1, 0.5, 1.1] # Adjust with your desired initial positions
    stiffness = [1e1,] * len(jointIds)  # P gain for each joint
    damping = [1e9,] * len(jointIds)  # D gain for each joint

    # Simulation loop
    # time.sleep(6)
    for i in range(80000):

        # Apply the calculated torques to all joints at once
        p.setJointMotorControlArray(bodyUniqueId=gripperId,
                                    jointIndices=jointIds,
                                    # controlMode=p.VELOCITY_CONTROL,
                                    # targetVelocities=[1,]*len(jointIds),
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=initial_positions,
                                    targetVelocities=[0,]*len(jointIds),
                                    positionGains=stiffness,
                                    velocityGains=damping,
                                    # controlMode=p.TORQUE_CONTROL,
                                    forces=[1,]*len(jointIds),)

        # res = p.getClosestPoints(gripperId, boxId, 100, 1, -1) # 1,3,
        # dist = res[0][8] if (len(res)>0) else 0 # and res[0][8]>=0
        joint_states = p.getJointStates(gripperId, jointIds)
        joint_positions = [state[0] for state in joint_states]
        print('joint_positions: ', joint_positions)

        # apply upward force on the gripper
        pos_gripper, _ = p.getBasePositionAndOrientation(gripperId)
        if i < 120:
            p.applyExternalForce(objectUniqueId=gripperId, linkIndex=-1, forceObj=[0,0,-15], posObj=pos_gripper, flags=p.WORLD_FRAME)
        else:
            p.applyExternalForce(objectUniqueId=gripperId, linkIndex=-1, forceObj=[0,0,15], posObj=pos_gripper, flags=p.WORLD_FRAME)

        # Step simulation
        p.stepSimulation()
        time.sleep(3./240.)
else:
    ############################# B #############################
    p.changeDynamics(gripperId, -1, mass=0)
    # p.changeDynamics(bread, -1, mass=1)
    p.resetJointState(gripperId, 1, 1)
    p.resetJointState(gripperId, 3, 1)
    p.changeDynamics(gripperId, -1, jointDamping=0.0001)
    for i in range(80000):

        # # Apply the calculated torques to all joints at once
        # p.setJointMotorControlArray(bodyUniqueId=gripperId,
        #                             jointIndices=jointIds,
        #                             # controlMode=p.VELOCITY_CONTROL,
        #                             # targetVelocities=[1,]*len(jointIds),
        #                             controlMode=p.POSITION_CONTROL,
        #                             targetPositions=initial_positions,
        #                             targetVelocities=[0,]*len(jointIds),
        #                             positionGains=stiffness,
        #                             velocityGains=damping,
        #                             # controlMode=p.TORQUE_CONTROL,
        #                             forces=[1,]*len(jointIds),)

        # # res = p.getClosestPoints(gripperId, boxId, 100, 1, -1) # 1,3,
        # # dist = res[0][8] if (len(res)>0) else 0 # and res[0][8]>=0
        # joint_states = p.getJointStates(gripperId, jointIds)
        # joint_positions = [state[0] for state in joint_states]
        # print('joint_positions: ', joint_positions)

        # # apply upward force on the gripper
        # pos_gripper, _ = p.getBasePositionAndOrientation(gripperId)
        # if i < 120:
        #     p.applyExternalForce(objectUniqueId=gripperId, linkIndex=-1, forceObj=[0,0,-15], posObj=pos_gripper, flags=p.WORLD_FRAME)
        # else:
        #     p.applyExternalForce(objectUniqueId=gripperId, linkIndex=-1, forceObj=[0,0,15], posObj=pos_gripper, flags=p.WORLD_FRAME)

        # Step simulation
        p.stepSimulation()
        time.sleep(1./240.)

    # Disconnect from PyBullet
    p.disconnect()