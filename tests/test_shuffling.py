
import pybullet as p
import pybullet_data
import time
import numpy as np
import math

########################################################### shuffling

# # Physics simulation setup
# p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# # p.setGravity(0, 0, -10)

# linkID = p.loadURDF("asset/linkage.urdf", basePosition=[0, 0, 4.55], baseOrientation=p.getQuaternionFromEuler([0,np.pi/2,0]))
# numJointsLink = p.getNumJoints(linkID)
# for i in range(numJointsLink+1):
#     p.changeDynamics(linkID, i, lateralFriction=1, spinningFriction=0, 
#                     rollingFriction=0, linearDamping=0, angularDamping=0)
# stiffness = [5e-3,]*4 # [8e-3,5e-3,5e-3,1e-2]  # P gain for each joint
# damping = [1e-1,] * numJointsLink  # D gain for each joint

# m = 1
# upperID = p.createCollisionShape(p.GEOM_BOX, halfExtents=[3,2,.2])
# upperID = p.createMultiBody(100*m, upperID, -1, [0,0,5.4])
# p.changeDynamics(upperID, -1, lateralFriction=1, spinningFriction=0, 
#                     rollingFriction=0, linearDamping=0, angularDamping=0)
# pivotInBox = [0, 0, 0] 
# pivotInWorld = [0, 0, 3]
# constraint1 = p.createConstraint(upperID, -1, -1, -1, p.JOINT_PRISMATIC, [0, 0, 1], pivotInBox, pivotInWorld)

# lowerID = p.loadURDF("plane.urdf")
# p.changeDynamics(lowerID, -1, lateralFriction=1, spinningFriction=0, 
#                     rollingFriction=0, linearDamping=0, angularDamping=0)

# # Retrieve the number of joints and print their info
# num_joints = p.getNumJoints(linkID)
# for i in range(num_joints):
#     print(p.getJointInfo(linkID, i))

# # Simulation loop
# # time.sleep(1)
# for i in range(8000):
#     linkBasePosition, _ = p.getBasePositionAndOrientation(linkID)
#     starti = 200
#     if i>starti and i < starti+1*240:
#         p.applyExternalForce(linkID, 1,
#                             [0,-2,0], 
#                             [0,0,0], 
#                             p.LINK_FRAME)

#     # Get the current orientation of the upperID
#     upperPosition, current_quat = p.getBasePositionAndOrientation(upperID)

#     # Pressing the upperID down
#     if i < 10*240:
#         joint_states = p.getJointStates(linkID, list(range(numJointsLink)))
#         joint_angles = [state[0] for state in joint_states]
#         force = [0,0,-1.45/(sum(joint_angles)+0.05)] # simple feedback control
#         p.applyExternalForce(upperID, -1,
#                         force, 
#                         upperPosition, 
#                         p.WORLD_FRAME)
    
#     # Mimic elastic cards stack behavior
#     p.setJointMotorControlArray(bodyUniqueId=linkID,
#                                 jointIndices=list(range(numJointsLink)),
#                                 controlMode=p.POSITION_CONTROL,
#                                 targetPositions=[0,]*numJointsLink,
#                                 targetVelocities=[0,]*numJointsLink,
#                                 positionGains=stiffness,
#                                 velocityGains=damping,
#                                 forces=[1,]*numJointsLink,)

#     # Step simulation
#     p.stepSimulation()
#     time.sleep(1./240.)

# # Disconnect from PyBullet
# p.disconnect()


########################################################### shuffling 2

# Physics simulation setup
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0, 0, -10)

linkID = p.loadURDF("asset/linkage.urdf", basePosition=[0, 0, 4.55], baseOrientation=p.getQuaternionFromEuler([0, np.pi/2, 0]))
numJointsLink = p.getNumJoints(linkID)
for i in range(numJointsLink+1):
    p.changeDynamics(linkID, i, lateralFriction=1, spinningFriction=0, 
                    rollingFriction=0, linearDamping=0, angularDamping=0)

m = 1
upperID = p.createCollisionShape(p.GEOM_BOX, halfExtents=[3,2,.2])
upperID = p.createMultiBody(100*m, upperID, -1, [0,0,5.4])
p.changeDynamics(upperID, -1, lateralFriction=1, spinningFriction=0, 
                    rollingFriction=0, linearDamping=0, angularDamping=0)
pivotInBox = [0, 0, 0] 
pivotInWorld = [0, 0, 3]
constraint1 = p.createConstraint(upperID, -1, -1, -1, p.JOINT_PRISMATIC, [0, 0, 1], pivotInBox, pivotInWorld)

lowerID = p.loadURDF("plane.urdf")
p.changeDynamics(lowerID, -1, lateralFriction=1, spinningFriction=0, 
                    rollingFriction=0, linearDamping=0, angularDamping=0)

stiffness = [1,]*numJointsLink # [8e-3,5e-3,5e-3,1e-2]  # P gain for each joint
damping = [1e-1,]*numJointsLink  # D gain for each joint
targetPositions = [0,0,0,0] # [0]*numJointsLink,
targetVelocities = [0,0,0,0] # [0.0,]*numJointsLink,

# Retrieve the number of joints and print their info
num_joints = p.getNumJoints(linkID)
# for i in range(num_joints):
#     print(p.getJointInfo(linkID, i))

# Simulation loop
# time.sleep(1)
for i in range(8000):
    # Push the card stack from the side
    linkBasePosition, _ = p.getBasePositionAndOrientation(linkID)
    starti = 200
    if i>starti and i < starti+1*240:
        p.applyExternalForce(linkID, 1,
                            [0,-1,0], 
                            [0,0,0], 
                            p.LINK_FRAME)

    # Squeezing the card stack
    upperPosition, current_quat = p.getBasePositionAndOrientation(upperID)
    joint_states = p.getJointStates(linkID, list(range(numJointsLink)))
    joint_angles = [state[0] for state in joint_states]
    if i < 10*240:
        force = [0,0,-4/(sum(joint_angles)+0.2)] # simple feedback control
        p.applyExternalForce(upperID, -1,
                        force, 
                        upperPosition, 
                        p.WORLD_FRAME)
    
    # Calculate the torques induced by joint elasticity
    joint_vels = [state[1] for state in joint_states]
    error_angles = [desired - current for desired, current in zip(targetPositions, joint_angles)]
    error_vels = [desired - current for desired, current in zip(targetVelocities, joint_vels)]
    target_torques = [kp * e - kd * av for kp, e, kd, av in zip(stiffness, error_angles, damping, error_vels)]
    # target_torques[2] = target_torques[2] + 1
    print("joint_angles: ", joint_angles)
    print("target_torques: ", target_torques)

    # Mimic elastic cards stack behavior
    p.setJointMotorControlArray(bodyUniqueId=linkID,
                                jointIndices=list(range(numJointsLink)),
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocities=[100,]*numJointsLink,
                                forces=target_torques,)
    joint_states = p.getJointStates(linkID, list(range(numJointsLink)))
    joint_tqs = [state[3] for state in joint_states]
    print("actual_torques: ", joint_tqs)
    print("==========")

    # Step simulation
    p.stepSimulation()
    time.sleep(1./240.)

# Disconnect from PyBullet
p.disconnect()
