import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# Physics simulation setup
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeUid = p.loadURDF("plane.urdf", basePosition=[0,0,0])

g = 9.81
m_box = .2

# boxId = p.createCollisionShape(p.GEOM_CYLINDER, radius=.3, height=.5)
# boxId = p.createMultiBody(m_box,
#                             boxId,
#                             -1,
#                             [0, 0, 0.3],
#                             p.getQuaternionFromEuler([0.5*math.pi,0,0]))

for i in range(10):
    boxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[.1,.1,.1,])
    boxId = p.createMultiBody(m_box*0.2, boxId, -1, 0.3*np.random.rand(3) + np.array([0, 0, 0.3]))    

gripperId = p.loadURDF(fileName='asset/lc_soft_enable_wide_grip/lc_soft_enable_wide_grip.urdf', 
                        basePosition=[0,0,2.6], 
                        baseOrientation=p.getQuaternionFromEuler([1*math.pi,0,0]),
                        globalScaling=10,
                        )
# p.changeDynamics(gripperId, -1, mass=0)

pivotInBox = [0, 0, 0] 
pivotInWorld = [0, 0, 0]
constraint1 = p.createConstraint(gripperId, -1, -1, -1, p.JOINT_PRISMATIC, [0, 0, 1], pivotInBox, pivotInWorld, p.getQuaternionFromEuler([1*math.pi,0,0]))

# Define the initial positions for the joints and the stiffness (P) and damping (D) coefficients
# num_joints = p.getNumJoints(gripperId)
# jointIds = [0,2]
# jointIds = [1,3]
jointIds = [0,1,2,3]
initial_positions = [0.5,] * len(jointIds) # Adjust with your desired initial positions
stiffness = [1e1,] * len(jointIds)  # P gain for each joint
damping = [1e9,] * len(jointIds)  # D gain for each joint
# for i, jointId in enumerate(jointIds):
#     p.resetJointState(gripperId, jointId, initial_positions[i])

# Simulation loop
# time.sleep(6)
for i in range(80000):
    print("i: ", i)
    # Read the current state of the gripper
    # joint_states = p.getJointStates(gripperId, jointIds)
    # target_torques = []

    # for k in range(len(jointIds)):
    #     current_position, current_velocity, _, T = joint_states[k]

    #     # Calculate the position deviation and the velocity
    #     position_deviation = initial_positions[k] - current_position
    #     velocity = -current_velocity

    #     # Calculate the control input (torque) based on stiffness and damping
    #     control_input = stiffness[k] * position_deviation + damping[k] * velocity
    #     target_torques.append(control_input)
        # print("getJointStates torques: ", T)
    #     print("current_position: ", current_position)
    # print("target_torques: ", target_torques)

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
    
    # Reset gripper base position
    # p.resetBasePositionAndOrientation(gripperId, [0,0,2.-i/1000], p.getQuaternionFromEuler([1*math.pi,0,0]))

    # Apply box gravity force
    # pos_box, _ = p.getBasePositionAndOrientation(boxId)
    # p.applyExternalForce(objectUniqueId=boxId, linkIndex=-1, forceObj=[0,0,-m_box*g], posObj=pos_box, flags=p.WORLD_FRAME)

    # Get gripper joint positions
    # Update initial_positions
    if i == 300:
        joint_states = p.getJointStates(gripperId, jointIds)
        joint_positions = [state[0] for state in joint_states]
        initial_positions = joint_positions
    # print("joint_states: ", joint_positions)

    # apply upward force on the gripper
    if i < 300:
        p.applyExternalForce(objectUniqueId=gripperId, linkIndex=-1, forceObj=[0,0,-20], posObj=[0,0,0], flags=p.WORLD_FRAME)
    else:
        p.applyExternalForce(objectUniqueId=gripperId, linkIndex=-1, forceObj=[0,0,20], posObj=[0,0,0], flags=p.WORLD_FRAME)

    # Step simulation
    p.stepSimulation()
    time.sleep(4./240.)

# Disconnect from PyBullet
p.disconnect()