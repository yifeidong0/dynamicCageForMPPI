import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# Physics simulation setup
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

g = 9.81
m_box = 30
boxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[.3,]*3)
boxId = p.createMultiBody(m_box, boxId, -1, [0, 0, 1.0])

# gripperId = p.loadURDF(fileName='asset/lc_soft_enable_wide_grip/lc_soft_enable_wide_grip.urdf', 
# gripperId = p.loadURDF(fileName='asset/lc_soft_enable_wide_grip/lc_soft_enable_wide_grip_150_20.urdf', 
gripperId = p.loadURDF(fileName='asset/lc_soft_enable_wide_grip/lc_soft_enable_wide_grip_60_60.urdf', 
                        basePosition=[0,0,0], 
                        baseOrientation=p.getQuaternionFromEuler([math.pi/2,0,0]),
                        globalScaling=10,
                        )
p.changeDynamics(gripperId, -1, mass=0)

# Define the initial positions for the joints and the stiffness (P) and damping (D) coefficients
# num_joints = p.getNumJoints(gripperId)
jointIds = [1,2,3,5,6,7,9,10,11]
initial_positions = [math.pi/8,] * len(jointIds) # Adjust with your desired initial positions
stiffness = [1e-2,] * len(jointIds)  # P gain for each joint
damping = [1e-1,] * len(jointIds)  # D gain for each joint
# for i, jointId in enumerate(jointIds):
#     p.resetJointState(gripperId, jointId, initial_positions[i])

# Simulation loop
# time.sleep(6)
for i in range(80000):
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
    #                             forces=[10,]*len(jointIds),)
                                # forces=target_torques)
    # p.setJointMotorControlArray(bodyUniqueId=gripperId,
    #                             jointIndices=jointIds,
    #                             controlMode=p.POSITION_CONTROL,
    #                             targetPositions=initial_positions,
    #                             forces=target_torques)
    
    # Apply box gravity force
    pos_box, _ = p.getBasePositionAndOrientation(boxId)
    # p.applyExternalForce(objectUniqueId=boxId, linkIndex=-1, forceObj=[0,0,-m_box*g], posObj=pos_box, flags=p.WORLD_FRAME)

    # apply upward force on the box
    if i > 600:
        p.applyExternalForce(objectUniqueId=boxId, linkIndex=-1, forceObj=[-.3*m_box*g,0,1.18*m_box*g], posObj=pos_box, flags=p.WORLD_FRAME)
        p.applyExternalTorque(objectUniqueId=boxId, linkIndex=-1, torqueObj=[-.3,.1,.18], flags=p.WORLD_FRAME)

    # # Test reset functionalities
    # if i % 10 == 0:
    #     pos, quat_object = p.getBasePositionAndOrientation(boxId)
    #     eul_object = p.getEulerFromQuaternion(quat_object) # rad
    #     vel_object, vel_ang_object = p.getBaseVelocity(boxId)
    #     joint_states = p.getJointStates(gripperId, jointIds)
    #     pos_gripper = [state[0] for state in joint_states]
    #     vel_gripper = [state[1] for state in joint_states]

    #     p.resetBasePositionAndOrientation(boxId, pos, quat_object)
    #     p.resetBaseVelocity(boxId, vel_object, vel_ang_object)
    #     for i, jid in enumerate(jointIds):
    #         p.resetJointState(gripperId, jid, targetValue=pos_gripper[i], targetVelocity=vel_gripper[i])

    # Step simulation
    p.stepSimulation()
    time.sleep(1./240.)

# Disconnect from PyBullet
p.disconnect()