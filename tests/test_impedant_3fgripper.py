import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# Physics simulation setup
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0, 0, -10)

g = 9.81
m_box = 1.0
boxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[.3,]*3)
boxId = p.createMultiBody(m_box, boxId, -1, [0, 0, 1.5])

gripperId = p.loadURDF(fileName='asset/robotiq_3f_gripper_visualization/cfg/robotiq-3f-gripper_articulated.urdf', 
# gripperId = p.loadURDF(fileName='asset/linkage.urdf', 
                        basePosition=[0,0,0], 
                        baseOrientation=p.getQuaternionFromEuler([math.pi/2,0,0]),
                        globalScaling=10,
                        )

# Define the initial positions for the joints and the stiffness (P) and damping (D) coefficients
# num_joints = p.getNumJoints(gripperId)
jointIds = [1,2,3,5,6,7,9,10,11]
initial_positions = [math.pi/6,] * len(jointIds) # Adjust with your desired initial positions
stiffness = [3e-1,] * len(jointIds)  # P gain for each joint
damping = [1,] * len(jointIds)  # D gain for each joint

p.changeDynamics(gripperId, -1, mass=0) 

# Simulation loop
time.sleep(1)
for i in range(8000):
    # Read the current state of the gripper
    joint_states = p.getJointStates(gripperId, jointIds)
    target_torques = []

    for k in range(len(jointIds)):
        current_position, current_velocity, _, T = joint_states[k]

        # Calculate the position deviation and the velocity
        position_deviation = initial_positions[k] - current_position
        velocity = -current_velocity

        # Calculate the control input (torque) based on stiffness and damping
        control_input = stiffness[k] * position_deviation + damping[k] * velocity
        target_torques.append(control_input)
        # print("getJointStates torques: ", T)
    #     print("current_position: ", current_position)
    # print("target_torques: ", target_torques)

    # Apply the calculated torques to all joints at once
    p.setJointMotorControlArray(bodyUniqueId=gripperId,
                                jointIndices=jointIds,
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocities=[1,]*len(jointIds),
                                forces=target_torques)
    
    # Apply box gravity force
    pos_box, _ = p.getBasePositionAndOrientation(boxId)
    p.applyExternalForce(objectUniqueId=boxId, linkIndex=-1, forceObj=[0,0,-m_box*g], posObj=pos_box, flags=p.WORLD_FRAME)

    # apply upward force on the box
    if i > 1000:
        p.applyExternalForce(objectUniqueId=boxId, linkIndex=-1, forceObj=[0,0,1.08*m_box*g], posObj=pos_box, flags=p.WORLD_FRAME)

    # Step simulation
    p.stepSimulation()
    time.sleep(1./240.)

# Disconnect from PyBullet
p.disconnect()