

########################################################### shuffling
import pybullet as p
import pybullet_data
import time
import numpy as np
import math

# Physics simulation setup
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0, 0, -10)

# Create two cubes
linkID = p.loadURDF("asset/linkage.urdf", basePosition=[0, 0, 4.55], baseOrientation=p.getQuaternionFromEuler([0,np.pi/2,0]))
numJointsLink = p.getNumJoints(linkID)
for i in range(numJointsLink+1):
    p.changeDynamics(linkID, i, lateralFriction=1, spinningFriction=0, 
                    rollingFriction=0, linearDamping=0, angularDamping=0)
stiffness = [5e-3,]*4 # [8e-3,5e-3,5e-3,1e-2]  # P gain for each joint
damping = [1e-1,] * numJointsLink  # D gain for each joint

# fishID = p.loadURDF("asset/fine-fish-10/fine-fish-10.urdf", basePosition=[0, -3, 0.5])

m = 1
upperID = p.createCollisionShape(p.GEOM_BOX, halfExtents=[3,2,.2])
upperID = p.createMultiBody(100*m, upperID, -1, [0,0,5.4])
p.changeDynamics(upperID, -1, lateralFriction=1, spinningFriction=0, 
                    rollingFriction=0, linearDamping=0, angularDamping=0)

lowerID = p.loadURDF("plane.urdf")
p.changeDynamics(lowerID, -1, lateralFriction=1, spinningFriction=0, 
                    rollingFriction=0, linearDamping=0, angularDamping=0)

# fingerID = p.createCollisionShape(p.GEOM_BOX, halfExtents=[.2,]*3)
# fingerID = p.createMultiBody(m, fingerID, -1, [0,.6,3])

# Joint parameters
target_velocity = 1  # rad/s, change to the desired constant velocity

# Retrieve the number of joints and print their info
num_joints = p.getNumJoints(linkID)
for i in range(num_joints):
    print(p.getJointInfo(linkID, i))

# Desired upright orientation (euler angles) for the upperID
desired_orientation = [0, 0, 0]  # Assuming you want it to be upright along the z-axis
kp = 10  # Proportional gain
kd = 1   # Derivative gain

# Simulation loop
time.sleep(1)
for i in range(8000):
    # fingerPosition, _ = p.getBasePositionAndOrientation(fingerID)
    linkBasePosition, _ = p.getBasePositionAndOrientation(linkID)
    starti = 600
    if i>starti and i < starti+2*240:
        p.applyExternalForce(linkID, 1,
                            [0,-2,0], 
                            [0,0,0], 
                            p.LINK_FRAME)

    ## Apply Pd controller to make sure the upperID orientation is always upright
    # Get the current orientation of the upperID
    upperPosition, current_quat = p.getBasePositionAndOrientation(upperID)
    current_euler = p.getEulerFromQuaternion(current_quat)
    # print("current_euler: ", current_euler)

    # Calculate the orientation error (difference between current and desired)
    error = [desired - current for desired, current in zip(desired_orientation, current_euler)]

    # Calculate the angular velocity of the upperID
    _, angular_velocity = p.getBaseVelocity(upperID)

    # Calculate the corrective torque using the PD controller
    corrective_torque = [kp * e - kd * av for e, av in zip(error, angular_velocity)]

    # Apply the corrective torque to the upperID to maintain it upright
    p.applyExternalTorque(objectUniqueId=upperID,
                         linkIndex=-1,
                         torqueObj=corrective_torque,
                         flags=p.WORLD_FRAME)                  

    # Pressing the upperID down
    if i < 10*240:
        joint_states = p.getJointStates(linkID, list(range(numJointsLink)))
        joint_angles = [state[0] for state in joint_states]
        force = [0,0,-2/(sum(joint_angles)+0.05)] # simple feedback control
        p.applyExternalForce(upperID, -1,
                        force, 
                        upperPosition, 
                        p.WORLD_FRAME)
    
    # Mimic elastic cards stack behavior
    p.setJointMotorControlArray(bodyUniqueId=linkID,
                                jointIndices=list(range(numJointsLink)),
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[0,]*numJointsLink,
                                targetVelocities=[0,]*numJointsLink,
                                positionGains=stiffness,
                                velocityGains=damping,
                                forces=[1,]*numJointsLink,)

    # Step simulation
    p.stepSimulation()
    time.sleep(1./240.)

# Disconnect from PyBullet
p.disconnect()