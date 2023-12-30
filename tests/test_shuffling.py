

########################################################### shuffling
import pybullet as p
import pybullet_data
import time
import numpy as np

# Physics simulation setup
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# Create two cubes
# height = 3.7
# box1ID = p.loadURDF("cube.urdf", basePosition=[-3, 0, height])
linkID = p.loadURDF("asset/linkage.urdf", basePosition=[0, 0, 4.7], baseOrientation=p.getQuaternionFromEuler([0,np.pi/2,0]))
# fishID = p.loadURDF("asset/fine-fish-10/fine-fish-10.urdf", basePosition=[0, -3, 0.5])
# ballID = p.createCollisionShape(p.GEOM_SPHERE, radius=.3)
# m = 1e-3
# ballID = p.createMultiBody(m, 
#                             ballID, 
#                             -1, 
#                             [0, 0, height])
m = 1
upperID = p.createCollisionShape(p.GEOM_BOX, halfExtents=[10,4,1])
upperID = p.createMultiBody(m, upperID, -1, [0,0,6.5])
p.changeDynamics(upperID, -1, lateralFriction=1, spinningFriction=0, 
                    rollingFriction=0, linearDamping=0, angularDamping=0)

lowerID = p.loadURDF("plane.urdf")

# Joint parameters
joint_number = 2  # Joint index, change if a different joint should be controlled
target_velocity = 1  # rad/s, change to the desired constant velocity

# Retrieve the number of joints and print their info
num_joints = p.getNumJoints(linkID)
for i in range(num_joints):
    print(p.getJointInfo(linkID, i))

# Set the desired velocity for the first joint and disable the motor to control it directly
# p.setJointMotorControl2(bodyUniqueId=linkID,
#                         jointIndex=2,
#                         controlMode=p.VELOCITY_CONTROL,
#                         targetVelocity=target_velocity,
#                         force=500)  # Set a suitable force value
    
# # Spring parameters
# rest_length = 3
# k = 8  # Spring constant

# # Create a fixed joint between the cubes, acting like a spring
# c_spring = p.createConstraint(box1ID, -1, ballID, -1, p.JOINT_FIXED, [1, 0, 0], [rest_length/2, 0, 0], [-rest_length/2, 0, 0])

# # Create point-to-point constraints to keep boxes along a line
# pivotInBox = [0, 0, 0] 
# pivotInWorld = [0, 0, height]
# constraint1 = p.createConstraint(box1ID, -1, -1, -1, p.JOINT_PRISMATIC, [1, 0, 0], pivotInBox, pivotInWorld)
# constraint2 = p.createConstraint(ballID, -1, -1, -1, p.JOINT_PRISMATIC, [1, 0, 0], pivotInBox, pivotInWorld)

# Simulation loop
time.sleep(1)
for i in range(8000):
    upperPosition, _ = p.getBasePositionAndOrientation(upperID)
    p.applyExternalForce(upperID, -1, # gravity
                        [0,0,8*m], 
                        upperPosition, 
                        p.WORLD_FRAME)
    
    # # Update the maxForce for the spring constraint
    # maxForce = k * np.abs(np.linalg.norm(np.array(box1Position) - np.array(box2Position)) - rest_length)
    # p.changeConstraint(c_spring, maxForce=maxForce)

    # Step simulation
    p.stepSimulation()
    time.sleep(10./240.)

# Disconnect from PyBullet
p.disconnect()