import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# Physics simulation setup
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")

boxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[.3,]*3)
boxId = p.createMultiBody(1, boxId, -1, [0, 0, 2])

gripperId = p.loadURDF(fileName='asset/robotiq_3f_gripper_visualization/cfg/robotiq-3f-gripper_articulated.urdf', 
                        basePosition=[0,0,.6], 
                        baseOrientation=p.getQuaternionFromEuler([math.pi/2,0,0]),
                        globalScaling=10,
                        )


# maxForce = 500
# for i in [1,5,9]:
#     p.setJointMotorControl2(jointIndex=i,
#                             bodyIndex=gripperId,
#                             controlMode=p.VELOCITY_CONTROL,
#                             targetVelocity = .1,
#                             force = maxForce)
# for i in [2,6,10]:
#     p.setJointMotorControl2(jointIndex=i,
#                             bodyIndex=gripperId,
#                             controlMode=p.VELOCITY_CONTROL,
#                             targetVelocity = .06,
#                             force = maxForce)    
# for i in [3,7,11]:
#     p.setJointMotorControl2(jointIndex=i,
#                             bodyIndex=gripperId,
#                             controlMode=p.VELOCITY_CONTROL,
#                             targetVelocity = .03,
#                             force = maxForce)    


# Simulation loop
time.sleep(1)
for i in range(8000):
    # Apply joint torque when the fingers deviate from initial position to mimic impedance behavior

    # Step simulation
    p.stepSimulation()
    time.sleep(1./240.)

# Disconnect from PyBullet
p.disconnect()