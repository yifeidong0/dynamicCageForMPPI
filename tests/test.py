# import pybullet as p
# import pybullet_data
# import time
# import math 

# # Initialize the PyBullet physics simulation
# p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())

# # Set the gravity
# p.setGravity(0, 0, -9.81)  # Earth's gravity (9.81 m/s^2 in the downward direction)

# # Create a box
# box_start_pos = [0, 0, 0.5]  # Initial position of the box (x, y, z)
# box_half_extents = [0.5, 0.5, 0.5]  # Half-extents of the box (x, y, z)
# objectId = p.createCollisionShape(p.GEOM_BOX, halfExtents=box_half_extents)
# box_id = p.createMultiBody( 1, 
#                             objectId, 
#                             -1, 
#                             box_start_pos)
# # box_id = p.createBox(
# #     halfExtents=box_half_extents,
# #     basePosition=box_start_pos,
# #     baseMass=1.0,
# #     collisionShapeType=p.GEOM_BOX
# # )

# # Set the initial velocity of the box (upward)
# initial_velocity = [-1, 0, 0.0]  # 2.0 m/s upward velocity
# p.resetBaseVelocity(box_id, linearVelocity=initial_velocity)
# # p.resetBasePositionAndOrientation(box_id, [0, 0, 0], p.getQuaternionFromEuler([0, -2, 0]))

# # Simulation loop
# for _ in range(10000):
#     p.applyExternalTorque(box_id, -1, 
#                         [0, -4, 0], # gravity compensated 
#                         # [0, 0, 0], 
#                         p.LINK_FRAME)
#     pos_object, ori = p.getBasePositionAndOrientation(box_id)

#     euler = p.getEulerFromQuaternion(ori)
#     print("euler", euler)
#     if euler[0] > -0.8 and euler[0] < 0.8:
#         y = euler[1]
#     elif (euler[0] > 3 or euler[0] < -3) and euler[1] > 0:
#         y = math.pi - euler[1]
#     elif (euler[0] > 3 or euler[0] < -3) and euler[1] < 0:
#         y = -math.pi - euler[1]
#     print("y", y)
#     ori_new = p.getQuaternionFromEuler([0, y, 0])
#     # p.resetBasePositionAndOrientation(box_id, pos_object, ori_new)

#     print('!!!!', p.getEulerFromQuaternion(ori_new))
#     print(box_id)
#     p.stepSimulation()
#     time.sleep(20/240)

# # Close the PyBullet simulation
# p.disconnect()


####################################
# import pybullet as p
# import time
# import pybullet_data
# physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
# p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
# p.setGravity(0,0,-10)
# planeId = p.loadURDF("plane.urdf", basePosition=[0,0,-1], baseOrientation=[0,0,0,1])
# startPos = [0,0,1]
# startOrientation = p.getQuaternionFromEuler([0,0,0])
# boxId = p.loadURDF("asset/4face-bottle.urdf", startPos, startOrientation)
# #set the center of mass frame (loadURDF sets base link frame)
# # startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos,
# # startOrientation)
# # for i in range (10000):
# #     p.stepSimulation()
# #     time.sleep(1./240.)
# objectId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[.2, .6, .1])
# objectUid = p.createMultiBody(1, 
#                             objectId, 
#                             -1, 
#                             [0,0,4],)
#                             # self.quat_object
# force_on_object = [0,0,1]
# torque_on_object = [0,0,1]
# pos_object,quat_object = p.getBasePositionAndOrientation(objectUid)
# for i in range(int(2400)):
#     # Apply external force on object
#     p.applyExternalForce(objectUid, -1, 
#                         force_on_object, # gravity compensated 
#                         pos_object,
#                         p.WORLD_FRAME)
#     # p.applyExternalTorque(objectUid, -1, 
#     #                     torque_on_object,
#     #                     p.WORLD_FRAME)
#     p.stepSimulation()
#     pos_object,quat_object = p.getBasePositionAndOrientation(objectUid)
#     eul_object = p.getEulerFromQuaternion(quat_object) # rad
#     print(pos_object)
#     time.sleep(2./240.)
# p.disconnect()

####################### IRC
# import numpy as np

# def rotate_point_around_another_point(point, center, angle):
#     """
#     Rotate a point around another point by a given angle.

#     Parameters:
#     point: The point to rotate (x, y).
#     center: The center of rotation (x, y).
#     angle: The rotation angle in radians.
    
#     Returns:
#     The rotated point (x, y).
#     """
#     # Translate point back to origin
#     temp_point = [point[0] - center[0], point[1] - center[1]]

#     # Rotate point
#     rotated_temp_point = [temp_point[0] * np.cos(angle) - temp_point[1] * np.sin(angle),
#                           temp_point[0] * np.sin(angle) + temp_point[1] * np.cos(angle)]

#     # Translate point back to the original location
#     rotated_point = [rotated_temp_point[0] + center[0], rotated_temp_point[1] + center[1]]
    
#     return rotated_point

# def find_new_position(x, y, vx, vy, omega, theta1, theta2):
#     """
#     Find the new position of the object after time T.

#     Parameters:
#     x, y: Initial position of the object.
#     vx, vy: Initial velocity components.
#     omega: Angular velocity.
#     T: Time duration.
    
#     Returns:
#     New position (x, y) of the object.
#     """
#     # Angular displacement
#     angular_displacement = theta2 - theta1

#     # Correct for wrapping
#     if angular_displacement > np.pi:
#         angular_displacement -= 2 * np.pi
#     elif angular_displacement < -np.pi:
#         angular_displacement += 2 * np.pi

#     # Time duration for the change in orientation
#     T = angular_displacement / omega if omega != 0 else 0

#     # Calculate the radius (distance to ICR)
#     R = np.sqrt(vx**2 + vy**2) / np.abs(omega) if omega != 0 else np.inf

#     # Direction perpendicular to the velocity vector (rotate velocity vector by 90 degrees)
#     if omega > 0:
#         dir_x, dir_y = -vy / np.sqrt(vx**2 + vy**2), vx / np.sqrt(vx**2 + vy**2)
#     elif omega < 0:
#         dir_x, dir_y = vy / np.sqrt(vx**2 + vy**2), -vx / np.sqrt(vx**2 + vy**2)
#     else:
#         dir_x, dir_y = 0, 0

#     # Calculate ICR position
#     icr_x = x + dir_x * R
#     icr_y = y + dir_y * R

#     # Calculate rotation angle
#     theta = omega * T

#     # Calculate the new position
#     new_x, new_y = rotate_point_around_another_point([x, y], [icr_x, icr_y], theta)

#     return new_x, new_y

# def rotate_vector(vector, angle):
#     """
#     Rotate a 2D vector by a given angle.

#     Parameters:
#     vector: A tuple or list representing the vector (vx, vy).
#     angle: The rotation angle in radians.
    
#     Returns:
#     The rotated vector (vx_new, vy_new).
#     """
#     rotation_matrix = np.array([
#         [np.cos(angle), -np.sin(angle)],
#         [np.sin(angle),  np.cos(angle)]
#     ])

#     return np.dot(rotation_matrix, vector)

# def calculate_new_velocity(vx, vy, omega, theta1, theta2):
#     """
#     Calculate the new velocity of the object after time T.

#     Parameters:
#     vx, vy: Initial velocity components.
#     omega: Angular velocity.
#     T: Time duration.
    
#     Returns:
#     New velocity components (vx_new, vy_new).
#     """
#     # Angular displacement
#     angular_displacement = theta2 - theta1

#     # Correct for wrapping
#     if angular_displacement > np.pi:
#         angular_displacement -= 2 * np.pi
#     elif angular_displacement < -np.pi:
#         angular_displacement += 2 * np.pi

#     # Time duration for the change in orientation
#     T = angular_displacement / omega if omega != 0 else 0

#     # Angular displacement
#     theta = omega * T

#     # Rotate the velocity vector
#     vx_new, vy_new = rotate_vector([vx, vy], theta)

#     return vx_new, vy_new


# # Example usage
# x, y = 0, 0                # Initial position
# vx, vy = 1, 1              # Initial velocity
# omega = np.pi / 4          # Angular velocity (radians per second)
# T = 2                      # Time duration (seconds)
# theta1, theta2 = 0, omega*T

# new_x, new_y = find_new_position(x, y, vx, vy, omega, theta1, theta2)
# print(f"New Position: ({new_x}, {new_y})")

# vx_new, vy_new = calculate_new_velocity(vx, vy, omega, theta1, theta2)
# print(f"New Velocity: ({vx_new}, {vy_new})")

###########################################################
# import pybullet as p
# import time
# import math

# import pybullet_data
# p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())

# p.loadURDF("plane.urdf")
# cubeId = p.loadURDF("cube_small.urdf", 0, 0, 1)
# p.setGravity(0, 0, -10)
# p.setRealTimeSimulation(1)
# cid = p.createConstraint(cubeId, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])
# print(cid)
# print(p.getConstraintUniqueId(0))
# a = -math.pi
# while 1:
#   a = a + 0.01
#   if (a > math.pi):
#     a = -math.pi
#   time.sleep(.01)
#   p.setGravity(0, 0, -10)
#   pivot = [a, 0, 1]
#   orn = p.getQuaternionFromEuler([a, 0, 0])
#   p.changeConstraint(cid, pivot, jointChildFrameOrientation=orn, maxForce=50)

# p.removeConstraint(cid)


########################################################### Box pivot
# import pybullet as p
# import pybullet_data
# import time
# import numpy as np

# # Physics simulation setup
# p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0, 0, -10)
# planeId = p.loadURDF("plane.urdf")

# # Create two cubes
# height = 3.7
# box1ID = p.loadURDF("cube.urdf", basePosition=[-3, 0, height])
# # ballID = p.loadURDF("cube.urdf", basePosition=[0, 0, 4.4])
# ballID = p.createCollisionShape(p.GEOM_SPHERE, radius=.3)
# m = 1e-3
# ballID = p.createMultiBody(m, 
#                             ballID, 
#                             -1, 
#                             [0, 0, height])
# gripperId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[2,2,2])
# m = .3
# box3ID = p.createMultiBody(m, 
#                             gripperId, 
#                             -1, 
#                             [6,0,2])

# p.changeDynamics(box1ID, -1, lateralFriction=0, spinningFriction=0, 
#                     rollingFriction=0, linearDamping=0, angularDamping=0)
# p.changeDynamics(ballID, -1, lateralFriction=0, spinningFriction=0, 
#                     rollingFriction=0, linearDamping=0, angularDamping=0)
# p.changeDynamics(box3ID, -1, lateralFriction=1, spinningFriction=0, 
#                     rollingFriction=0, linearDamping=0, angularDamping=0)

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

# # Simulation loop
# time.sleep(1)
# for i in range(8000):
#     box3Position, _ = p.getBasePositionAndOrientation(box3ID)
#     p.applyExternalForce(box3ID, -1, # gravity
#                         [0,0,-9.81*m], 
#                         box3Position, 
#                         p.WORLD_FRAME)
    
#     p.applyExternalForce(box1ID, -1, # push spring
#                         [1,0,0], 
#                         [0,0,0], 
#                         p.LINK_FRAME)
#     if i > 800:
#         p.applyExternalForce(box3ID, -1, # push box3
#                             [2,0,0], 
#                             box3Position, 
#                             p.WORLD_FRAME)
#         # p.applyExternalTorque(box3ID, -1, # rotate box3
#         #                     [0,-5,0], 
#         #                     p.WORLD_FRAME)
    
#     # Get positions of the boxes
#     box1Position, _ = p.getBasePositionAndOrientation(box1ID)
#     box2Position, _ = p.getBasePositionAndOrientation(ballID)
    
#     # Update the maxForce for the spring constraint
#     maxForce = k * np.abs(np.linalg.norm(np.array(box1Position) - np.array(box2Position)) - rest_length)
#     p.changeConstraint(c_spring, maxForce=maxForce)

#     # Step simulation
#     p.stepSimulation()
#     time.sleep(1./240.)

# # Disconnect from PyBullet
# p.disconnect()
