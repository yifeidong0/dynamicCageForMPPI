# import pybullet as p
# import pybullet_data
# import time

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

# # Simulation loop
# for _ in range(1000):
#     p.applyExternalForce(box_id, -1, 
#                         [0, 0, 9.81], # gravity compensated 
#                         [0, 0, 0], 
#                         p.LINK_FRAME)

#     p.stepSimulation()
#     time.sleep(10/240)

# # Close the PyBullet simulation
# p.disconnect()



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


import pybullet as p
import pybullet_data as pd
import os
p.connect(p.DIRECT)
name_in = os.path.join(pd.getDataPath(), "/home/yif/Documents/KTH/research/dynamicCaging/cad/bottle/5face-bottle.obj")
name_out = "5face-bottle-vhacd.obj"
name_log = "log.txt"
p.vhacd(name_in, name_out, name_log)