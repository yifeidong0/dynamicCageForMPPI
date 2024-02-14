# test pybullet
import pybullet as p
import time
import pybullet_data
import numpy as np
import csv 

p.connect(p.GUI)
# p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
# p.setGravity(0, 0, -9.8)
# p.setRealTimeSimulation(1)
# p.loadURDF("r2d2.urdf", [0, 0, 1])
# id_gripper = p.loadURDF("asset/real-world/circle-gripper/circle-gripper.urdf", [0,0,0.5], globalScaling=1)
id_gripper = p.loadURDF("asset/real-world/jaw-gripper/jaw-gripper.urdf", [0,0,0.5], globalScaling=1)

# id_object = p.loadURDF("asset/real-world/rectangle-object/rectangle-object.urdf", [0,1,0.5], p.getQuaternionFromEuler([0, 0, 0]), globalScaling=1)
# id_object = p.loadURDF("asset/real-world/triangle-object/triangle-object.urdf", [0,1,0.5], p.getQuaternionFromEuler([0, 0, 0]), globalScaling=1)
# id_object = p.loadURDF("asset/real-world/convex-object/convex-object.urdf", [0,1,0.5], p.getQuaternionFromEuler([0, 0, 0]), globalScaling=1)
id_object = p.loadURDF("asset/real-world/concave-object/concave-object.urdf", [0,0,1], p.getQuaternionFromEuler([0, 0, 0]), globalScaling=1)
# id_object = p.loadURDF("asset/real-world/irregular-object/irregular-object.urdf", [0,1,0.5], p.getQuaternionFromEuler([0, 0, 0]), globalScaling=1)

scale_factor = 10

# read the object's pose and twist from a csv file
# filename = "data/evaluation/real-world/circle-pushes-rectangle/failure/apriltag_results.csv"
# rows = []
# with open(filename, 'r') as file:
#     csv_reader = csv.reader(file)
#     header = next(csv_reader)
#     for id, row in enumerate(csv_reader):
#         rows.append([float(d) for d in row])
# print(len(rows))
# print((rows[0]))

for i in range(100000):
    p.stepSimulation()

    # if i % (240/30) == 0:
    #     # get the pose and twist of the object
    #     print(i//8)
    #     row = rows[i//8]
    #     tvec_world_tag_1_x, tvec_world_tag_1_y, rvec_world_tag_1_z, vel_tvec_world_tag_1_x, vel_tvec_world_tag_1_y, vel_rvec_world_tag_1_z = row[2:8]
    #     tvec_world_tag_2_x, tvec_world_tag_2_y, rvec_world_tag_2_z, vel_tvec_world_tag_2_x, vel_tvec_world_tag_2_y, vel_rvec_world_tag_2_z = row[9:]

    #     # update the object's pose and twist
    #     p.resetBasePositionAndOrientation(id_object, [tvec_world_tag_2_y*scale_factor, tvec_world_tag_2_x*scale_factor, 0.03/2*scale_factor], p.getQuaternionFromEuler([0, 0, -rvec_world_tag_2_z]))
    #     p.resetBaseVelocity(id_object, [vel_tvec_world_tag_2_y*scale_factor, vel_tvec_world_tag_2_x*scale_factor, 0], [0, 0, -vel_rvec_world_tag_2_z])

    #     # update the gripper's pose and twist
    #     p.resetBasePositionAndOrientation(id_gripper, [tvec_world_tag_1_y*scale_factor, tvec_world_tag_1_x*scale_factor, 0.03/2*scale_factor], p.getQuaternionFromEuler([0, 0, -rvec_world_tag_1_z]))
    #     p.resetBaseVelocity(id_gripper, [vel_tvec_world_tag_1_y*scale_factor, vel_tvec_world_tag_1_x*scale_factor, 0], [0, 0, -vel_rvec_world_tag_1_z])

    time.sleep(3./240.)
