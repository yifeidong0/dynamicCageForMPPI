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


####################### VHACD

# import pybullet as p
# import pybullet_data as pd
# import os
# p.connect(p.DIRECT)
# name_in = os.path.join(pd.getDataPath(), "/home/yif/Documents/KTH/research/dynamicCaging/cad/bottle/5face-bottle.obj")
# name_out = "5face-bottle-vhacd.obj"
# name_log = "log.txt"
# p.vhacd(name_in, name_out, name_log)


####################### forward sim of calculated escape path
# Test
from pomp.example_problems.waterswing import *

data = [5.0,3.033163020833334,-0.0,0.0,-0.040875,0.0,
        5.011899972172689,3.333474942671044,-0.01189997217268861,2.855993321445266,0.03398624105058318,-2.855993321445266]
gipper_vel = data[-3:]
# mass_object = .1
# mass_gripper = 10
# moment_gripper = 1 # moment of inertia
# half_extents_gripper = [.7, .4] # movement on x-z plane
# radius_object = 0.01
# params = [mass_object, mass_gripper, moment_gripper, half_extents_gripper, radius_object]
dynamics_sim = forwardSimulationWaterSwing(gui=1)
cage = WaterSwing(data, dynamics_sim)
cage.controlSpace()

# From high to low
# data = [1.02, 5.11, 0.00, 0,
#             1.01, 4.70, -0.00, 0.00, 1, -0.50]
# gipper_vel = data[-3:]
time.sleep(2.5)

# energy labeler
states = [[5.0, 3.033163020833334, -0.0, 0.0, -0.040875, 0.0, 5.011899972172689, 3.333474942671044, -0.01189997217268861], [5.5671763414601045, 3.310336015142659, -0.5378715161364246, 2.429562791191379, 2.3553821156594243, -2.645144140961709, 5.5555546867799785, 3.339473834378901, -0.5536200713384032], [5.759711349707775, 3.590924974005998, -1.108346759833577, -0.37977872183612765, 0.3470654088866472, -2.8537926646014853, 6.115355319637294, 3.3469744060969555, -1.1122187159534802], [5.756722614305164, 3.3855824060819906, -1.5402775583286359, 0.4883287620798012, -2.109167298530658, -1.2721340377225723, 6.62715630089533, 3.35434589871918, -1.520660219733035], [5.740470014626494, 2.9843126916143463, -1.4090764428839775, -0.6633182575055041, -2.9376993050983327, -1.1591177883413206, 7.0793552434575, 3.3597270535521857, -1.1687334912945921], [5.533820355457216, 2.4870487054838124, -1.5371282883860657, -1.6743206475239296, -2.749791670549409, -1.0758691486392784, 7.5791540747104245, 3.3656746457360343, -1.473060331042279], [5.30599605557949, 2.122158621941426, -1.478154106550573, -2.212034705453876, -3.479409137776254, -1.0889812806983892, 7.912353295545707, 3.3696397071919333, -1.3353331017122338], [5.090052431744534, 1.7941506772169156, -1.5707963267948966, -2.700556605986409, -3.993437759647297, -1.1102468224961193, 8.162252711172169, 3.3726135032838576, -1.556360136251105], [4.648952193108082, 1.159618938361709, -1.4091652069999145, -3.330810831721101, -4.688843078402528, -1.1065151537978635, 8.578751737216272, 3.3775698301037314, -1.1687334912945875], [4.349681161532462, 0.7172683345099153, -1.5285735097287443, -2.450409045961191, -3.838309844892318, -1.1830623699495812, 8.87625104153349, 3.3811100635464983, -1.4662327956118038]]
inputs = [[0.19558328515278664, -9.677691218407459, 9.696611199886338, -0.22278282788686088], [0.1960827013593753, -9.337849710871719, -0.5967729186353861, 1.7721754777210723], [0.17954364703737855, 5.205342367555696, -0.6738408588952858, 0.09114237855143603], [0.15998668953522374, -7.273560123696663, -5.232833725690551, 2.141360514592142], [0.1774081493441613, -5.777156514391002, 1.0737579117081069, 1.4271195377492987], [0.11880611129619295, -4.608977639399525, -6.253864004801571, -0.33716911009149353], [0.08930153395490718, -5.583107434657482, -5.874612821383356, -0.72910429020782], [0.14961615177778415, -4.321743262180789, -4.768493614321687, 0.0767657560784678], [0.10791765282695472, 8.451857143295125, 8.165119041698059, -2.204559825169508]]
print(len(states), len(inputs))
# dynamics_sim.reset_states(states[0]+gipper_vel)
for i in range(len(inputs)):
    dynamics_sim.reset_states(states[i]+gipper_vel)
    new_states, viapoints = dynamics_sim.run_forward_sim(inputs[i])
    print('new_states', new_states)

# # ball balance
# states = [[1.02, 5.11, 1.01, 4.7, -0.0], [4.304206592094783, 5.195005075715069, 1.0126748209851473, 4.754665910584282, -0.3832874049208745], [4.589857682473735, 1.4220963753814768, 1.0126748209851473, 5.1671659105842584, -0.58691567122751]]
# inputs = [[0.4341884048103374, 7.5790919267402685, -8.60582670121947], [0.41600639801144806, 0.6924874918276647, -9.146445334142033]]
# for i in range(len(inputs)):
#     state = states[i][:2]+inputs[i][1:]+states[i][2:]+gipper_vel
#     sim.reset_states(state)
#     new_states = sim.run_forward_sim_ball_balance(inputs[i][0])
#     print('new_states', new_states)

dynamics_sim.finish_sim()