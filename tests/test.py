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


####################### forward sim of calculated escape path WATER SWING
# Test
# from pomp.example_problems.waterswing import *

# # data = [5.0,3.033163020833334,-0.0,0.0,-0.040875,0.0,5.022439947525641,3.333501183748318,-0.007479982508547126,5.385587406153931,0.040284099596283096,-1.7951958020513104]
# data = [5.074644055136539,9.600087506924961,-3.026508729636365,-5.920500860019322,0.08414776571064868,-1.6569289931874707,4.977230984628767,9.260130976322039,-3.027816115766067,-5.355934681080274,-0.0864626138285583,-1.690548122182003]
# gripper_vel = data[-3:]
# dynamics_sim = forwardSimulationWaterSwing(gui=1)
# cage = WaterSwing(data, dynamics_sim)
# cage.controlSpace()
# time.sleep(2.5)

# # energy labeler
# # CENTRIPETAL FORCE FORMULATION
# states = [[5.074644055136539, 9.600087506924961, -3.026508729636365, -5.920500860019322, 0.08414776571064868, -1.6569289931874707, 4.977230984628767, 9.260130976322039, -3.027816115766067], [4.555974200111394, 9.603242189602899, 3.1080502600001423, -5.94127955378561, -0.003926065357463208, -1.7039772203123287, 4.508660777543481, 9.252709626570386, 3.1075199853497297], [3.9735525750220972, 9.513972375589606, 2.97391852127864, -7.365864457848442, -1.2884172369922906, -1.597992121741334, 4.046208595514666, 9.17652060322789, 2.959457248103738], [3.51135658485565, 9.347922359944286, 2.839153905591779, -5.282259736623044, -2.584915589168772, -1.8505858689084531, 3.663528112596285, 9.05337567889426, 2.8322603710603302], [3.2939433401149407, 9.228984997344758, 2.764921129993117, -5.147412485938431, -3.062716994863535, -1.5940123636312928, 3.461521218109792, 8.958459282632061, 2.7618153523130813], [3.019308805073901, 9.064168249078945, 2.675096991622076, -5.038775291854997, -3.048124189949817, -1.688725078561367, 3.2082160781146265, 8.816865585575579, 2.670190630084611], [2.6244720156870205, 8.708287139890045, 2.5137508881910775, -3.6512663833908396, -4.646707195352613, -1.7946399792252588, 2.803316361064061, 8.539075934702032, 2.5152198268588277], [2.48283074348738, 8.50338384161128, 2.4395236654760963, -3.1933102913918185, -5.139383492330339, -1.770664280923968, 2.6409688749197047, 8.385914520169466, 2.4447803217679107], [2.3133159450283878, 8.129800506517459, 2.3273191019663324, -1.9367529796301135, -5.959756030659656, -1.230866842349381, 2.399102627552731, 8.123178762055435, 2.3320692348425824], [2.231751189118909, 7.7313080546806985, 2.2283452244299724, -0.6180312608039285, -6.037327675129212, -1.6426095728623984, 2.1883069640316894, 7.83487984564111, 2.219381498496321], [2.21969182527512, 7.553378873221592, 2.1754630636200285, -0.23600804140861073, -6.083481796470004, -1.4655719912744436, 2.110852763276432, 7.699180603137185, 2.170087602405323], [2.2413838593049222, 7.090922989966355, 2.040914566166153, 0.6958480374437015, -5.656500696380489, -1.6607569437691245, 1.9189607038722962, 7.320747193193736, 2.036508860582316], [2.3627962724673113, 6.596940776426274, 1.887575329967627, 1.8691895871310094, -5.11327215411381, -1.5849643713133748, 1.757106914216606, 6.856857117823072, 1.8821257590031186], [2.5587737331106464, 6.140059511834214, 1.7888657497399871, 2.19010682896681, -4.633081360797089, -0.49215807598688277, 1.6644816589320162, 6.351735005747292, 1.7206592635676312], [2.739202211500657, 5.740040852314065, 1.8419502232350005, 2.1467187888724086, -5.08210515157647, 1.0873721539744476, 1.6555817345694315, 5.905334423311296, 1.580077617631441], [2.976009172323318, 5.23782399356564, 1.9471010128390207, 2.768313659986852, -5.385740786023406, 1.1062579842704967, 1.717359996502212, 5.395721375554756, 1.4180667559223337], [3.2297829287008017, 4.872276607728541, 2.0269939531725276, 3.9342142548113577, -4.416060623126824, 1.0285381909202684, 1.8294071909117986, 5.009915165469796, 1.2912756467586823], [3.520241352600404, 4.431334124603412, 2.123026308995414, 2.7607162897991753, -5.605962701705705, 1.160216680753642, 2.0159963957012765, 4.579951015095304, 1.1433526860677572], [3.6923330988788488, 3.9365126108608535, 2.2203531053470495, 1.244986928075897, -5.699776025559721, 1.0687513015323906, 2.2639177993372455, 4.182182640824057, 0.9954297253768323], [3.7588592077309126, 3.502016124663334, 2.2977101201706973, 0.566723919563167, -5.876953733699677, 0.998031315226325, 2.524350943919096, 3.876281377802152, 0.8686386162131817], [3.7666458450456104, 2.9683689922226284, 2.3798788592651694, -0.3453132945549289, -6.3005249498884, 0.8854712936736387, 2.870879308329929, 3.560682273541741, 0.7207156555222564], [3.642992694456782, 2.374255747435815, 2.452676609921972, -1.5532208684559543, -6.559249143237516, -1.5804985406372838, 3.297196151202999, 3.2747480333480268, 0.5587561783758354], [3.7256215766067355, 1.5985565274232107, 2.227769955611161, 1.5022311212408401, -8.425113373402821, -1.2794687436633128, 3.7629920640098167, 3.061996669064555, 0.4006533355982899]]
# inputs = [[0.08833340908839188, 3.5381540756519527, 6.13840079314609, 3.688654042587622], [0.09029960902436805, -17.354300625058578, -5.694542545543637, 3.523964056747511], [0.07589926910518918, 7.800457905315778, -10.101568126229097, -1.5088814797419694], [0.043373529065591464, 2.500985724448981, -11.974318530880147, 2.291287936533479], [0.05523445977244858, -1.7103597066625866, -2.5648352741139355, 4.747742346062543], [0.095235331347469, 14.989898590979159, -17.47251958926963, -2.425690366798431], [0.04558534642647555, 10.990946207976506, -11.824231127465495, 1.7262502776929107], [0.070588750119695, 18.205776177985854, -13.386295876190001, 1.663447536585469], [0.06816806374659343, 18.765081636703407, -3.0710238009280424, -1.03588677951308], [0.03242746103079124, 10.251096826421225, -8.735391128582819, 2.277447364929631], [0.08006297677917473, 7.679848178804217, -8.05219703060434, 1.2506432987871925], [0.09525417599245978, 10.759139715792038, -7.604965706804849, -1.5329015346351382], [0.09800071741665346, 3.1406670536005663, -1.518313859636283, -1.3688508678557554], [0.085582240135747, -0.4379001346756972, -9.488090603114397, -4.750729043606236], [0.09678133075908896, 6.486207350759372, -3.1683718377071344, 0.5912086005719619], [0.07524879240615703, 15.545341264326723, 12.92906883862107, -3.108791734009082], [0.09137172054776388, -13.411405314424961, -13.598880898044436, 4.514691080001445], [0.0901337658112804, -17.322621276837495, -1.072152272617231, -3.1359558590143424], [0.07538385244548144, -9.043506780169736, -2.362369441866008, -2.828799452242566], [0.09047741449479957, -10.423282447063954, -4.8408138992997465, -3.8592007389492187], [0.09874469385922269, -19.471914111608992, 2.4508641504637474, -1.2191897891677659], [0.09729494418959511, 3.874551757499706, -1.3048954506088108, 2.259458459135386]]

# # CONSTANT LINEAR VELOCITY
# # states = [[5.0, 3.033163020833334, -0.0, 0.0, -0.040875, 0.0, 5.022439947525641, 3.333501183748318, -0.007479982508547126], [5.95876541555008, 3.3792289745587887, -0.2958511283765771, 5.333132730087204, 3.632813350959119, -1.5962957351614433, 5.959643571602314, 3.3403060103719167, -0.31420578352409523], [6.4369395143500725, 3.91261854784483, -0.5631584341556312, 1.6508714550997965, 3.884597089591947, -1.5068101245569234, 6.72339889435961, 3.3466138328696418, -0.5675621284939881], [6.447554762661514, 4.572986807604476, -0.8104902269830497, -1.4381747963923452, 3.3602032523565644, -1.2404936609879993, 7.710784634114282, 3.354029925215076, -0.8965697687282982], [6.004293434760635, 4.897962261931547, -1.013033945887861, -3.821380983734645, 0.6082997862216579, -1.1912617633116476, 8.608382535139913, 3.3607439418144542, -1.1957690690701832], [5.5662174965159466, 4.790893074608046, -1.1894403212803437, -2.2319304697353535, -2.0020945179671306, -1.227004450994327, 9.39378069853733, 3.3666187063389104, -1.457568456869334], [5.280343310980954, 4.508737598472248, -1.342573703431613, -2.1956779676479754, -2.3553685314427875, -1.146684059461869, 10.089419071832186, 3.3718220692034286, -1.6894479146342951], [5.026065357171852, 3.654781942470994, -1.6128532787924101, 0.015595661377884651, -4.793070295169063, -1.1296532559192836, 11.368496080793694, 3.381389542857543, -2.1158069176214815], [4.708143652692311, 2.151224563856306, -1.9514228712853119, -2.3995210805563865, -6.6333934911651244, -1.4449203520249616, 12.782212774909045, 3.391964119001564, -2.5870458156599505], [4.148964223263807, 0.03788996194276228, -2.3701952295009763, -1.503409894969248, -8.047569307418176, -1.4679449715117383, 14.330569154178239, 3.4035457976354917, -3.103164608749702], [4.1223733062095285, -1.8937505577676352, -2.7201402013111036, 1.2607874338001255, -6.464470509133012, -1.1614315886711037, 15.76672579581923, 3.414288224194497, 2.701301817882868], [5.081357694476469, -3.058472536561743, -2.984807959700907, 5.3511280124246055, -1.7068936457160968, -0.6866167999638666, 17.315082175088424, 3.425869902828425, 2.1851830247931163]]
# # inputs = [[0.17600275645778898, -6.127988380166208, 19.406729626091924, -3.109097405323047], [0.14450939240944294, -11.416577589554645, 13.448650605224898, -3.8938214276107743], [0.18670215858065964, -16.66529705684376, -2.6573414463380196, 2.985829083762468], [0.16923993779563246, -14.2992371240538, -16.51142079680945, 0.8861741581743177], [0.1489783549867931, 10.899089238852273, -17.899846657294542, -0.735278146615105], [0.12955167266980097, 0.2806645322893573, -2.7350246204567377, 1.8655058678505672], [0.23828064518121525, 9.310625806424667, -10.264007426215853, 0.2151259394852314], [0.2644775318940373, -9.200444731178177, -7.010755032365928, -3.6030525269220206], [0.2905189021078567, 3.116908471607431, -4.91887240435843, -0.24025689899238678], [0.269913930064216, 10.365739982885152, 5.9366204935693645, 3.4482755569570642], [0.28870075260840444, 14.227271577824304, 16.5480934379719, 4.954589099553804]]
# print(len(states), len(inputs))
# new_states = states[0]
# for i in range(len(inputs)):
#     vxg_init, vyg_init = gripper_vel[:2]
#     omegag = gripper_vel[2]
#     vxg, vyg = calculate_new_velocity(vxg_init, vyg_init, omegag, states[0][8], new_states[8])
#     print("!!!!", vxg, vyg)
#     # xaug = x + [vxg, vyg, omegag]
#     dynamics_sim.reset_states(states[i]+[vxg, vyg, omegag])
#     new_states, viapoints = dynamics_sim.run_forward_sim(inputs[i])
#     print('new_states', new_states)

# dynamics_sim.finish_sim()

####################### forward sim of calculated escape path Box Pivot
# from pomp.example_problems.boxpivot import *

# data = [7.057175022493289,2.666821472276362,0.4458188048791134,2.8897499124955996,1.037772590214243,1.0853131332758004,3.0909588106585626,5.002371997836917,-0.601473693634434,2.461966018556275]
# gripper_vel = data[-2:]
# dynamics_sim = forwardSimulationBoxPivot(gui=1)
# cage = BoxPivot(data, dynamics_sim)
# cage.controlSpace()
# time.sleep(4.5)

# states= [[7.057175022493289, 2.666821472276362, 0.4458188048791134, 2.8897499124955996, 1.037772590214243, 1.0853131332758004, 3.0909588106585626, 5.002371997836917], [7.398496880003466, 2.7192703778476326, 0.5066874187444336, 3.0751317057143184, 0.03132935480629533, 0.040321185744062814, 2.8985267829571497, 5.3124883663027305], [8.201744728374415, 2.75817366752867, 0.56206661758692, 0.38362916667957103, 0.087235977781876, 0.13909627146766132, 2.530090027665026, 6.034383058307879], [8.008121727193748, 2.703464961838547, 0.4870396145987758, -1.4711380377981575, -0.45039827888026956, -0.5437789456772736, 3.094284184365327, 5.619457675642711], [7.820204452861863, 2.6179981229376277, 0.39765796080816634, -1.0638265306427308, -0.7155139979677786, -0.6828012278881793, 2.941921518063143, 5.780206212843101], [7.638721192813267, 1.9997758378857937, -0.00010654271196966325, 0.004550629266490112, 0.004476893284447558, 0.0022741668672033597, 2.4791103738614138, 5.29812285757004], [7.713941308949088, 1.999986673228017, 8.261086974033578e-07, -1.001107631704739e-05, -2.0531510169785605e-06, 3.608864103908649e-07, 2.231031656787603, 5.366867175920232]]
# inputs= [[0.11442836424982207, -1.2273441513618286, -9.468395720526445], [0.46079878846698, -2.9490013776938335, -0.9093533891400831], [0.4704191462751449, -2.771422451005921, -1.4435677987141133], [0.1451282087869632, -2.3629990259936613, -1.2875300688970555], [0.4207833234839916, -0.3895490817868197, -1.801939495060143], [0.43193319982160544, 0.32992159970150103, -5.397456846772952]]
# new_states = states[0]
# for i in range(len(inputs)):
#     # vxg_init, vyg_init = gripper_vel[:2]
#     # omegag = gripper_vel[2]
#     # vxg, vyg = calculate_new_velocity(vxg_init, vyg_init, omegag, states[0][8], new_states[8])
#     # print("!!!!", vxg, vyg)
#     # # xaug = x + [vxg, vyg, omegag]
#     dynamics_sim.reset_states(states[i]+gripper_vel)
#     new_states, viapoints = dynamics_sim.run_forward_sim(inputs[i])
#     print('new_states', new_states)

# dynamics_sim.finish_sim()

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