
####################### VHACD

import pybullet as p
import pybullet_data as pd
import os
p.connect(p.DIRECT)
name_in = os.path.join(pd.getDataPath(), "/home/yif/Documents/git/dynamicCageForMPPI/asset/real-world/jaw-gripper/jaw-gripper.obj")
name_out = "/home/yif/Documents/git/dynamicCageForMPPI/asset/real-world/jaw-gripper/jaw-gripper-vhacd.obj"
name_log = "log.txt"
p.vhacd(name_in, name_out, name_log)