
####################### VHACD

import pybullet as p
import pybullet_data as pd
import os
p.connect(p.DIRECT)
name_in = os.path.join(pd.getDataPath(), "/home/yif/Documents/KTH/research/dynamicCaging/cad/bottle/5face-bottle.obj")
name_out = "5face-bottle-vhacd.obj"
name_log = "log.txt"
p.vhacd(name_in, name_out, name_log)