import pybullet as p
from time import sleep
import pybullet_data

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD) # enable Finite Element Method (FEM, mass-spring systems) rather than position based dynamics (PBD)
p.resetDebugVisualizerCamera(3,-420,-30,[0.3,0.9,-2])
p.setGravity(0, 0, -10)

tex = p.loadTexture("uvmap.png")
planeId = p.loadURDF("plane.urdf", [0,0,-2])

boxId = p.loadURDF("cube.urdf", [0,3,2],useMaximalCoordinates = True)

# bunny2 = p.loadSoftBody("torus/torus_textured.obj", simFileName="asset/deformable/torus/torus.vtk", mass = 3, useNeoHookean = 1, NeoHookeanMu = 180, NeoHookeanLambda = 600, NeoHookeanDamping = 0.01, collisionMargin = 0.006, useSelfCollision = 1, frictionCoeff = 0.5, repulsionStiffness = 800)
# p.changeVisualShape(bunny2, -1, rgbaColor=[1,1,1,1], textureUniqueId=tex, flags=0)

# bunny2 = p.loadURDF("asset/deformable/torus/torus_deform.urdf", [0,1,0.5], flags=p.URDF_USE_SELF_COLLISION)

# bunny2 = p.loadURDF("asset/deformable/cube/cube.urdf", [0,1,0.5], flags=p.URDF_USE_SELF_COLLISION)

bunny2 = p.loadSoftBody("asset/deformable/bread/bread.vtk", basePosition = [0,0,-1], scale = 0.5, mass = 1, useMassSpring = 1, springElasticStiffness = 100, useSelfCollision = 1, frictionCoeff = .5, collisionMargin = 0.001)

# bunny2 = p.loadSoftBody("asset/deformable/ball/ball.vtk", basePosition = [0,0,-1], scale = 0.5, mass = 4, useNeoHookean = 1, NeoHookeanMu = 400, NeoHookeanLambda = 600, NeoHookeanDamping = 0.001, useSelfCollision = 1, frictionCoeff = .5, collisionMargin = 0.001)

p.changeVisualShape(bunny2, -1, rgbaColor=[1,1,1,1], textureUniqueId=tex, flags=0)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(0)

while p.isConnected():
  p.stepSimulation()
  p.getCameraImage(320,200)
  p.setGravity(0,0,-10)