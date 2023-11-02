import pybullet as p
import pybullet_data
import time

# Initialize PyBullet and set up the simulation
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)  # Set gravity in the Z direction

# Load the gripper and ball objects
# gripper = p.loadURDF("gripper.urdf", [0, 0, 0.1])
# ball = p.loadURDF("sphere.urdf", [0, 0, 0.2])
# planeId = p.loadURDF("plane.urdf")
# planeId = p.loadURDF("mini_cheetah/mini_cheetah.urdf")
# boxId = p.loadURDF("r2d2.urdf")
sphereRadius = 1
mass = 1
visualShapeId = -1
baseOrientation = [0, 0, 0, 1]
basePosition = [0,0,1]
cylinderId = p.createCollisionShape(p.GEOM_CYLINDER, radius=sphereRadius)
boxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[sphereRadius, sphereRadius, sphereRadius])
cylinderUid = p.createMultiBody(mass, cylinderId, visualShapeId, basePosition,
                                      baseOrientation)
basePosition = [3,1,1]
boxUid = p.createMultiBody(mass, boxId, visualShapeId, basePosition,
                                      baseOrientation)

# # Set the gripper's initial pose and control parameters
# initial_gripper_pos = [0, 0, 0.2]
# p.resetBasePositionAndOrientation(gripper, initial_gripper_pos, [0, 0, 0, 1])
# p.changeDynamics(gripper, -1, linearDamping=0.1, angularDamping=0.1)

# Simulation parameters
time_step = 1.0 / 240.0  # Simulation time step (adjust as needed)
num_steps = 10000  # Number of simulation steps

for step in range(num_steps):
#     # Apply accelerations to the gripper (change these values based on your control strategy)
#     gripper_acc_x = 0.1  # Acceleration along the X-axis
#     gripper_acc_y = 0.1  # Acceleration along the Y-axis
#     p.applyExternalForce(gripper, -1, [gripper_acc_x, gripper_acc_y, 0], [0, 0, 0], p.LINK_FRAME)

    # Step the simulation
    p.stepSimulation()

#     # Get the gripper and ball states
#     gripper_pos, _ = p.getBasePositionAndOrientation(gripper)
#     ball_pos, ball_vel, _, _, _, _, _, _, _ = p.getLinkState(ball, 0, computeLinkVelocity=1)

#     # Print the positions and velocities (for example)
#     print(f"Step {step}: Gripper Pos: {gripper_pos}, Ball Pos: {ball_pos}, Ball Vel: {ball_vel}")

    # Sleep to control the simulation speed (adjust as needed)
    time.sleep(time_step)

# Clean up and close the simulation
p.disconnect()