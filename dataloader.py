from pomp.example_problems.cageDatasetGenerator import *
import time

# TODO: dataset is generated in two ways:
# 1. sim in Pybullet (high quality with object on the top of gripper)
# 2. random C-space sampling (low quality)

mass_object = 1
mass_gripper = 10
moment_gripper = 1 # moment of inertia
half_extents_gripper = [.5, .1] # movement on x-z plane
radius_object = 0.01
params = [mass_object, mass_gripper, moment_gripper, 
                half_extents_gripper, radius_object]
y_range = 10

sim = data_generator_sim(params)

states = [[2, 4, 0, 0, 2, 4.11, 0, 0, 0, 0]]
inputs = [[0.26041288977332316, -6.204326694977351, -17.95028134004808, -0.008637703247645373]]

# Generate random states and inputs
xo = 10*random.random()
yo = 10*random.random()
yo_goal = 4
start_state = [2,yo,
               0,0,
               2,yo+radius_object+half_extents_gripper[1],0,
               0,0,0]

# time.sleep(3.5)

for i in range(len(inputs)):
    q, mu = toBulletStateInput(states[i], inputs[i], y_range)
    sim.reset_states(q)

    q_new = sim.run_forward_sim(mu)
    x_new = toOpenglStateInput(q_new, y_range)
    print("x_new",x_new)

sim.finish_sim()
