from pomp.example_problems.balancegrasp import *
from pomp.example_problems.planepush import *
from pomp.example_problems.waterswing import *
from pomp.example_problems.boxpivot import *
from pomp.example_problems.shuffling import *
from pomp.example_problems.gripper import *
from pomp.bullet.scriptedmovement import *

problem_name = "BoxPivot" # "PlanePush", "BalanceGrasp", "BoxPivot", "Gripper", "Shuffling", "WaterSwing", 
gui = 1
num_via_points = 10
num_trajs = 1

if problem_name == 'PlanePush':
    total_time = 2.5
    num_state_planner = 9
    fake_data = [5.0, 6.3, 0.0, 0.0, 0.0, 0.0, 
                 5.0, 6.0, 0.0, 0.0, 2.0, 0.0]
    dynamics_sim = forwardSimulationPlanePush(gui=0)
    cage = PlanePush(fake_data, dynamics_sim)
    x_init = fake_data
    dynamics_sim.finish_sim()
    sim = scriptedMovementSimPlanePush(cage, gui=gui)
if problem_name == 'BalanceGrasp':
    total_time = 3
    num_state_planner = 9
    fake_data = [5.0, 4.3, 0.0, 0.0, 0.0, 0.0, 
                 5.0, 4.0, 0.0, 0.0, 0.0, 0.0]
    dynamics_sim = forwardSimulationBalanceGrasp(gui=0)
    cage = BalanceGrasp(fake_data, dynamics_sim)
    x_init = fake_data
    dynamics_sim.finish_sim()
    sim = scriptedMovementSimBalanceGrasp(cage, gui=gui)
if problem_name == 'BoxPivot':
    total_time = 2.5
    num_state_planner = 8
    fake_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, ]
    dynamics_sim = forwardSimulationBoxPivot(gui=0)
    cage = BoxPivot(fake_data, dynamics_sim)
    x_init = [6, 2, 0, 0, 0, 0,
              2, 3.7, 0, 0]
    # x_init = [3, 0.5, 0, 0, 0, 0, # for_paper_vis
    #           0, 1, 0, 0]
    dynamics_sim.finish_sim()
    sim = scriptedMovementSimBoxPivot(cage, gui=gui)
if problem_name == 'Gripper':
    total_time = 2
    num_state_planner = 6+6+9+1
    fake_data = [-0.99, 0.6, 0.0, 0.0, 0.0, 0.0] + [0.0,]*6 + [0*math.pi/12]*9 + [0.0,] + [0.0]*9 + [0.0,]
    dynamics_sim = forwardSimulationGripper(gui=0)
    cage = Gripper(fake_data, dynamics_sim)
    x_init = fake_data
    dynamics_sim.finish_sim()
    sim = scriptedMovementSimGripper(cage, gui=gui)


dataset = []
heuriset = []
success_labelset = []
capture_labelset = []
for i in range(num_trajs):
    if problem_name == 'BoxPivot' or problem_name == 'Gripper':
        sim.sample_init_state()
    elif problem_name == 'PlanePush' or problem_name == 'BalanceGrasp':
        x_init = sim.sample_init_state()
    
    sim.reset_states(x_init)
    if problem_name == 'BoxPivot':
        _ = sim.run_forward_sim(num_via_points=1, do_cutdown_test=1) # get cutdown time
        sim.reset_states(x_init)
        x_news = sim.run_forward_sim(sim.cutoff_t, num_via_points, do_cutdown_test=0)
    else:
        x_news = sim.run_forward_sim(total_time, num_via_points)
    heuristics = sim.heuristics_traj

sim.finish_sim()
