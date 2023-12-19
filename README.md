# Dynamic Cage as Stability Measure
MPPI manipulation planning with a cage-based instability cost.

Requirements:
- Python 2.X or 3.X
- Numpy/Scipy
- PyOpenGL
- Matplotlib (optional)

Branches:

1. 
main: initial trials. 
[cage.py]: static obstacle with escaping double-integrator point-mass object.
[cagemovingobstacle.py]: moving obstacle with constant velocity, and escaping double-integrator point-mass object.
[cageplanner.py]: goal-conditioned motion planner with AO-RRT that controls a plate-like robot to transport a point-mass object to goal region.
[dataloader.py]: load data for model training via cageplanner.
[datalabeler.py]: label samples loaded from above via [cageenerylabeler.py].
[test_mppi.py]: goal-conditioned MPPI with instability cage-penalty via [mppi.py]. The same task as above. It also loads samples for [datalabeler.py] via MPPI, which is better.

2. 
bullet-collision: 
[cageenerylabeler.py]: double-integrator point mass system with moving robot gripper that is embedded with Bullet forward dynamics simulation. Edge collision checking is thus not needed anymore.
[planepush.py]: similar to the above task but on a horizontal plane.
[waterswing.py]: Swinging a bottle of water rapidly such that the water does not split.
[boxpivot.py]: Pivoting a box around one of its edges by pushing with a spring. State space - [rot_box_z, pos_box_x, rot_box_z, vel_box_x, pos_spring1_x, pos_spring2_x], constant - [vel_spring1_x, vel_spring2_x] input space - [torque_box_z, force_box_x]
[ballbalance.py]: same task but single-integrator system without any acceleration limits.

3.
single-integrator:
[ballbalance.py]: single-integrator point-mass system with moving robot gripper and acceleration limits.


Usage:

1.
Run test_mppi.py first to generate sample features in a dataset by rolling out MPPI given random starts and goals. 

Then run cagelabeler.py to label each sample by running AO-RRT. Set vis to 0, your desired runtime (sec).


2.
  "python main.py [-v] [PROBLEM] [PLANNER]"

where -v is an optional flag to show the planner in a GUI.  If not specified,
the planner is run 10 times on the given problem and stats are saved to
disk.  You can also name multiple planners.

[PROBLEM] can be any of:
 - "Bugtrap"
 - "Kink"
 - "Pendulum" 
 - "Dubins" 
 - "Dubins2"
 - "DoubleIntegrator"
 - "Flappy"
 - "CageMovingObstacle"

[PLANNER] can be any of
 - "r-rrt"
 - "r-est"
 - "r-rrt-prune"
 - "r-est-prune"
 - "ao-rrt"
 - "ao-est"
 - "stable-sparse-rrt" 
 - "sst*" 
 - "all": run all planners
You may also add keyword arguments to change parameters of the planner, e.g.
"r-rrt(numControlSamples=1)".

Visualization controls:

- 'p' will do 1000 iterations of planning
- ' ' will do a single iteration.
- 'm' will start saving frames to disk every 100 iterations of planning, which
  can be later turned into a movie.

Once data has been saved to disk, you can run:

   "python processresults.py [folder]"

to generate a csv file summarizing all of the results for a single
problem.  If you have matplotlib, you can then view the results using

   "python viewsummary [file]"


