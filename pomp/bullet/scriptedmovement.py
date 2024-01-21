from .forwardsimulator import *
from ..structures.toolfunc import *
import random
import pybullet as p
import pybullet_data
import time
import math
import numpy as np

def check_bounds(x, bd):
    # Iterate over elements in x and corresponding bounds in bd
    for i, xi in enumerate(x):
        lower_bound, upper_bound = bd[i]
        if xi < lower_bound or xi > upper_bound:
            return False
    return True

def generate_random_point(bd):
    # Generate a random value for each dimension within its bounds
    random_point = []
    for bounds in bd:
        lower_bound, upper_bound = bounds
        random_value = np.random.uniform(lower_bound, upper_bound)
        random_point.append(random_value)
    return random_point


class scriptedMovementSimPlanePush(forwardSimulationPlanePush):
    def __init__(self, cage, gui=False):
        super().__init__(gui=gui)
        self.set_params(cage.params)
        self.create_shapes()
    
    def sample_init_state(self):
        # init_neutral = [5.0, 4.3, 0.0, 0.0, 0.0, 0.0, 
        #                 5.0, 4.0, 0.0, 0.0, 0.0, 0.0]
        xo = random.uniform(4,6)
        yo = random.uniform(6.2,8.2)
        thetao = random.uniform(-math.pi/18, math.pi/18)
        vxo = random.uniform(-0.0, 0.0)
        vyo = random.uniform(-0.0, 0.0)
        omegao = random.uniform(-0.0, 0.0)
        xg = xo + random.uniform(-0.3, 0.3)
        yg = yo + random.uniform(-0.37, -0.35)
        vxg = random.uniform(-0.0, 0.0)
        vyg = random.uniform(0, 0.2)
        init_state = [xo, yo, thetao, vxo, vyo, omegao,
                      xg, yg, 0, vxg, vyg, 0]
        return init_state

    def run_forward_sim(self, total_time=10, num_via_points=20, do_cutdown_test=False):
        num_steps = int(total_time * 240)  # Number of time steps
        interval = int(num_steps/num_via_points)
        interval = 3 if interval==0 else interval

        # Step the simulation
        via_points = []
        self.heuristics_traj = []
        self.task_success_label = 0
        for t in range(num_steps):
            # Apply external force
            self.pos_object,_ = p.getBasePositionAndOrientation(self.objectUid)
            self.pos_gripper,_ = p.getBasePositionAndOrientation(self.gripperUid)
            rand_force = [random.uniform(-0.4,0.4), random.uniform(8,13), 0]
            p.applyExternalForce(self.gripperUid, -1, 
                                rand_force,
                                #  [0,10,0] ,
                                self.pos_gripper, 
                                p.WORLD_FRAME)

            # Print object via-points along the trajectory for visualization
            if t % interval == 0 or t == int(t*240)-1:
                # Get the object and gripper states
                self.pos_object, self.quat_object = p.getBasePositionAndOrientation(self.objectUid)
                self.eul_object = p.getEulerFromQuaternion(self.quat_object) # rad
                self.vel_object, self.vel_ang_object = p.getBaseVelocity(self.objectUid)
                self.pos_gripper, self.quat_gripper = p.getBasePositionAndOrientation(self.gripperUid)
                self.eul_gripper = p.getEulerFromQuaternion(self.quat_gripper)
                self.vel_gripper,self.vel_ang_gripper = p.getBaseVelocity(self.gripperUid)

                # Get contact forces
                res = p.getContactPoints(self.gripperUid, self.objectUid)
                all_contact_normal_forces = [contact[9] for contact in res]
                contact_normal_force = sum(all_contact_normal_forces) if len(all_contact_normal_forces)>0 else 0.0
                s_engage = contact_normal_force
                contact_friction_force_xy = sum([contact[10] for contact in res]) if len(all_contact_normal_forces)>0 else 0 # friction along z is not considered
                # Sticking quality measure in the paper - Criteria for Maintaining Desired Contacts for Quasi-Static Systems
                s_stick = (self.lateral_friction_coef*contact_normal_force - abs(contact_friction_force_xy)) * math.cos(np.arctan(self.lateral_friction_coef))
                # print("@@@@@@@s_stick: ", s_stick)
                # print("s_engage: ", s_engage)
                # print("len(res): ", len(res))
                # print([contact[10] for contact in res])
                # print([contact[11] for contact in res])
                # print([contact[12] for contact in res])
                # print([contact[13] for contact in res])

                # Get bodies closest points distance
                dist = p.getClosestPoints(self.gripperUid, self.objectUid, 100)
                dist = np.linalg.norm(np.array(dist[0][5]) - np.array(dist[0][6])) if len(dist)>0 else 0
                
                self.heuristics_traj.append([dist, s_stick, s_engage,])
                new_states = [self.pos_object[0], self.pos_object[1], self.eul_object[2],
                            self.vel_object[0], self.vel_object[1], self.vel_ang_object[2],
                            self.pos_gripper[0], self.pos_gripper[1], self.eul_gripper[2], 
                            self.vel_gripper[0], self.vel_gripper[1], self.vel_ang_gripper[2]
                            ]
                via_points.append(new_states)

            p.stepSimulation()

            # Record cutoff time for the manual scripted movement dataset
            object_reached = (abs(self.pos_object[1]-self.y_obstacle) < 0.2 + 0.01)
            gripper_reached = (abs(self.pos_gripper[1]-self.y_obstacle) < (0.1+0.01))
            if do_cutdown_test and (gripper_reached or object_reached):
                self.cutoff_t = t / 240.0 + 0.2
                return via_points
            if not do_cutdown_test and object_reached:
                self.task_success_label = 1
            if self.gui:
                time.sleep(2/240)
        return via_points


class scriptedMovementSimBalanceGrasp(forwardSimulationBalanceGrasp):
    def __init__(self, cage, gui=False):
        super().__init__(gui=gui)
        self.set_params(cage.params)
        self.create_shapes()
    
    def sample_init_state(self):
        # init_neutral = [5.0, 4.3, 0.0, 0.0, 0.0, 0, # point gripper with cylinder/box object
                        # 5.0, 4, 0.0, 0.0, 0.0, 0]
        xo = 5
        yo = 5
        vxo = random.uniform(-0.01, 0.01)
        vyo = random.uniform(-0.01, 0.01)
        xg = xo + random.uniform(-0.6, 0.6)
        yg = yo - 0.3
        # thetag = random.uniform(-math.pi/15, math.pi/15)
        vxg = random.uniform(-0.01, 0.01)
        vyg = random.uniform(-0.01, 0.01)
        # omegag = random.uniform(-0.01, 0.01)
        init_state = [xo, yo, 0, vxo, vyo, 0,
                      xg, yg, 0, vxg, vyg, 0]
        return init_state

    def run_forward_sim(self, total_time=10, num_via_points=20, taulim=1):
        num_steps = int(total_time * 240)  # Number of time steps
        interval = int(num_steps/num_via_points)
        interval = 3 if interval==0 else interval

        # Step the simulation
        via_points = []
        self.heuristics_traj = []
        self.task_success_label = 0
        for t in range(num_steps):
            self.pos_gripper,_ = p.getBasePositionAndOrientation(self.gripperUid)
            p.applyExternalForce(self.gripperUid, -1, # gravity compensation
                                [0, -self.g*self.mass_gripper*math.sin(self.angle_slope), 0], 
                                self.pos_gripper, 
                                p.WORLD_FRAME)
            p.applyExternalForce(self.gripperUid, -1, # gravity compensation
                                [0, np.random.uniform(0,1)*self.mass_gripper*math.sin(self.angle_slope), 0], 
                                self.pos_gripper, 
                                p.WORLD_FRAME)
            p.applyExternalTorque(self.gripperUid, -1, 
                                [0,0,np.random.uniform(-taulim*self.mass_gripper,taulim*self.mass_gripper)],
                                # [0,0,800],
                                p.WORLD_FRAME)
            
            # Print object via-points along the trajectory for visualization
            if (t+1) % interval == 0:
                # Get the object and gripper states
                self.pos_object, self.quat_object = p.getBasePositionAndOrientation(self.objectUid)
                self.eul_object = p.getEulerFromQuaternion(self.quat_object) # rad
                self.vel_object, self.vel_ang_object = p.getBaseVelocity(self.objectUid)
                self.pos_gripper, self.quat_gripper = p.getBasePositionAndOrientation(self.gripperUid)
                self.eul_gripper = p.getEulerFromQuaternion(self.quat_gripper)
                self.vel_gripper,self.vel_ang_gripper = p.getBaseVelocity(self.gripperUid)

                # Get contact forces
                res = p.getContactPoints(self.gripperUid, self.objectUid)
                all_contact_normal_forces = [contact[9] for contact in res]
                contact_normal_force = sum(all_contact_normal_forces) if len(all_contact_normal_forces)>0 else 0.0
                s_engage = contact_normal_force
                contact_friction_force_xy = sum([contact[10] for contact in res]) if len(all_contact_normal_forces)>0 else 0 # friction along z is not considered
                # Sticking quality measure in the paper - Criteria for Maintaining Desired Contacts for Quasi-Static Systems
                s_stick = (self.lateral_friction_coef*contact_normal_force - abs(contact_friction_force_xy)) * math.cos(np.arctan(self.lateral_friction_coef))

                # Get bodies closest points distance
                # com_dist = np.linalg.norm(np.array(self.pos_gripper) - np.array(self.pos_object)) 
                dist = p.getClosestPoints(self.gripperUid, self.objectUid, 100)
                dist = np.linalg.norm(np.array(dist[0][5]) - np.array(dist[0][6])) if len(dist)>0 else 0
                
                self.heuristics_traj.append([dist, s_stick, s_engage,])
                new_states = [self.pos_object[0], self.pos_object[1], self.eul_object[2],
                            self.vel_object[0], self.vel_object[1], self.vel_ang_object[2],
                            self.pos_gripper[0], self.pos_gripper[1], self.eul_gripper[2], 
                            self.vel_gripper[0], self.vel_gripper[1], self.vel_ang_gripper[2]
                            ]
                via_points.append(new_states)

            p.stepSimulation()
            if self.gui:
                time.sleep(2/240)

        # Record cutoff time for the manual scripted movement dataset
        dist = p.getClosestPoints(self.gripperUid, self.objectUid, 100)
        dist = np.linalg.norm(np.array(dist[0][5]) - np.array(dist[0][6])) if len(dist)>0 else 0
        object_balanced = (dist < 1e-1)
        # gripper_reached = (abs(self.pos_gripper[1]-self.y_obstacle) < (0.1+0.01))
        # if do_cutdown_test and (gripper_reached or object_reached):
        #     self.cutoff_t = t / 240.0 + 0.2
        #     return via_points
        # if not do_cutdown_test and object_reached:
        if object_balanced:
            self.task_success_label = 1
        
        return via_points


class scriptedMovementSimWaterSwing(forwardSimulationWaterSwing):
    def __init__(self, cage, gui=False):
        super().__init__(gui=gui)
        p.setGravity(0, 0, self.g)

        self.set_params(cage.params)
        self.create_shapes()

    def run_forward_sim(self, total_time=10, num_via_points=20):
        num_steps = int(total_time * 240)  # Number of time steps
        radius = 3.0  # Radius of the circular path
        velocity = 2 * np.pi * radius/ total_time  # Radians per second for a full circle
        angular_velocity = 2 * np.pi / total_time  # Radians per second for a full circle
        initial_angular_velocity = -2 * np.pi / total_time  # Initial velocity for a full circle
        vel_gripper = [velocity, 0, 0]
        vel_angular_gripper = [0, initial_angular_velocity, 0]
        p.resetBaseVelocity(self.gripperUid, vel_gripper, vel_angular_gripper) # linear and angular vels both in world coordinates

        dt = total_time / num_steps # 1/240

        interval = int(num_steps/num_via_points)
        interval = 3 if interval==0 else interval

        # Step the simulation
        via_points = []
        
        for t in range(num_steps):
            # Calculate the current angular displacement
            theta = angular_velocity * t * dt
            
            # Calculate centripetal acceleration
            centripetal_acceleration = (velocity ** 2) / radius

            # Calculate force components along x and y axes (centripetal force)
            force_x = -centripetal_acceleration * self.mass_gripper * np.sin(theta)
            force_z = centripetal_acceleration*self.mass_gripper*np.cos(theta) + self.mass_gripper*(-self.g)

            # Apply external force
            self.pos_gripper,_ = p.getBasePositionAndOrientation(self.gripperUid)
            self.pos_object,_ = p.getBasePositionAndOrientation(self.objectUid)
            p.applyExternalForce(self.gripperUid, -1, 
                                [force_x, 0, force_z], 
                                self.pos_gripper, 
                                p.WORLD_FRAME)

            p.stepSimulation()

            # Print object via-points along the trajectory for visualization
            if t % interval == 0 or t == int(t*240)-1:
                # Get the object and gripper states
                self.pos_object, self.quat_object = p.getBasePositionAndOrientation(self.objectUid)
                self.eul_object = p.getEulerFromQuaternion(self.quat_object) # rad
                self.vel_object, self.vel_ang_object = p.getBaseVelocity(self.objectUid)
                self.pos_gripper, self.quat_gripper = p.getBasePositionAndOrientation(self.gripperUid)
                self.eul_gripper = p.getEulerFromQuaternion(self.quat_gripper)
                self.vel_gripper,self.vel_ang_gripper = p.getBaseVelocity(self.gripperUid)

                new_states = [self.pos_object[0], self.pos_object[2], correct_euler(self.eul_object)[1],
                            self.vel_object[0], self.vel_object[2], self.vel_ang_object[1],
                            self.pos_gripper[0], self.pos_gripper[2], correct_euler(self.eul_gripper)[1], 
                            self.vel_gripper[0], self.vel_gripper[2], self.vel_ang_gripper[1]
                            ]
                via_points.append(new_states)

            if self.gui:
                time.sleep(2/240)

        return via_points


class scriptedMovementSimBoxPivot(forwardSimulationBoxPivot):
    def __init__(self, cage, gui=False):
        super().__init__(gui=gui)
        # p.setGravity(0, 0, self.g)

        self.set_params(cage.params)
        self.create_shapes()

    def run_forward_sim(self, total_time=4, num_via_points=20):
        num_steps = int(total_time * 240)  # Number of time steps
        interval = int(num_steps/num_via_points)
        interval = 3 if interval==0 else interval

        # Step the simulation
        via_points = []
        for t in range(num_steps):
            p.applyExternalForce(self.gripperUid1, -1, # push spring
                                [2,0,0], # force
                                [0,0,0], 
                                p.LINK_FRAME)
            
            # Get positions of the boxes
            self.pos_gripper1, _ = p.getBasePositionAndOrientation(self.gripperUid1)
            self.pos_gripper2, _ = p.getBasePositionAndOrientation(self.gripperUid2)
            
            # Update the maxForce for the spring constraint
            maxForce = self.k * np.abs(np.linalg.norm(np.array(self.pos_gripper2) - np.array(self.pos_gripper1)) - self.rest_length)
            p.changeConstraint(self.c_spring, maxForce=maxForce)

            p.stepSimulation()

            # Print object via-points along the trajectory for visualization
            if t % interval == 0 or t == int(t*240)-1:
                # Get the object and gripper states
                self.pos_object, self.quat_object = p.getBasePositionAndOrientation(self.objectUid)
                self.eul_object = p.getEulerFromQuaternion(self.quat_object) # rad
                self.vel_object, self.vel_ang_object = p.getBaseVelocity(self.objectUid)
                self.pos_gripper1, _ = p.getBasePositionAndOrientation(self.gripperUid1)
                self.pos_gripper2, _ = p.getBasePositionAndOrientation(self.gripperUid2)
                self.vel_gripper1,_ = p.getBaseVelocity(self.gripperUid1)
                self.vel_gripper2,_ = p.getBaseVelocity(self.gripperUid2)

                new_states = [self.pos_object[0], self.pos_object[2], self.eul_object[1],
                            self.vel_object[0], self.vel_object[2], self.vel_ang_object[1],
                            self.pos_gripper1[0], self.pos_gripper2[0],  
                            self.vel_gripper1[0], self.vel_gripper2[0], 
                            ]
                via_points.append(new_states)

            if self.gui:
                time.sleep(2/240)

        return via_points


class scriptedMovementSimShuffling(forwardSimulationShuffling):
    def __init__(self, cage, gui=False):
        super().__init__(gui=gui)
        # p.setGravity(0, 0, self.g)

        self.set_params(cage.params)
        self.create_shapes()
        self.t_start_side_push = 200

    def run_forward_sim(self, total_time=6, num_via_points=10):
        num_steps = int(total_time * 240)  # Number of time steps
        interval = int(num_steps/num_via_points)
        interval = 3 if interval==0 else interval

        # Step the simulation
        via_points = []
        for t in range(num_steps):
            # Push the card stack from the side
            # linkBasePosition, _ = p.getBasePositionAndOrientation(self.objectUid)
            if t>self.t_start_side_push and t < self.t_start_side_push+1*240:
                p.applyExternalForce(self.objectUid, 1,
                                    [0,-1,0], 
                                    [0,0,0], 
                                    p.LINK_FRAME)

            # Squeezing the card stack
            joint_states = p.getJointStates(self.objectUid, list(range(self.num_joints)))
            joint_angles = [state[0] for state in joint_states]
            upperPosition, _ = p.getBasePositionAndOrientation(self.gripperUid)
            if t < 10*240:
                force = [0,0,-4/(sum(joint_angles)+0.2)] # simple feedback control
                p.applyExternalForce(self.gripperUid, -1,
                                force, 
                                upperPosition, 
                                p.WORLD_FRAME)
                
            # Calculate the torques induced by joint elasticity and external force

            joint_vels = [state[1] for state in joint_states]
            error_angles = [desired - current for desired, current in zip(self.target_positions, joint_angles)]
            error_vels = [desired - current for desired, current in zip(self.target_velocities, joint_vels)]
            target_torques = [kp*e - kd*av for kp, e, kd, av in zip(self.stiffness, error_angles, self.damping, error_vels)]

            # Mimic elastic cards stack behavior
            p.setJointMotorControlArray(bodyUniqueId=self.objectUid,
                                        jointIndices=list(range(self.num_joints)),
                                        controlMode=p.VELOCITY_CONTROL,
                                        targetVelocities=[100,]*self.num_joints,
                                        forces=target_torques,)

            p.stepSimulation()

            # Print object via-points along the trajectory for visualization
            if t % interval == 0 or t == int(t*240)-1:
                pos, self.quat_object = p.getBasePositionAndOrientation(self.objectUid)
                pos_object_GL = [pos[1], pos[2], pos[0]] # y,z,x
                self.eul_object = p.getEulerFromQuaternion(self.quat_object) # rad
                self.vel_object, self.vel_ang_object = p.getBaseVelocity(self.objectUid)
                joint_states = p.getJointStates(self.objectUid, list(range(self.num_joints)))
                self.pos_object_joints = [state[0] for state in joint_states]
                self.vel_object_joints = [state[1] for state in joint_states]
                self.pos_gripper, _ = p.getBasePositionAndOrientation(self.gripperUid)
                self.vel_gripper, _ = p.getBaseVelocity(self.gripperUid)

                new_states = (pos_object_GL + list(self.eul_object) + list(self.vel_object) + list(self.vel_ang_object) + 
                              self.pos_object_joints + self.vel_object_joints + [self.pos_gripper[2], self.vel_gripper[2]]
                              )
                via_points.append(new_states)

            if self.gui:
                time.sleep(2/240)

        return via_points
