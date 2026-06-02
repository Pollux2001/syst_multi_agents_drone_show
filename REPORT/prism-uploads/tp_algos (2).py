#!/usr/bin/python3
'''
    CentraleSupelec TP 2A/3A
    Aarsh THAKKER,2025
    (all variables in SI unit)

###########################################################################################

============================ READ THIS BEFORE STARTING TO CODE ============================

    You ONLY modify the part that is marked << TO BE MODIFIED >> in the functions
    YOU MUST NOT MODIFY THE NAME OF THE FILE OR THE NAME OF THE FUNCTIONS OR THE INPUT/OUTPUT OF THE FUNCTIONS
    FOR THE SUBMISSION, ONLY THE CODE INSIDE THE FUNCTION (MARKED << TO BE MODIFIED >>) WILL BE CONSIDERED FOR EVALUATION
    variables used by the functions of this script
        - robotNo: Current robot number in the fleet of same type of robots
        - robotPose: current position of the robot (x,y,z, ... depending on the robot)
        - nbTB3B: number of total tb3-Burger robots in the fleet (>=0)
        - nbTB3W: number of total tb3-Waffle robots in the fleet (>=0)
        - nbRMTT: number of total dji robomaster TT drones in the fleet (>=0)
        - nbCF2: number of total crazyflie 2 drones in the fleet (>=0)
        - nbRMEP: number of total dji robomaster EP in the fleet (>=0)  
        - nbOBSTACLE: number of total obstacle positions in the environment (>=0)
        
        - tb3B_poses:  size (3 x nbTB3B) 
            eg. of use: for robot number 'robotNo', position of the robot can be obtained by: 
            tb3B_poses[:,robotNo-1]   (indexes in Python start from 0 !)
            tb3B_poses[0,robotNo-1]: x-coordinate of robot position (in m)
            tb3B_poses[1,robotNo-1]: y-coordinate of robot position (in m)
            tb3B_poses[2,robotNo-1]: orientation angle of robot (in rad)
            
        - tb3W_poses:  size (3 x nbTB3W) 
            tb3W_poses[0,robotNo-1]: x-coordinate of robot position (in m)
            tb3W_poses[1,robotNo-1]: y-coordinate of robot position (in m)
            tb3W_poses[2,robotNo-1]: orientation angle of robot (in rad)

        - rmtt_poses:  size (3 x nbRMTT) 
            rmtt_poses[0,robotNo-1]: x-coordinate of robot position (in m)
            rmtt_poses[1,robotNo-1]: y-coordinate of robot position (in m)
            rmtt_poses[2,robotNo-1]: z-coordinate of robot position (in m)
            rmtt_poses[3,robotNo-1]: orientation angle of robot (in rad) (Ask Supervisor if needed)
            
        - cf2_poses:  size (3 x nbCF2) 
            cf2_poses[0,robotNo-1]: x-coordinate of robot position (in m)
            cf2_poses[1,robotNo-1]: y-coordinate of robot position (in m)
            cf2_poses[2,robotNo-1]: z-coordinate of robot position (in m)

        - rmep_poses:  size (3 x nbRMEP) 
            rmep_poses[0,robotNo-1]: x-coordinate of robot position (in m)
            rmep_poses[1,robotNo-1]: y-coordinate of robot position (in m)
            rmep_poses[2,robotNo-1]: orientation angle of robot (in rad)
            rmep_poses[2,robotNo-1]: orientation angle of robot (in rad)

        - obstacle_pose:  size (3 x nbOBSTACLE)  
            obstacle_pose[0,nbOBSTACLE-1]: x-coordinate of center position of obstacle (in m)
            obstacle_pose[1,nbOBSTACLE-1]: y-coordinate of center position of obstacle (in m)
            obstacle_pose[2,nbOBSTACLE-1]: z-coordinate of center position of obstacle (in m)
        
        - obstacle_size: size (3 x nbOBSTACLE)
            obstacle_size[0,nbOBSTACLE-1]: size of the obstacle in x (in m)
            obstacle_size[1,nbOBSTACLE-1]: size of the obstacle in y (in m)
            obstacle_size[2,nbOBSTACLE-1]: size of the obstacle in z (in m)

    In case of doubt related to the robots, this code or may be something else,
    open a discussion at https://tp-cs.talkyard.net/
    Use your own GitHub account or CS email to signup.
###########################################################################################

'''

import random
import numpy as np
import math, time

# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ===================
# all variables declared here will be known by functions below
# use keyword "global" inside a function if the variable needs to be modified by the function



global TAKEOFF_DONE, Time2Takeoff
TAKEOFF_DONE = False
Time2Takeoff = 5 # time to wait before takeoff for the cf2 drone (in seconds)

# ===================================================================================
# Control function for turtlebot3 Burger ground vehicle Unicycle model
# should ONLY return (vx,vy) for the robot command
# max useable numbers of robots = 6 
# ====================================
def tb3B_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
# ====================================

    nbTB3= len(tb3B_poses[0]) # number of total tb3 robots in the use
    nbTB3W = len(tb3W_poses[0]) # number of total tb3W robots in the use
    nbRMTT = len(rmtt_poses[0]) # number of total dji rmtt drones in the use
    nbCF2 = len(cf2_poses[0]) # number of total cf2 drones in the use
    nbRMEP = len(rmep_poses[0]) # number of total dji rmep in the use
    nbOBSTACLE = len(obstacle_pose[0]) # number of total obstacle positions in the environment

    #  --- TO BE MODIFIED --- 
    if robotNo == 1:    
        goal = [2,-2]
    if robotNo == 2:    
        goal = [1,-2]
    if robotNo == 3:
        time.sleep(2)
        goal = [0,-2]  
    vx = 0.2 * (-robotPose[0] + goal[0])
    vy = 0.2 * (-robotPose[1] + goal[1])
    # -----------------------

    return vx,vy
# ====================================        


# ===================================================================================
# Control function for turtlebot3 Waffle ground vehicle Unicycle model
# should ONLY return (vx,vy) for the robot command
# max useable numbers of robots = 2
# ====================================
def tb3W_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
# ====================================

    nbTB3= len(tb3B_poses[0]) # number of total tb3 robots in the use
    nbTB3W = len(tb3W_poses[0]) # number of total tb3W robots in the use
    nbRMTT = len(rmtt_poses[0]) # number of total dji rmtt drones in the use
    nbCF2 = len(cf2_poses[0]) # number of total cf2 drones in the use
    nbRMEP = len(rmep_poses[0]) # number of total dji rmep in the use
    nbOBSTACLE = len(obstacle_pose[0]) # number of total obstacle positions in the environment

    #  --- TO BE MODIFIED --- 
    goal = [-1,1]
    vx = 0.2 * (-robotPose[0] + goal[0])
    vy = 0.2 * (-robotPose[1] + goal[1])
    # -----------------------

    return vx,vy
# ====================================   


# ====================================        
# Control function for dji rmtt drones
# should ONLY return (vx,vy,vz) for the robot command
# max useable numbers of drones = 4
# ====================================
def rmtt_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
# ====================================
    nbTB3= len(tb3B_poses[0]) # number of total tb3 robots in the use
    nbTB3W = len(tb3W_poses[0]) # number of total tb3W robots in the use
    nbRMTT = len(rmtt_poses[0]) # number of total dji rmtt drones in the use
    nbCF2 = len(cf2_poses[0]) # number of total cf2 drones in the use
    nbRMEP = len(rmep_poses[0]) # number of total dji rmep in the use
    nbOBSTACLE = len(obstacle_pose[0]) # number of total obstacle positions in the environment
    led = (0,0,0) # led color (r,g,b) in range [0,255]
    
    #  --- TO BE MODIFIED ---
    vx = 0.0
    vy = 0.0
    vz = 0.0
    trigger_land = False # trigger to land the drone (True/False)
    goal = [-2,1,1]
    ex = goal[0] - robotPose[0]
    ey = goal[1] - robotPose[1]
    ez = goal[2] - robotPose[2]
    if abs(ex) > 0.2 or abs(ey) > 0.2 or abs(ez) > 0.1:
        vx = 0.5 * ex
        vy = 0.5 * ey
        vz = 0.5 * ez
        led = (255,0,0)
    else:
        vx = 0
        vy = 0
        vz = 0
        led = (0,255,0)
        trigger_land = True
    # -----------------------

    return vx,vy,vz,trigger_land,led
# ====================================    

# ====================================
# Control function for Crazyflie 2 drones
# should ONLY return (vx,vy,z_dist) for the robot command
# max useable numbers of drones = 3
# ====================================
def cf2_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
    global TAKEOFF_DONE, Time2Takeoff
    nbTB3= len(tb3B_poses[0]) # number of total tb3 robots in the use
    nbTB3W = len(tb3W_poses[0]) # number of total tb3W robots in the use
    nbRMTT = len(rmtt_poses[0]) # number of total dji rmtt drones in the use
    nbCF2 = len(cf2_poses[0]) # number of total cf2 drones in the use
    nbRMEP = len(rmep_poses[0]) # number of total dji rmep in the use
    nbOBSTACLE = len(obstacle_pose[0]) # number of total obstacle positions in the environment

    #  --- TO BE MODIFIED ---

    # ============================================================
    # Default values of the function's outputs:
    # ============================================================
    vx = 0.0
    vy = 0.0
    z_dist = 1.0
    trigger_takeoff = False
    trigger_land = False
    led = (0, 0, 0)

    # ============================================================
    # Parameters of the show
    # ============================================================
    z_hover = 1.0
    z_takeoff_threshold = 0.9 * z_hover
    # Initial positions
    initial_positions = np.array([
        [1.0, 0.0, 0.0],
        [-0.5, math.sqrt(3) / 2, 0.0],
        [-0.5, -math.sqrt(3) / 2, 0.0]
    ])

    # Circle used in phase 1
    circle_center = np.array([0.0, 0.0])
    circle_radius = 1.0

    # Durations of the show phases
    rotation_duration = 12.0
    consensus_duration_max = 10.0
    inverse_consensus_duration = 6.0
    formation_no_rotation_duration = 14.0
    formation_rotation_duration = 14.0
    return_duration_max = 12.0

    # Barycenter trajectory used in phases 4 and 5
    barycenter_radius = 0.55
    barycenter_omega = 2.0 * math.pi / formation_no_rotation_duration

    # PD gains
    kp_xy = 0.65
    kd_xy = 0.16

    kp_z = 0.8
    kd_z = 0.2

    # Low-pass filter time constant for the derivative term
    derivative_filter_tau = 0.35

    # Saturations
    max_vxy_cmd = 0.6
    max_vz_cmd = 0.8

    # Tolerances
    return_tolerance = 0.12
    consensus_distance_threshold = 0.85
    inverse_consensus_distance_threshold = 1.35

    # ============================================================
    # Persistent variables
    # ============================================================

    if not hasattr(cf2_control_fn, "initialized"):
        cf2_control_fn.initialized = True

        # Individual state of each drone:
        # 0 = waiting/takeoff
        # 1 = flying show
        # 2 = landing
        # 3 = finished
        cf2_control_fn.mission_state = {}
        for k in range(1, nbCF2 + 1):
            cf2_control_fn.mission_state[k] = 0

        # Common show phase:
        # 0 = takeoff
        # 1 = rotation
        # 2 = partial consensus
        # 3 = inverse consensus
        # 4 = triangle formation without rotation
        # 5 = triangle formation with rotation
        # 6 = return to initial positions
        # 7 = landing
        # 8 = finished
        cf2_control_fn.show_phase = 0
        cf2_control_fn.phase_start_clock = clock

        # For the filtered derivative in the PD control
        cf2_control_fn.previous_error = {}
        cf2_control_fn.filtered_derivative = {}
        cf2_control_fn.previous_clock = {}
        cf2_control_fn.last_phase_seen = {}

        for k in range(1, nbCF2 + 1):
            cf2_control_fn.previous_error[k] = np.zeros(3)
            cf2_control_fn.filtered_derivative[k] = np.zeros(3)
            cf2_control_fn.previous_clock[k] = clock
            cf2_control_fn.last_phase_seen[k] = 0

    # Current indices
    i = robotNo - 1

    # ========================================================
    # Helper functions
    # ========================================================

    def change_phase(new_phase):
        cf2_control_fn.show_phase = new_phase
        cf2_control_fn.phase_start_clock = clock

    def saturate(value, limit):
        if value > limit:
            return limit
        if value < -limit:
            return -limit
        return value

    def pd_position_control(target_position):
        """
        PD controller with low-pass filtered derivative.
        Outputs vx, vy, and an altitude command z_dist.
        In this simulator, z_dist is interpreted as a target altitude,
        so we use z_dist = current_z + vz_cmd to emulate a vertical speed command.
        """
        current_position = np.array([robotPose[0], robotPose[1], robotPose[2]])
        error = target_position - current_position

        dt_control = clock - cf2_control_fn.previous_clock[robotNo]

        # Reset derivative memory at each phase change to avoid derivative kick
        if cf2_control_fn.last_phase_seen[robotNo] != cf2_control_fn.show_phase:
            cf2_control_fn.previous_error[robotNo] = error.copy()
            cf2_control_fn.filtered_derivative[robotNo] = np.zeros(3)
            cf2_control_fn.last_phase_seen[robotNo] = cf2_control_fn.show_phase

        raw_derivative = (error - cf2_control_fn.previous_error[robotNo]) / dt_control

        # First-order low-pass filter:
        # d_filtered[k] = alpha*d_filtered[k-1] + (1-alpha)*d_raw[k]
        alpha = derivative_filter_tau / (derivative_filter_tau + dt_control)
        d_filtered = (
            alpha * cf2_control_fn.filtered_derivative[robotNo]
            + (1.0 - alpha) * raw_derivative
        )

        cf2_control_fn.previous_error[robotNo] = error.copy()
        cf2_control_fn.filtered_derivative[robotNo] = d_filtered.copy()
        cf2_control_fn.previous_clock[robotNo] = clock

        vx_cmd = kp_xy * error[0] + kd_xy * d_filtered[0]
        vy_cmd = kp_xy * error[1] + kd_xy * d_filtered[1]
        vz_cmd = kp_z * error[2] + kd_z * d_filtered[2]

        vx_cmd = saturate(vx_cmd, max_vxy_cmd)
        vy_cmd = saturate(vy_cmd, max_vxy_cmd)
        vz_cmd = saturate(vz_cmd, max_vz_cmd)

        z_cmd = robotPose[2] + vz_cmd
        z_cmd = max(0.0, min(2.0, z_cmd))

        return vx_cmd, vy_cmd, z_cmd

    def max_distance_between_cf2():
        max_dist = 0.0
        for a in range(nbCF2):
            for b in range(a + 1, nbCF2):
                dx = cf2_poses[0, a] - cf2_poses[0, b]
                dy = cf2_poses[1, a] - cf2_poses[1, b]
                dz = cf2_poses[2, a] - cf2_poses[2, b]
                dist = math.sqrt(dx**2 + dy**2 + dz**2)
                if dist > max_dist:
                    max_dist = dist
        return max_dist
    
    def min_distance_between_cf2():
        min_dist = math.inf
        for a in range(nbCF2):
            for b in range(a + 1, nbCF2):
                dx = cf2_poses[0, a] - cf2_poses[0, b]
                dy = cf2_poses[1, a] - cf2_poses[1, b]
                dz = cf2_poses[2, a] - cf2_poses[2, b]
                dist = math.sqrt(dx**2 + dy**2 + dz**2)
                if dist < min_dist:
                    min_dist = dist
        return min_dist

    def all_drones_above_takeoff_threshold():
        for a in range(nbCF2):
            if cf2_poses[2, a] < z_takeoff_threshold:
                return False
        return True

    def all_drones_close_to_initial_positions():
        for a in range(3):
            dx = cf2_poses[0, a] - initial_positions[a, 0]
            dy = cf2_poses[1, a] - initial_positions[a, 1]
            dz = cf2_poses[2, a] - z_hover
            dist = math.sqrt(dx**2 + dy**2 + dz**2)
            if dist > return_tolerance:
                return False
        return True

    # ========================================================
    # Phase transitions
    # ========================================================

    phase = cf2_control_fn.show_phase
    phase_time = clock - cf2_control_fn.phase_start_clock

    if phase == 0:
        if all_drones_above_takeoff_threshold():
            change_phase(1)

    elif phase == 1:
        if phase_time >= rotation_duration:
            change_phase(2)

    elif phase == 2:
        if max_distance_between_cf2() < consensus_distance_threshold:
            change_phase(3)
        elif phase_time >= consensus_duration_max:
            change_phase(3)

    elif phase == 3:
        if min_distance_between_cf2() > inverse_consensus_distance_threshold:
            change_phase(4)
        elif phase_time >= inverse_consensus_duration:
            change_phase(4)

    elif phase == 4:
        if phase_time >= formation_no_rotation_duration:
            change_phase(5)

    elif phase == 5:
        if phase_time >= formation_rotation_duration:
            change_phase(6)

    elif phase == 6:
        if all_drones_close_to_initial_positions():
            change_phase(7)
        elif phase_time >= return_duration_max:
            change_phase(7)

    # Refresh local phase variables after possible transition
    phase = cf2_control_fn.show_phase
    phase_time = clock - cf2_control_fn.phase_start_clock

    # ========================================================
    # Phase 0: takeoff
    # ========================================================

    if phase == 0:
        z_dist = z_hover
        led = (0, 0, 255)

        if cf2_control_fn.mission_state[robotNo] == 0:
            trigger_takeoff = True

            if robotPose[2] > z_takeoff_threshold:
                cf2_control_fn.mission_state[robotNo] = 1

        vx = 0.0
        vy = 0.0

    # ========================================================
    # Phase 1: rotation on a circle
    # ========================================================

    elif phase == 1:
        cf2_control_fn.mission_state[robotNo] = 1

        initial_angle = math.atan2(initial_positions[i, 1], initial_positions[i, 0])
        angle = initial_angle + 2.0 * math.pi * phase_time / rotation_duration

        target_xy = circle_center + circle_radius * np.array([
            math.cos(angle),
            math.sin(angle)
        ])

        target = np.array([target_xy[0], target_xy[1], z_hover])
        vx, vy, z_dist = pd_position_control(target)
        led = (255, 120, 0)

    # ========================================================
    # Phase 2: partial consensus
    # drone i knows drone i+1, not drone i-1
    # ========================================================

    elif phase == 2:
        next_index = (i + 1) % 3

        target = np.array([
            cf2_poses[0, next_index],
            cf2_poses[1, next_index],
            z_hover
        ])

        vx, vy, z_dist = pd_position_control(target)
        led = (255, 0, 255)

    # ========================================================
    # Phase 3: inverse partial consensus
    # same relation, opposite sign
    # ========================================================

    elif phase == 3:
        next_index = (i + 1) % 3

        dx = robotPose[0] - cf2_poses[0, next_index]
        dy = robotPose[1] - cf2_poses[1, next_index]

        target = np.array([
            robotPose[0] + dx,
            robotPose[1] + dy,
            z_hover
        ])

        vx, vy, z_dist = pd_position_control(target)
        led = (255, 255, 0)

    # ========================================================
    # Phase 4: triangle formation without rotation
    # The barycenter follows a circle, the triangle orientation is fixed.
    # ========================================================

    elif phase == 4:
        angle = barycenter_omega * phase_time

        barycenter = np.array([
            barycenter_radius * math.cos(angle),
            barycenter_radius * math.sin(angle)
        ])

        fixed_offset = np.array([
            initial_positions[i, 0],
            initial_positions[i, 1]
        ])

        target_xy = barycenter + fixed_offset
        target = np.array([target_xy[0], target_xy[1], z_hover])

        vx, vy, z_dist = pd_position_control(target)
        led = (0, 255, 255)

    # ========================================================
    # Phase 5: triangle formation with rotation
    # The barycenter follows a circle, and the formation rotates.
    # ========================================================

    elif phase == 5:
        angle_barycenter = barycenter_omega * phase_time

        barycenter = np.array([
            barycenter_radius * math.cos(angle_barycenter),
            barycenter_radius * math.sin(angle_barycenter)
        ])

        formation_angle = 2.0 * math.pi * phase_time / formation_rotation_duration

        c = math.cos(formation_angle)
        s = math.sin(formation_angle)

        offset = np.array([
            initial_positions[i, 0],
            initial_positions[i, 1]
        ])

        rotated_offset = np.array([
            c * offset[0] - s * offset[1],
            s * offset[0] + c * offset[1]
        ])

        target_xy = barycenter + rotated_offset
        target = np.array([target_xy[0], target_xy[1], z_hover])

        vx, vy, z_dist = pd_position_control(target)
        led = (0, 180, 255)

    # ========================================================
    # Phase 6: return to initial positions
    # ========================================================

    elif phase == 6:
        target = np.array([
            initial_positions[i, 0],
            initial_positions[i, 1],
            z_hover
        ])

        vx, vy, z_dist = pd_position_control(target)
        led = (120, 255, 120)

    # ========================================================
    # Phase 7: landing
    # ========================================================

    elif phase == 7:
        vx = 0.0
        vy = 0.0
        z_dist = 0.0
        trigger_land = True
        led = (0, 255, 0)

        cf2_control_fn.mission_state[robotNo] = 2

        if robotPose[2] < 0.15:
            cf2_control_fn.mission_state[robotNo] = 3

    # ========================================================
    # Phase 8 or unknown: finished / safety stop
    # ========================================================

    else:
        vx = 0.0
        vy = 0.0
        z_dist = 0.0
        trigger_takeoff = False
        trigger_land = False
        led = (0, 255, 0)
        cf2_control_fn.mission_state[robotNo] = 3

    # -----------------------

    return vx, vy, z_dist, trigger_takeoff, trigger_land, led

# ====================================
# (Ask Supervisor if you need to use these robots)
# Control function for dji rmep robots (omnidirectional robots with gripper)
# should ONLY return (vx,vy,wz) for the robot command
# max useable numbers of robots = 2
# ====================================
def rmep_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
# ====================================
    nbTB3= len(tb3B_poses[0]) # number of total tb3 robots in the use
    nbTB3W = len(tb3W_poses[0]) # number of total tb3W robots in the use
    nbRMTT = len(rmtt_poses[0]) # number of total dji rmtt drones in the use
    nbCF2 = len(cf2_poses[0]) # number of total cf2 drones in the use
    nbRMEP = len(rmep_poses[0]) # number of total dji rmep in the use
    nbOBSTACLE = len(obstacle_pose[0]) # number of total obstacle positions in the environment

    #  --- TO BE MODIFIED ---

    vx = 0.0
    vy = 0.0
    wz = 0.0
    goal = [1.5,1,1.57]
    ex = goal[0] - robotPose[0]
    ey = goal[1] - robotPose[1]
    etheta = goal[2] - robotPose[2]
    # try to avoid using the wz if possible, not reliable 
    
    if abs(ex) > 0.1 or abs(ey) > 0.1 or abs(etheta) > 0.1:
        vx = 0.3 * ex
        vy = 0.3 * ey 
        wz = 0.1 * etheta
    else:
        vx = 0.0
        vy = 0.0
        wz = 0.0
    # -----------------------

    return vx, vy, wz
# ====================================





# ======== ! DO NOT MODIFY ! ============
def tb3B_controller(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
    vx,vy = tb3B_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock)
    return vx,vy
def tb3W_controller(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
    vx,vy = tb3W_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock)
    return vx,vy
def rmtt_controller(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
    vx, vy, vz, trigger_land, led = rmtt_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock)
    return vx, vy, vz, trigger_land, led
def cf2_controller(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
    vx, vy, z_dist, trigger_takeoff, trigger_land, led = cf2_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock)
    return vx, vy, z_dist, trigger_takeoff, trigger_land, led
def rmep_controller(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
    vx,vy,wz = rmep_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock)
    return vx,vy,wz
# =======================================
