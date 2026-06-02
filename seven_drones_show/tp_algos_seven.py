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
    z_dist = 0.0
    trigger_takeoff = False
    trigger_land = False
    led = (0, 0, 0)

    # ============================================================
    # Parameters of the show
    # ============================================================
    z_hover = 1.5
    z_takeoff_threshold = 0.9 * z_hover
    z_landed_threshold = 0.15
    takeoff_duration_max = 10.0
    hexagon_rotation_duration = 12.0
    hexagon_radius = 1.0
    consensus_min_distance_threshold = 0.5
    inverse_consensus_min_distance_threshold = 1.0
    pair_setup_duration = 4.0
    pair_rotation_duration = 12.0
    pair_half_distance = 0.35
    altitude_separation_duration = 5.0
    vertical_line_duration = 7.0
    vertical_line_heights = np.array([0.45, 0.85, 1.25, 1.65, 2.05, 2.45, 2.85])
    serpentine_duration = 10.0
    serpentine_amplitude = 0.75
    serpentine_oscillation_period = 5.0
    vertical_line_return_duration = 4.0
    vertical_hexagon_setup_duration = 12.0
    vertical_hexagon_rotation_duration = 12.0
    vertical_hexagon_center_z = 1.65
    vertical_hexagon_radius = 0.75
    double_triangle_setup_duration = 12.0
    double_triangle_rotation_duration = 12.0
    triangle_dephase_duration = 8.0
    triangle_radius = 0.75
    lower_triangle_z = 1.1
    middle_drone_z = 1.65
    upper_triangle_z = 2.2
    triangle_dephase_offset = math.pi / 3.0
    transition_lane_x = np.array([0.0, -1.2, -0.8, -0.4, 0.4, 0.8, 1.2])

    # XY PD controller parameters.
    # z_dist must stay a direct altitude setpoint chosen by each phase.
    kp_xy = 0.65
    kd_xy = 0.16
    derivative_filter_tau = 0.35
    max_vxy_cmd = 0.6
    guardrail_radius = 0.6
    guardrail_gain = 0.55

    # Harmonized LED palette, one mood per phase family.
    led_takeoff = (40, 255, 120)
    led_orbit = (70, 140, 255)
    led_consensus = (180, 80, 255)
    led_expansion = (255, 190, 70)
    led_pairs = (0, 220, 220)
    led_vertical = (90, 255, 180)
    led_serpentine = (210, 90, 255)
    led_hexagon = (90, 170, 255)
    led_rotation = (255, 140, 60)
    led_triangles = (255, 90, 170)
    led_prelanding = (160, 255, 120)
    led_landing = (40, 255, 120)
    led_finished = (255, 60, 60)

    # ============================================================
    # Persistent variables
    # ============================================================
    if not hasattr(cf2_control_fn, "initialized"):
        cf2_control_fn.initialized = True

        # Individual state of each drone:
        # 0 = waiting/takeoff
        # 1 = flying
        # 2 = landing
        # 3 = finished
        cf2_control_fn.mission_state = {}
        for k in range(1, nbCF2 + 1):
            cf2_control_fn.mission_state[k] = 0

        # Common show phase:
        # 0 = takeoff
        # 1 = horizontal hexagon rotation
        # 2 = consensus
        # 3 = inverse consensus
        # 4 = pair setup
        # 5 = pair rotation
        # 6 = altitude separation
        # 7 = vertical line formation
        # 8 = serpentine
        # 9 = vertical line return
        # 10 = vertical hexagon formation
        # 11 = vertical hexagon rotation around Oz
        # 12 = double triangle formation
        # 13 = counter-rotating triangles
        # 14 = triangle dephasing before landing
        # 15 = landing
        # 16 = finished
        cf2_control_fn.show_phase = 0
        cf2_control_fn.phase_start_clock = clock
        cf2_control_fn.pair_rotation_initialized = False
        cf2_control_fn.pair_initial_center_angles = {}
        cf2_control_fn.pair_initial_center_radii = {}
        cf2_control_fn.pair_initial_local_angles = {}

        # Memory for the XY-only PD controller.
        cf2_control_fn.previous_xy_error = {}
        cf2_control_fn.filtered_xy_derivative = {}
        cf2_control_fn.previous_clock = {}
        cf2_control_fn.last_phase_seen = {}

        for k in range(1, nbCF2 + 1):
            cf2_control_fn.previous_xy_error[k] = np.zeros(2)
            cf2_control_fn.filtered_xy_derivative[k] = np.zeros(2)
            cf2_control_fn.previous_clock[k] = clock
            cf2_control_fn.last_phase_seen[k] = 0

    # ========================================================
    # Helper functions
    # ========================================================
    def change_phase(new_phase):
        cf2_control_fn.show_phase = new_phase
        cf2_control_fn.phase_start_clock = clock
        if new_phase == 5:
            initialize_pair_rotation()

    def saturate(value, limit):
        if value > limit:
            return limit
        if value < -limit:
            return -limit
        return value

    def pd_xy_control(target_xy):
        current_xy = np.array([robotPose[0], robotPose[1]])
        target_xy = np.array(target_xy)
        error = target_xy - current_xy

        dt_control = clock - cf2_control_fn.previous_clock[robotNo]
        if dt_control <= 1e-6:
            dt_control = 1e-6

        # Reset derivative memory at each phase change to avoid derivative kick.
        if cf2_control_fn.last_phase_seen[robotNo] != cf2_control_fn.show_phase:
            cf2_control_fn.previous_xy_error[robotNo] = error.copy()
            cf2_control_fn.filtered_xy_derivative[robotNo] = np.zeros(2)
            cf2_control_fn.last_phase_seen[robotNo] = cf2_control_fn.show_phase

        raw_derivative = (error - cf2_control_fn.previous_xy_error[robotNo]) / dt_control
        alpha = derivative_filter_tau / (derivative_filter_tau + dt_control)
        filtered_derivative = (
            alpha * cf2_control_fn.filtered_xy_derivative[robotNo]
            + (1.0 - alpha) * raw_derivative
        )

        cf2_control_fn.previous_xy_error[robotNo] = error.copy()
        cf2_control_fn.filtered_xy_derivative[robotNo] = filtered_derivative.copy()
        cf2_control_fn.previous_clock[robotNo] = clock

        vx_cmd = kp_xy * error[0] + kd_xy * filtered_derivative[0]
        vy_cmd = kp_xy * error[1] + kd_xy * filtered_derivative[1]

        vx_cmd = saturate(vx_cmd, max_vxy_cmd)
        vy_cmd = saturate(vy_cmd, max_vxy_cmd)

        return vx_cmd, vy_cmd

    def apply_separation_guardrail(vx_cmd, vy_cmd):
        current_position = np.array([robotPose[0], robotPose[1], robotPose[2]])
        correction = np.zeros(2)

        for other_index in range(nbCF2):
            if other_index == robotNo - 1:
                continue

            other_position = cf2_poses[:, other_index]
            delta = current_position - other_position
            distance = np.linalg.norm(delta)

            if distance < 1e-6:
                angle = 2.0 * math.pi * (robotNo - 1) / max(nbCF2, 1)
                delta = np.array([math.cos(angle), math.sin(angle), 0.0])
                distance = 1.0

            if distance < guardrail_radius:
                delta_xy = delta[0:2]
                xy_distance = np.linalg.norm(delta_xy)
                if xy_distance < 1e-6:
                    angle = 2.0 * math.pi * (robotNo - 1) / max(nbCF2, 1)
                    direction_xy = np.array([math.cos(angle), math.sin(angle)])
                else:
                    direction_xy = delta_xy / xy_distance

                proximity = (guardrail_radius - distance) / guardrail_radius
                correction += guardrail_gain * proximity * proximity * direction_xy

        vx_cmd = saturate(vx_cmd + correction[0], max_vxy_cmd)
        vy_cmd = saturate(vy_cmd + correction[1], max_vxy_cmd)

        return vx_cmd, vy_cmd

    def all_drones_above_takeoff_threshold():
        for a in range(nbCF2):
            if cf2_poses[2, a] < z_takeoff_threshold:
                return False
        return True

    def all_drones_landed():
        for a in range(nbCF2):
            if cf2_poses[2, a] > z_landed_threshold:
                return False
        return True

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

    def pair_indices(pair_id):
        first_index = 1 + 2 * pair_id
        second_index = first_index + 1
        return first_index, second_index

    def pair_setup_target_xy(pair_id, member_id):
        first_index, second_index = pair_indices(pair_id)
        first_xy = cf2_poses[0:2, first_index]
        second_xy = cf2_poses[0:2, second_index]
        center_xy = 0.5 * (first_xy + second_xy)
        pair_axis = first_xy - second_xy
        pair_distance = np.linalg.norm(pair_axis)

        if pair_distance < 1e-6:
            angle = math.pi / 6.0 + 2.0 * math.pi * pair_id / 3.0
            pair_axis = np.array([math.cos(angle), math.sin(angle)])
        else:
            pair_axis = pair_axis / pair_distance

        local_sign = 1.0 if member_id == 0 else -1.0
        return center_xy + local_sign * pair_half_distance * pair_axis

    def initialize_pair_rotation():
        pair_initial_center_angles = {}
        pair_initial_center_radii = {}
        pair_initial_local_angles = {}

        for pair_id in range(3):
            first_index, second_index = pair_indices(pair_id)
            first_xy = cf2_poses[0:2, first_index]
            second_xy = cf2_poses[0:2, second_index]
            center_xy = 0.5 * (first_xy + second_xy)
            local_axis = first_xy - center_xy

            center_radius = np.linalg.norm(center_xy)
            if center_radius < 1e-6:
                center_angle = math.pi / 6.0 + 2.0 * math.pi * pair_id / 3.0
                center_radius = 1.0
            else:
                center_angle = math.atan2(center_xy[1], center_xy[0])

            if np.linalg.norm(local_axis) < 1e-6:
                local_angle = center_angle + math.pi / 2.0
            else:
                local_angle = math.atan2(local_axis[1], local_axis[0])

            pair_initial_center_angles[pair_id] = center_angle
            pair_initial_center_radii[pair_id] = center_radius
            pair_initial_local_angles[pair_id] = local_angle

        cf2_control_fn.pair_initial_center_angles = pair_initial_center_angles
        cf2_control_fn.pair_initial_center_radii = pair_initial_center_radii
        cf2_control_fn.pair_initial_local_angles = pair_initial_local_angles
        cf2_control_fn.pair_rotation_initialized = True

    def vertical_hexagon_target(rotation_angle):
        if robotNo == 1:
            return [0.0, 0.0], vertical_hexagon_center_z

        hexagon_angle = 2.0 * math.pi * (robotNo - 2) / 6.0
        radial_offset = vertical_hexagon_radius * math.cos(hexagon_angle)
        target_z = vertical_hexagon_center_z + vertical_hexagon_radius * math.sin(hexagon_angle)

        target_xy = [
            -radial_offset * math.sin(rotation_angle),
            radial_offset * math.cos(rotation_angle)
        ]

        return target_xy, target_z

    def double_triangle_target(rotation_angle):
        if robotNo == 1:
            return [0.0, 0.0], middle_drone_z

        if robotNo <= 4:
            triangle_member = robotNo - 2
            triangle_z = lower_triangle_z
            triangle_rotation = rotation_angle
        else:
            triangle_member = robotNo - 5
            triangle_z = upper_triangle_z
            triangle_rotation = -rotation_angle

        angle = 2.0 * math.pi * triangle_member / 3.0 + triangle_rotation
        target_xy = [
            triangle_radius * math.cos(angle),
            triangle_radius * math.sin(angle)
        ]

        return target_xy, triangle_z

    def dephased_triangle_target():
        if robotNo == 1:
            return [0.0, 0.0], middle_drone_z

        if robotNo <= 4:
            triangle_member = robotNo - 2
            triangle_z = lower_triangle_z
            angle_offset = 0.0
        else:
            triangle_member = robotNo - 5
            triangle_z = upper_triangle_z
            angle_offset = triangle_dephase_offset

        angle = 2.0 * math.pi * triangle_member / 3.0 + angle_offset
        target_xy = [
            triangle_radius * math.cos(angle),
            triangle_radius * math.sin(angle)
        ]

        return target_xy, triangle_z

    # ========================================================
    # Phase transitions
    # ========================================================
    phase = cf2_control_fn.show_phase
    phase_time = clock - cf2_control_fn.phase_start_clock

    if phase == 0:
        if all_drones_above_takeoff_threshold():
            change_phase(1)
        elif phase_time >= takeoff_duration_max:
            change_phase(15)

    elif phase == 1:
        if phase_time >= hexagon_rotation_duration:
            change_phase(2)

    elif phase == 2:
        if min_distance_between_cf2() <= consensus_min_distance_threshold:
            change_phase(3)

    elif phase == 3:
        if min_distance_between_cf2() >= inverse_consensus_min_distance_threshold:
            change_phase(4)

    elif phase == 4:
        if phase_time >= pair_setup_duration:
            change_phase(5)

    elif phase == 5:
        if phase_time >= pair_rotation_duration:
            change_phase(6)

    elif phase == 6:
        if phase_time >= altitude_separation_duration:
            change_phase(7)

    elif phase == 7:
        if phase_time >= vertical_line_duration:
            change_phase(8)

    elif phase == 8:
        if phase_time >= serpentine_duration:
            change_phase(9)

    elif phase == 9:
        if phase_time >= vertical_line_return_duration:
            change_phase(10)

    elif phase == 10:
        if phase_time >= vertical_hexagon_setup_duration:
            change_phase(11)

    elif phase == 11:
        if phase_time >= vertical_hexagon_rotation_duration:
            change_phase(12)

    elif phase == 12:
        if phase_time >= double_triangle_setup_duration:
            change_phase(13)

    elif phase == 13:
        if phase_time >= double_triangle_rotation_duration:
            change_phase(14)

    elif phase == 14:
        if phase_time >= triangle_dephase_duration:
            change_phase(15)

    elif phase == 15:
        if all_drones_landed():
            change_phase(16)

    phase = cf2_control_fn.show_phase
    phase_time = clock - cf2_control_fn.phase_start_clock

    # ========================================================
    # Phase 0: takeoff
    # ========================================================
    if phase == 0:
        z_dist = z_hover
        led = led_takeoff

        if cf2_control_fn.mission_state[robotNo] == 0:
            trigger_takeoff = True

            if robotPose[2] > z_takeoff_threshold:
                cf2_control_fn.mission_state[robotNo] = 1

        vx = 0.0
        vy = 0.0

    # ========================================================
    # Phase 1: horizontal hexagon rotation
    # ========================================================
    elif phase == 1:
        z_dist = z_hover
        led = led_orbit
        cf2_control_fn.mission_state[robotNo] = 1

        if robotNo == 1:
            target_xy = [0.0, 0.0]
        else:
            initial_angle = 2.0 * math.pi * (robotNo - 2) / 6.0
            angle = initial_angle + 2.0 * math.pi * phase_time / hexagon_rotation_duration
            target_xy = [
                hexagon_radius * math.cos(angle),
                hexagon_radius * math.sin(angle)
            ]

        vx, vy = pd_xy_control(target_xy)
        vx, vy = apply_separation_guardrail(vx, vy)

    # ========================================================
    # Phase 2: consensus
    # ========================================================
    elif phase == 2:
        z_dist = z_hover
        led = led_consensus
        cf2_control_fn.mission_state[robotNo] = 1

        consensus_target_xy = [
            np.mean(cf2_poses[0, :]),
            np.mean(cf2_poses[1, :])
        ]
        vx, vy = pd_xy_control(consensus_target_xy)

    # ========================================================
    # Phase 3: inverse consensus
    # ========================================================
    elif phase == 3:
        z_dist = z_hover
        led = led_expansion
        cf2_control_fn.mission_state[robotNo] = 1

        barycenter_xy = np.array([
            np.mean(cf2_poses[0, :]),
            np.mean(cf2_poses[1, :])
        ])
        current_xy = np.array([robotPose[0], robotPose[1]])
        delta_from_barycenter = current_xy - barycenter_xy
        target_xy = current_xy + delta_from_barycenter

        vx, vy = pd_xy_control(target_xy)
        vx, vy = apply_separation_guardrail(vx, vy)

    # ========================================================
    # Phase 4: pair setup
    # ========================================================
    elif phase == 4:
        z_dist = z_hover
        led = led_pairs
        cf2_control_fn.mission_state[robotNo] = 1

        if robotNo == 1:
            target_xy = [0.0, 0.0]
        else:
            pair_id = (robotNo - 2) // 2
            member_id = (robotNo - 2) % 2
            target_xy = pair_setup_target_xy(pair_id, member_id)

        vx, vy = pd_xy_control(target_xy)
        vx, vy = apply_separation_guardrail(vx, vy)

    # ========================================================
    # Phase 5: pair rotation
    # ========================================================
    elif phase == 5:
        z_dist = z_hover
        led = led_pairs
        cf2_control_fn.mission_state[robotNo] = 1

        if not cf2_control_fn.pair_rotation_initialized:
            initialize_pair_rotation()

        if robotNo == 1:
            target_xy = [0.0, 0.0]
        else:
            pair_id = (robotNo - 2) // 2
            member_id = (robotNo - 2) % 2
            local_sign = 1.0 if member_id == 0 else -1.0

            if pair_id not in cf2_control_fn.pair_initial_center_angles:
                initialize_pair_rotation()

            rotation_progress = phase_time / pair_rotation_duration
            orbit_angle = (
                cf2_control_fn.pair_initial_center_angles[pair_id]
                + 2.0 * math.pi * rotation_progress
            )
            center_radius = cf2_control_fn.pair_initial_center_radii[pair_id]
            pair_center_xy = np.array([
                center_radius * math.cos(orbit_angle),
                center_radius * math.sin(orbit_angle)
            ])

            self_angle = (
                cf2_control_fn.pair_initial_local_angles[pair_id]
                + 4.0 * math.pi * rotation_progress
            )
            local_offset = local_sign * pair_half_distance * np.array([
                math.cos(self_angle),
                math.sin(self_angle)
            ])

            target_xy = pair_center_xy + local_offset

        vx, vy = pd_xy_control(target_xy)
        vx, vy = apply_separation_guardrail(vx, vy)

    # ========================================================
    # Phase 6: altitude separation
    # ========================================================
    elif phase == 6:
        z_dist = vertical_line_heights[robotNo - 1]
        led = led_vertical
        cf2_control_fn.mission_state[robotNo] = 1
        vx = 0.0
        vy = 0.0

    # ========================================================
    # Phase 7: vertical line formation
    # ========================================================
    elif phase == 7:
        z_dist = vertical_line_heights[robotNo - 1]
        led = led_vertical
        cf2_control_fn.mission_state[robotNo] = 1

        target_xy = [0.0, 0.0]
        vx, vy = pd_xy_control(target_xy)
        vx, vy = apply_separation_guardrail(vx, vy)

    # ========================================================
    # Phase 8: serpentine in the Oyz plane
    # ========================================================
    elif phase == 8:
        z_dist = vertical_line_heights[robotNo - 1]
        led = led_serpentine
        cf2_control_fn.mission_state[robotNo] = 1

        serpentine_phase = (
            (robotNo - 1) * math.pi / 2.0
            + 2.0 * math.pi * phase_time / serpentine_oscillation_period
        )
        target_xy = [
            0.0,
            serpentine_amplitude * math.sin(serpentine_phase)
        ]

        vx, vy = pd_xy_control(target_xy)
        vx, vy = apply_separation_guardrail(vx, vy)

    # ========================================================
    # Phase 9: vertical line return
    # ========================================================
    elif phase == 9:
        z_dist = vertical_line_heights[robotNo - 1]
        led = led_vertical
        cf2_control_fn.mission_state[robotNo] = 1

        target_xy = [0.0, 0.0]
        vx, vy = pd_xy_control(target_xy)
        vx, vy = apply_separation_guardrail(vx, vy)

    # ========================================================
    # Phase 10: vertical hexagon formation
    # ========================================================
    elif phase == 10:
        led = led_hexagon
        cf2_control_fn.mission_state[robotNo] = 1

        target_xy, hexagon_z = vertical_hexagon_target(0.0)
        lane_x = transition_lane_x[robotNo - 1]

        if phase_time < vertical_hexagon_setup_duration / 3.0:
            z_dist = vertical_line_heights[robotNo - 1]
            target_xy = [lane_x, robotPose[1]]
        elif phase_time < 2.0 * vertical_hexagon_setup_duration / 3.0:
            z_dist = vertical_line_heights[robotNo - 1]
            target_xy = [lane_x, target_xy[1]]
        else:
            z_dist = hexagon_z

        vx, vy = pd_xy_control(target_xy)
        vx, vy = apply_separation_guardrail(vx, vy)

    # ========================================================
    # Phase 11: vertical hexagon rotation around Oz
    # ========================================================
    elif phase == 11:
        led = led_rotation
        cf2_control_fn.mission_state[robotNo] = 1

        rotation_angle = 2.0 * math.pi * phase_time / vertical_hexagon_rotation_duration
        target_xy, z_dist = vertical_hexagon_target(rotation_angle)
        vx, vy = pd_xy_control(target_xy)
        vx, vy = apply_separation_guardrail(vx, vy)

    # ========================================================
    # Phase 12: double triangle formation
    # ========================================================
    elif phase == 12:
        led = led_triangles
        cf2_control_fn.mission_state[robotNo] = 1

        triangle_xy, triangle_z = double_triangle_target(0.0)
        hexagon_xy, hexagon_z = vertical_hexagon_target(0.0)
        lane_x = transition_lane_x[robotNo - 1]

        if phase_time < double_triangle_setup_duration / 3.0:
            target_xy = [lane_x, hexagon_xy[1]]
            z_dist = hexagon_z
        elif phase_time < 2.0 * double_triangle_setup_duration / 3.0:
            target_xy = [lane_x, triangle_xy[1]]
            z_dist = triangle_z
        else:
            target_xy = triangle_xy
            z_dist = triangle_z

        vx, vy = pd_xy_control(target_xy)
        vx, vy = apply_separation_guardrail(vx, vy)

    # ========================================================
    # Phase 13: counter-rotating triangles
    # ========================================================
    elif phase == 13:
        led = led_triangles
        cf2_control_fn.mission_state[robotNo] = 1

        rotation_angle = 2.0 * math.pi * phase_time / double_triangle_rotation_duration
        target_xy, z_dist = double_triangle_target(rotation_angle)
        vx, vy = pd_xy_control(target_xy)
        vx, vy = apply_separation_guardrail(vx, vy)

    # ========================================================
    # Phase 14: triangle dephasing before landing
    # ========================================================
    elif phase == 14:
        led = led_prelanding
        cf2_control_fn.mission_state[robotNo] = 1

        target_xy, z_dist = dephased_triangle_target()
        vx, vy = pd_xy_control(target_xy)
        vx, vy = apply_separation_guardrail(vx, vy)

    # ========================================================
    # Phase 15: landing
    # ========================================================
    elif phase == 15:
        vx = 0.0
        vy = 0.0
        z_dist = 0.0
        trigger_land = True
        led = led_landing

        cf2_control_fn.mission_state[robotNo] = 2

        if robotPose[2] < z_landed_threshold:
            cf2_control_fn.mission_state[robotNo] = 3
            if all_drones_landed():
                change_phase(16)

    # ========================================================
    # Phase 16: finished / safety stop
    # ========================================================
    else:
        vx = 0.0
        vy = 0.0
        z_dist = 0.0
        trigger_takeoff = False
        trigger_land = False
        led = led_finished
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
