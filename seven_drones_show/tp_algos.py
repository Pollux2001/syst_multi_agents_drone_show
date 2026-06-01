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
    z_dist = 1.0
    trigger_takeoff = False
    trigger_land = False
    led = (0, 0, 0)

    # ============================================================
    # Parameters of the show
    # ============================================================
    z_hover = 1.5
    z_takeoff_threshold = 0.9 * z_hover

    sqrt3 = math.sqrt(3) # to reduce position matrixes' sizes

    # Initial positions: regular hexagon + barycenter
    # drone 1 is the center, drones 2 to 7 are on the hexagon
    initial_positions = np.array([
        [ 0.0,        0.0, 0.0],
        [ 1.0,        0.0, 0.0],
        [ 0.5,  sqrt3/2.0, 0.0],
        [-0.5,  sqrt3/2.0, 0.0],
        [-1.0,        0.0, 0.0],
        [-0.5, -sqrt3/2.0, 0.0],
        [ 0.5, -sqrt3/2.0, 0.0]
    ])

    # Phase 1 / 2 vertical circle geometry
    vertical_center = np.array([0.0, 0.0, z_hover])
    vertical_radius = 1.0
    safe_consensus_radius = 0.55
    vertical_positions = np.array([
        [ 0.0, 0.0,             z_hover],
        [ 1.0, 0.0,             z_hover],
        [ 0.5, 0.0, z_hover + sqrt3/2.0],
        [-0.5, 0.0, z_hover + sqrt3/2.0],
        [-1.0, 0.0,             z_hover],
        [-0.5, 0.0, z_hover - sqrt3/2.0],
        [ 0.5, 0.0, z_hover - sqrt3/2.0]
    ])

    # Phase 6: line formation
    line_positions = np.array([
        [-1.5, 0.0, z_hover],
        [-1.0, 0.0, z_hover],
        [-0.5, 0.0, z_hover],
        [ 0.0, 0.0, z_hover],
        [ 0.5, 0.0, z_hover],
        [ 1.0, 0.0, z_hover],
        [ 1.5, 0.0, z_hover]
    ])

    # Intermediate y-offsets used to avoid collisions when going to line / new formation
    lane_offsets = np.array([0.0, 0.45, -0.45, 0.9, -0.9, 1.35, -1.35])

    # Phase 8: new formation
    new_formation_positions = np.array([
        [ 0.0,  1.65, z_hover],
        [-1.1,  0.65,    1.9],
        [-0.35, 0.65,    2.3],
        [ 1.1,  0.65,    1.9],
        [-1.1, -0.65,    1.9],
        [ 0.35,-0.65,    2.3],
        [ 1.1, -0.65,    1.9]
    ])

    # Phase 9: tunnel target for drone 1
    tunnel_target_drone_1 = np.array([0.0, -1.65, z_hover])

    # Durations
    move_to_vertical_duration_max = 10.0
    rotation_duration = 12.0
    consensus_duration_max = 8.0
    inverse_consensus_duration = 6.0
    pair_flight_duration = 14.0
    line_duration = 12.0
    chain_duration = 16.0
    new_formation_duration = 12.0
    tunnel_duration_max = 10.0

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
    consensus_distance_threshold = 0.85
    position_tolerance = 0.10

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
        # 1 = move to vertical formation
        # 2 = rotation around vertical circle
        # 3 = partial consensus
        # 4 = inverse partial consensus
        # 5 = pair flight
        # 6 = line formation
        # 7 = leader/follower chain
        # 8 = new formation
        # 9 = tunnel
        # 10 = landing
        # 11 = finished
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

    def smooth_step(progress):
        progress = max(0.0, min(1.0, progress))
        return progress * progress * (3.0 - 2.0 * progress)

    def vertical_ring_target(radius):
        if robotNo == 1:
            return vertical_center.copy()

        angle = 2.0 * math.pi * (robotNo - 2) / 6.0
        return np.array([
            vertical_center[0] + radius * math.cos(angle),
            0.0,
            vertical_center[2] + radius * math.sin(angle)
        ])

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
        z_cmd = max(0.0, min(2.5, z_cmd))

        return vx_cmd, vy_cmd, z_cmd

    def apply_separation_guardrail(vx_cmd, vy_cmd, z_cmd):
        guard_radius = 0.55
        strength = 0.35
        current_position = np.array([robotPose[0], robotPose[1], robotPose[2]])
        correction = np.zeros(3)

        for other_index in range(nbCF2):
            if other_index == i:
                continue

            other_position = cf2_poses[:, other_index]
            delta = current_position - other_position
            distance = np.linalg.norm(delta)

            if distance < 1e-4:
                angle = 2.0 * math.pi * i / max(nbCF2, 1)
                delta = np.array([math.cos(angle), math.sin(angle), 0.0])
                distance = 1.0

            if distance < guard_radius:
                direction = delta / distance
                proximity = (guard_radius - distance) / guard_radius
                correction += strength * proximity * proximity * direction

        vx_cmd = saturate(vx_cmd + correction[0], max_vxy_cmd)
        vy_cmd = saturate(vy_cmd + correction[1], max_vxy_cmd)
        z_cmd = max(0.0, min(2.5, z_cmd + correction[2]))

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

    def all_drones_close_to_targets(targets, tol):
            for a in range(nbCF2):
                dx = cf2_poses[0, a] - targets[a, 0]
                dy = cf2_poses[1, a] - targets[a, 1]
                dz = cf2_poses[2, a] - targets[a, 2]
                dist = math.sqrt(dx**2 + dy**2 + dz**2)
                if dist > tol:
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
        if all_drones_close_to_targets(vertical_positions, position_tolerance):
            change_phase(2)
        elif phase_time >= move_to_vertical_duration_max:
            change_phase(2)

    elif phase == 2:
        if phase_time >= rotation_duration:
            change_phase(3)

    elif phase == 3:
        if phase_time >= consensus_duration_max:
            change_phase(4)

    elif phase == 4:
        if phase_time >= inverse_consensus_duration:
            change_phase(5)

    elif phase == 5:
        if phase_time >= pair_flight_duration:
            change_phase(6)

    elif phase == 6:
        if all_drones_close_to_targets(line_positions, position_tolerance):
            change_phase(7)
        elif phase_time >= line_duration:
            change_phase(7)

    elif phase == 7:
        if phase_time >= chain_duration:
            change_phase(8)

    elif phase == 8:
        if all_drones_close_to_targets(new_formation_positions, position_tolerance):
            change_phase(9)
        elif phase_time >= new_formation_duration:
            change_phase(9)

    elif phase == 9:
        dx = cf2_poses[0, 0] - tunnel_target_drone_1[0]
        dy = cf2_poses[1, 0] - tunnel_target_drone_1[1]
        dz = cf2_poses[2, 0] - tunnel_target_drone_1[2]
        if math.sqrt(dx**2 + dy**2 + dz**2) < position_tolerance:
            change_phase(10)
        elif phase_time >= tunnel_duration_max:
            change_phase(10)

    phase = cf2_control_fn.show_phase
    phase_time = clock - cf2_control_fn.phase_start_clock

    # ========================================================
    # Phase 0: takeoff
    # ========================================================
    if phase == 0:
        z_dist = z_hover
        led = (0, 255, 0)

        if cf2_control_fn.mission_state[robotNo] == 0:
            trigger_takeoff = True

            if robotPose[2] > z_takeoff_threshold:
                cf2_control_fn.mission_state[robotNo] = 1

        vx = 0.0
        vy = 0.0

    # ========================================================
    # Phase 1: vertical formation
    # ========================================================
    elif phase == 1:
        cf2_control_fn.mission_state[robotNo] = 1

        target = vertical_positions[i]
        vx, vy, z_dist = pd_position_control(target)

        led = (80, 120, 255)

    # ========================================================
    # Phase 2: vertical rotation
    # ========================================================
    elif phase == 2:
        cf2_control_fn.mission_state[robotNo] = 1

        if robotNo == 1:
            target = vertical_positions[0]
        else:
            initial_angle = 2.0 * math.pi * (robotNo - 2) / 6.0
            angle = initial_angle + 2.0 * math.pi * phase_time / rotation_duration

            target = np.array([
                vertical_center[0] + vertical_radius * math.cos(angle),
                0.0,
                vertical_center[2] + vertical_radius * math.sin(angle)
            ])

        vx, vy, z_dist = pd_position_control(target)
        led = (255, 120, 0)

    # ========================================================
    # Phase 3: consensus contraction
    # ========================================================
    elif phase == 3:
        progress = smooth_step(phase_time / consensus_duration_max)
        radius = vertical_radius + progress * (safe_consensus_radius - vertical_radius)
        target = vertical_ring_target(radius)

        vx, vy, z_dist = pd_position_control(target)
        led = (255, 0, 255)

    # ========================================================
    # Phase 4: safe consensus expansion
    # ========================================================
    elif phase == 4:
        progress = smooth_step(phase_time / inverse_consensus_duration)
        radius = safe_consensus_radius + progress * (vertical_radius - safe_consensus_radius)
        target = vertical_ring_target(radius)

        vx, vy, z_dist = pd_position_control(target)
        led = (255, 255, 0)

    # ========================================================
    # Phase 5: pairs
    # ========================================================
    elif phase == 5:
        if robotNo == 1:
            target = np.array([0.0, 0.0, z_hover])
        else:
            pair_id = (robotNo - 2) // 2
            member_id = (robotNo - 2) % 2

            pair_angle = 2.0 * math.pi * pair_id / 3.0
            global_angle = pair_angle + 2.0 * math.pi * phase_time / pair_flight_duration

            pair_center_radius = 1.05
            pair_half_distance = 0.30

            pair_center = np.array([
                pair_center_radius * math.cos(global_angle),
                pair_center_radius * math.sin(global_angle),
                z_hover
            ])

            self_rotation_angle = 4.0 * math.pi * phase_time / pair_flight_duration

            if member_id == 0:
                local_sign = 1.0
            else:
                local_sign = -1.0

            local_offset = np.array([
                pair_half_distance * math.cos(self_rotation_angle),
                pair_half_distance * math.sin(self_rotation_angle),
                0.0
            ]) * local_sign

            target = pair_center + local_offset

        vx, vy, z_dist = pd_position_control(target)
        led = (0, 255, 255)

    # ========================================================
    # Phase 6: drones' line
    # ========================================================
    elif phase == 6:
        # First third: move sideways into a lane.
        # Second third: move along that lane to the line slot.
        # Last third: collapse all y values back to 0.
        use_guardrail = phase_time < 2.0 * line_duration / 3.0
        if phase_time < line_duration / 3.0:
            target = np.array([
                robotPose[0],
                lane_offsets[i],
                z_hover
            ])
        elif use_guardrail:
            target = np.array([
                line_positions[i, 0],
                lane_offsets[i],
                line_positions[i, 2]
            ])
        else:
            target = line_positions[i]

        vx, vy, z_dist = pd_position_control(target)
        if use_guardrail:
            vx, vy, z_dist = apply_separation_guardrail(vx, vy, z_dist)
        led = (120, 255, 120)


    # ========================================================
    # Phase 7: travelling S wave
    # ========================================================
    elif phase == 7:
        omega = 2.0 * math.pi / chain_duration
        wave_angle = omega * phase_time + i * math.pi / 3.0

        # Keep the line slots fixed in x. Only y/z make the S wave,
        # so neighboring drones do not chase or cross each other.
        target = np.array([
            line_positions[i, 0],
            0.75 * math.sin(wave_angle),
            z_hover + 0.18 * math.sin(wave_angle + math.pi / 2.0)
        ])

        vx, vy, z_dist = pd_position_control(target)
        led = (180, 80, 255)

    # ========================================================
    # Phase 8: new formation
    # ========================================================
    elif phase == 8:
        use_guardrail = phase_time < 2.0 * new_formation_duration / 3.0
        if phase_time < new_formation_duration / 3.0:
            target = np.array([
                robotPose[0],
                new_formation_positions[i, 1] + lane_offsets[i],
                robotPose[2]
            ])
        elif use_guardrail:
            target = np.array([
                new_formation_positions[i, 0],
                new_formation_positions[i, 1] + lane_offsets[i],
                new_formation_positions[i, 2]
            ])
        else:
            target = new_formation_positions[i]

        vx, vy, z_dist = pd_position_control(target)
        if use_guardrail:
            vx, vy, z_dist = apply_separation_guardrail(vx, vy, z_dist)
        led = (0, 180, 255)

    # ========================================================
    # Phase 9: tunnel
    # ========================================================
    elif phase == 9:
        if robotNo == 1:
            target = tunnel_target_drone_1
        else:
            # Other drones hold the tunnel formation
            target = new_formation_positions[i]

        vx, vy, z_dist = pd_position_control(target)
        vx, vy, z_dist = apply_separation_guardrail(vx, vy, z_dist)
        led = (255, 80, 80)

    # ========================================================
    # Phase 10: landing
    # ========================================================
    elif phase == 10:
        vx = 0.0
        vy = 0.0
        z_dist = 0.0
        trigger_land = True
        led = (0, 255, 0)

        cf2_control_fn.mission_state[robotNo] = 2

        if robotPose[2] < 0.15:
            cf2_control_fn.mission_state[robotNo] = 3
            cf2_control_fn.show_phase = 11

    # ========================================================
    # Phase de sécurité: safety stop
    # ========================================================
    else:
        vx = 0.0
        vy = 0.0
        z_dist = 0.0
        trigger_takeoff = False
        trigger_land = False
        led = (255, 0, 0)
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
