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
global TAKEOFF_DONE, Time2Takeoff
TAKEOFF_DONE = False
Time2Takeoff = 0


# ===================================================================================
# Control function for turtlebot3 Burger ground vehicle Unicycle model
# should ONLY return (vx,vy) for the robot command
# max useable numbers of robots = 6
# ====================================
def tb3B_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
# ====================================

    nbTB3= len(tb3B_poses[0])
    nbTB3W = len(tb3W_poses[0])
    nbRMTT = len(rmtt_poses[0])
    nbCF2 = len(cf2_poses[0])
    nbRMEP = len(rmep_poses[0])
    nbOBSTACLE = len(obstacle_pose[0])

    #  --- TO BE MODIFIED ---

    # Ground robots are not used in this air-writing mission.
    vx = 0.0
    vy = 0.0

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

    nbTB3= len(tb3B_poses[0])
    nbTB3W = len(tb3W_poses[0])
    nbRMTT = len(rmtt_poses[0])
    nbCF2 = len(cf2_poses[0])
    nbRMEP = len(rmep_poses[0])
    nbOBSTACLE = len(obstacle_pose[0])

    #  --- TO BE MODIFIED ---

    # Ground robots are not used in this air-writing mission.
    vx = 0.0
    vy = 0.0

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
    nbTB3= len(tb3B_poses[0])
    nbTB3W = len(tb3W_poses[0])
    nbRMTT = len(rmtt_poses[0])
    nbCF2 = len(cf2_poses[0])
    nbRMEP = len(rmep_poses[0])
    nbOBSTACLE = len(obstacle_pose[0])
    led = (0,0,0)

    #  --- TO BE MODIFIED ---

    # ============================================================
    # Six-drone air-writing mission:
    # RMTT #1,#2,#3 -> virtual agents 0,1,2
    # CF2  #1,#2,#3 -> virtual agents 3,4,5
    #
    # TD1 consensus idea:
    #     q_i = p_i - d_i
    #     all q_i should agree on the same formation center.
    #
    # TD2 formation idea:
    #     p_i should track c_ref(t) + d_i(t).
    # ============================================================

    agent_id = robotNo - 1
    trigger_land = False

    def all_air_positions():
        if rmtt_poses.shape[1] == 0 and cf2_poses.shape[1] == 0:
            return np.zeros((3, 0))
        elif rmtt_poses.shape[1] == 0:
            return cf2_poses.copy()
        elif cf2_poses.shape[1] == 0:
            return rmtt_poses.copy()
        else:
            return np.hstack((rmtt_poses, cf2_poses))

    def linear_interpolation(p0, p1, s):
        s = max(0.0, min(1.0, s))
        return (1.0 - s) * np.array(p0) + s * np.array(p1)

    def formation_radius(t):
        if t < 8.0:
            return 0.45
        elif t < 44.0:
            return 0.42
        elif t < 60.0:
            return 0.55 + 0.10 * math.sin(2.5 * t)
        else:
            return 0.45

    def desired_offset(idx, t, N):
        R = formation_radius(t)

        # Rotate the hexagonal formation during the star/firework stage.
        omega = 0.0
        if 44.0 <= t < 60.0:
            omega = 1.2 * (t - 44.0)

        angle = 2.0 * math.pi * idx / N + omega

        return np.array([
            R * math.cos(angle),
            R * math.sin(angle),
            0.0
        ])

    def heart_center(t):
        # Heart drawn in the vertical X-Z plane.
        T = 16.0
        tau = 2.0 * math.pi * ((t - 8.0) / T)

        x_raw = 16.0 * (math.sin(tau) ** 3)
        z_raw = (
            13.0 * math.cos(tau)
            - 5.0 * math.cos(2.0 * tau)
            - 2.0 * math.cos(3.0 * tau)
            - math.cos(4.0 * tau)
        )

        x = 0.075 * x_raw
        y = -0.45
        z = 1.85 + 0.055 * z_raw

        return np.array([x, y, z])

    def infinity_center(t):
        # Infinity symbol drawn in the vertical X-Z plane.
        T = 16.0
        tau = 2.0 * math.pi * ((t - 28.0) / T)

        x = 1.15 * math.sin(tau)
        y = 0.10
        z = 2.00 + 0.55 * math.sin(2.0 * tau)

        return np.array([x, y, z])

    def star_center(t):
        # Five-point star drawn in the vertical X-Z plane.
        T = 16.0
        local_t = (t - 44.0) % T

        angles_deg = [90, 234, 18, 162, 306, 90]
        points = []

        for a_deg in angles_deg:
            a = math.radians(a_deg)
            points.append(np.array([
                0.95 * math.cos(a),
                0.65,
                2.00 + 0.75 * math.sin(a)
            ]))

        seg_T = T / 5.0
        k = int(local_t // seg_T)
        k = min(k, 4)
        s = (local_t - k * seg_T) / seg_T

        return linear_interpolation(points[k], points[k + 1], s)

    def reference_center(t):
        # Center trajectory of the whole six-drone formation.
        launch_center = np.array([0.0, -2.75, 1.10])
        start_draw = np.array([0.0, -0.45, 2.10])

        if t < 3.0:
            return launch_center

        elif t < 8.0:
            s = (t - 3.0) / 5.0
            return linear_interpolation(launch_center, start_draw, s)

        elif t < 24.0:
            return heart_center(t)

        elif t < 44.0:
            if t < 28.0:
                s = (t - 24.0) / 4.0
                return linear_interpolation(heart_center(24.0), infinity_center(28.0), s)
            else:
                return infinity_center(t)

        elif t < 60.0:
            return star_center(t)

        elif t < 68.0:
            s = (t - 60.0) / 8.0
            return linear_interpolation(star_center(60.0), launch_center, s)

        else:
            return launch_center

    # After the mission, ask RMTT to land.
    if clock >= 68.0:
        vx = 0.0
        vy = 0.0
        vz = 0.0
        trigger_land = True
        led = (0, 255, 0)
        return vx, vy, vz, trigger_land, led

    all_pos = all_air_positions()
    N = all_pos.shape[1]

    if N == 0:
        vx = 0.0
        vy = 0.0
        vz = 0.0
        return vx, vy, vz, trigger_land, led

    p_i = np.array([robotPose[0], robotPose[1], robotPose[2]])
    d_i = desired_offset(agent_id, clock, N)
    q_i = p_i - d_i

    # Fully connected graph.
    # This is the consensus part.
    k_cons = 0.18
    u_cons = np.zeros(3)

    for j in range(N):
        if j == agent_id:
            continue

        p_j = all_pos[:, j]
        d_j = desired_offset(j, clock, N)
        q_j = p_j - d_j

        u_cons += -(q_i - q_j)

    if N > 1:
        u_cons = k_cons * u_cons / (N - 1)

    # Formation tracking part.
    k_track = 0.75
    c_ref = reference_center(clock)
    p_des = c_ref + d_i
    u_track = k_track * (p_des - p_i)

    u = u_cons + u_track

    # Safety clipping before simulator speed limit.
    max_u = 0.55
    norm_u = np.linalg.norm(u)

    if norm_u > max_u:
        u = u * max_u / norm_u

    vx = u[0]
    vy = u[1]
    vz = u[2]

    # LED color is only used as semantic information.
    if clock < 24.0:
        led = (255, 0, 0)        # heart
    elif clock < 44.0:
        led = (0, 80, 255)       # infinity
    elif clock < 60.0:
        led = (255, 220, 0)      # star/firework
    else:
        led = (255, 255, 255)    # return

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
    nbTB3= len(tb3B_poses[0])
    nbTB3W = len(tb3W_poses[0])
    nbRMTT = len(rmtt_poses[0])
    nbCF2 = len(cf2_poses[0])
    nbRMEP = len(rmep_poses[0])
    nbOBSTACLE = len(obstacle_pose[0])
    led = (0,0,0)

    #  --- TO BE MODIFIED ---

    # ============================================================
    # Six-drone air-writing mission:
    # RMTT #1,#2,#3 -> virtual agents 0,1,2
    # CF2  #1,#2,#3 -> virtual agents 3,4,5
    #
    # The same formation-consensus law is used here.
    # Difference:
    #     RMTT returns vz.
    #     CF2 returns z_dist, so z command is converted below.
    # ============================================================

    vx = 0.0
    vy = 0.0
    z_dist = 1.0
    trigger_takeoff = False
    trigger_land = False

    agent_id = nbRMTT + robotNo - 1

    def all_air_positions():
        if rmtt_poses.shape[1] == 0 and cf2_poses.shape[1] == 0:
            return np.zeros((3, 0))
        elif rmtt_poses.shape[1] == 0:
            return cf2_poses.copy()
        elif cf2_poses.shape[1] == 0:
            return rmtt_poses.copy()
        else:
            return np.hstack((rmtt_poses, cf2_poses))

    def linear_interpolation(p0, p1, s):
        s = max(0.0, min(1.0, s))
        return (1.0 - s) * np.array(p0) + s * np.array(p1)

    def formation_radius(t):
        if t < 8.0:
            return 0.45
        elif t < 44.0:
            return 0.42
        elif t < 60.0:
            return 0.55 + 0.10 * math.sin(2.5 * t)
        else:
            return 0.45

    def desired_offset(idx, t, N):
        R = formation_radius(t)

        omega = 0.0
        if 44.0 <= t < 60.0:
            omega = 1.2 * (t - 44.0)

        angle = 2.0 * math.pi * idx / N + omega

        return np.array([
            R * math.cos(angle),
            R * math.sin(angle),
            0.0
        ])

    def heart_center(t):
        T = 16.0
        tau = 2.0 * math.pi * ((t - 8.0) / T)

        x_raw = 16.0 * (math.sin(tau) ** 3)
        z_raw = (
            13.0 * math.cos(tau)
            - 5.0 * math.cos(2.0 * tau)
            - 2.0 * math.cos(3.0 * tau)
            - math.cos(4.0 * tau)
        )

        x = 0.075 * x_raw
        y = -0.45
        z = 1.85 + 0.055 * z_raw

        return np.array([x, y, z])

    def infinity_center(t):
        T = 16.0
        tau = 2.0 * math.pi * ((t - 28.0) / T)

        x = 1.15 * math.sin(tau)
        y = 0.10
        z = 2.00 + 0.55 * math.sin(2.0 * tau)

        return np.array([x, y, z])

    def star_center(t):
        T = 16.0
        local_t = (t - 44.0) % T

        angles_deg = [90, 234, 18, 162, 306, 90]
        points = []

        for a_deg in angles_deg:
            a = math.radians(a_deg)
            points.append(np.array([
                0.95 * math.cos(a),
                0.65,
                2.00 + 0.75 * math.sin(a)
            ]))

        seg_T = T / 5.0
        k = int(local_t // seg_T)
        k = min(k, 4)
        s = (local_t - k * seg_T) / seg_T

        return linear_interpolation(points[k], points[k + 1], s)

    def reference_center(t):
        launch_center = np.array([0.0, -2.75, 1.10])
        start_draw = np.array([0.0, -0.45, 2.10])

        if t < 3.0:
            return launch_center

        elif t < 8.0:
            s = (t - 3.0) / 5.0
            return linear_interpolation(launch_center, start_draw, s)

        elif t < 24.0:
            return heart_center(t)

        elif t < 44.0:
            if t < 28.0:
                s = (t - 24.0) / 4.0
                return linear_interpolation(heart_center(24.0), infinity_center(28.0), s)
            else:
                return infinity_center(t)

        elif t < 60.0:
            return star_center(t)

        elif t < 68.0:
            s = (t - 60.0) / 8.0
            return linear_interpolation(star_center(60.0), launch_center, s)

        else:
            return launch_center

    # Trigger takeoff at the beginning.
    if robotPose[2] < 0.05 and clock < 8.0:
        trigger_takeoff = True
        led = (255, 255, 255)
        return vx, vy, z_dist, trigger_takeoff, trigger_land, led

    # After the mission, ask CF2 to land.
    if clock >= 68.0:
        vx = 0.0
        vy = 0.0
        z_dist = 0.0
        trigger_land = True
        led = (0, 255, 0)
        return vx, vy, z_dist, trigger_takeoff, trigger_land, led

    all_pos = all_air_positions()
    N = all_pos.shape[1]

    if N == 0:
        return vx, vy, z_dist, trigger_takeoff, trigger_land, led

    p_i = np.array([robotPose[0], robotPose[1], robotPose[2]])
    d_i = desired_offset(agent_id, clock, N)
    q_i = p_i - d_i

    # Fully connected graph.
    # This is the consensus part.
    k_cons = 0.18
    u_cons = np.zeros(3)

    for j in range(N):
        if j == agent_id:
            continue

        p_j = all_pos[:, j]
        d_j = desired_offset(j, clock, N)
        q_j = p_j - d_j

        u_cons += -(q_i - q_j)

    if N > 1:
        u_cons = k_cons * u_cons / (N - 1)

    # Formation tracking part.
    k_track = 0.75
    c_ref = reference_center(clock)
    p_des = c_ref + d_i
    u_track = k_track * (p_des - p_i)

    u = u_cons + u_track

    # Safety clipping before simulator speed limit.
    max_u = 0.55
    norm_u = np.linalg.norm(u)

    if norm_u > max_u:
        u = u * max_u / norm_u

    vx = u[0]
    vy = u[1]

    # CF2 returns target altitude, not vertical velocity.
    z_dist = robotPose[2] + u[2]
    z_dist = max(0.7, min(3.0, z_dist))

    if clock < 24.0:
        led = (255, 0, 0)        # heart
    elif clock < 44.0:
        led = (0, 80, 255)       # infinity
    elif clock < 60.0:
        led = (255, 220, 0)      # star/firework
    else:
        led = (255, 255, 255)    # return

    # -----------------------

    return vx, vy, z_dist, trigger_takeoff, trigger_land, led


# ====================================
# Control function for dji rmep robots omnidirectional robots with gripper
# should ONLY return (vx,vy,wz) for the robot command
# max useable numbers of robots = 2
# ====================================
def rmep_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
# ====================================
    nbTB3= len(tb3B_poses[0])
    nbTB3W = len(tb3W_poses[0])
    nbRMTT = len(rmtt_poses[0])
    nbCF2 = len(cf2_poses[0])
    nbRMEP = len(rmep_poses[0])
    nbOBSTACLE = len(obstacle_pose[0])

    #  --- TO BE MODIFIED ---

    # RMEP robots are not used in this air-writing mission.
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