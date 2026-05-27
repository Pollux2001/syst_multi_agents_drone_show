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

    In case of doubt related to the robots, this code or may be something else,
    open a discussion at https://tp-cs.talkyard.net/
    Use your own GitHub account or CS email to signup.
###########################################################################################
'''

import numpy as np
import time

# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ===================

N_RMTT   = 4
N_CF2    = 3
N_TOTAL  = N_RMTT + N_CF2   # 7 drones total

# Snake path parameters
SNAKE_R   = 1.5    # base radius of the circular path (m)
SNAKE_A   = 0.4    # radial undulation amplitude (m)
SNAKE_AZ  = 0.3    # vertical undulation amplitude (m)
SNAKE_M   = 2      # number of body waves (2 = one full S-curve along the body)
SNAKE_SPACING = 0.7  # arc spacing between consecutive drones (rad)

OMEGA_BASE  = 0.25   # nominal rotation speed (rad/s)
Z_RMTT      = 1.0    # RMTT cruise altitude (m)
Z_CF2       = 1.0    # CF2 cruise altitude (m)
DESIRED_GAP = 1.0    # desired Euclidean gap between consecutive drones (m)

global Time2Takeoff
Time2Takeoff = 2

# Per-drone omega proposals for all-to-all consensus
# Indices 0-3 → RMTT, 4-6 → CF2
consensus_omega  = [OMEGA_BASE] * N_TOTAL

# Per-CF2 takeoff flag (avoids race condition on a shared flag)
cf2_takeoff_done = [False] * N_CF2

# ===================================================================================

def snake_path(s):
    """
    Circular snake path with radial undulation.
    r(s) = SNAKE_R + SNAKE_A * sin(SNAKE_M * s)
    Returns (x, y, z) target position.
    """
    r = SNAKE_R + SNAKE_A * np.sin(SNAKE_M * s)
    x = r * np.cos(s)
    y = r * np.sin(s)
    z = Z_RMTT + SNAKE_AZ * np.sin(SNAKE_M * s)
    return x, y, z

def get_drone_xy(g_idx, rmtt_poses, cf2_poses):
    """Return (x, y) of the drone with global index g_idx."""
    if g_idx < N_RMTT:
        return rmtt_poses[0, g_idx], rmtt_poses[1, g_idx]
    else:
        ci = g_idx - N_RMTT
        return cf2_poses[0, ci], cf2_poses[1, ci]

def hsv_to_rgb255(hue):
    """Full-saturation HSV → (r, g, b) each in [0, 255]."""
    h6 = (hue % 1.0) * 6.0
    x  = 1.0 - abs(h6 % 2.0 - 1.0)
    if   h6 < 1: r, g, b = 1.0, x,   0.0
    elif h6 < 2: r, g, b = x,   1.0, 0.0
    elif h6 < 3: r, g, b = 0.0, 1.0, x
    elif h6 < 4: r, g, b = 0.0, x,   1.0
    elif h6 < 5: r, g, b = x,   0.0, 1.0
    else:         r, g, b = 1.0, 0.0, x
    return (int(r * 255), int(g * 255), int(b * 255))

# ===================================================================================
# Control function for turtlebot3 Burger ground vehicle Unicycle model
# should ONLY return (vx,vy) for the robot command
# max useable numbers of robots = 6
# ====================================
def tb3B_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
# ====================================
    #  --- TO BE MODIFIED ---
    vx, vy = 0.0, 0.0
    # -----------------------
    return vx, vy
# ====================================


# ===================================================================================
# Control function for turtlebot3 Waffle ground vehicle Unicycle model
# should ONLY return (vx,vy) for the robot command
# max useable numbers of robots = 2
# ====================================
def tb3W_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
# ====================================
    #  --- TO BE MODIFIED ---
    vx, vy = 0.0, 0.0
    # -----------------------
    return vx, vy
# ====================================


# ====================================
# Control function for dji rmtt drones
# should ONLY return (vx,vy,vz) for the robot command
# max useable numbers of drones = 4
# ====================================
def rmtt_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
# ====================================
    global consensus_omega

    led = (0, 0, 0)
    trigger_land = False
    vx = vy = vz = 0.0

    #  --- TO BE MODIFIED ---

    g_idx = robotNo - 1                              # global index 0-3
    phi_i = g_idx * SNAKE_SPACING                    # arc offset for this drone

    # --- Consensus: read shared omega, compute target point on the snake path ---
    omega    = np.mean(consensus_omega)
    s_i      = omega * clock - phi_i                 # path parameter for this drone
    tx, ty, tz = snake_path(s_i)

    # Consensus vote: measure gap to the drone immediately ahead (g_idx - 1).
    # If too far → propose higher omega (group speeds up to close the gap).
    # If too close → propose lower omega (group slows to space out).
    if g_idx > 0:
        ax, ay = get_drone_xy(g_idx - 1, rmtt_poses, cf2_poses)
        gap    = np.hypot(robotPose[0] - ax, robotPose[1] - ay)
        omega_vote = OMEGA_BASE + 0.08 * (gap - DESIRED_GAP)
    else:
        omega_vote = OMEGA_BASE                      # head always proposes the base speed

    consensus_omega[g_idx] = np.clip(omega_vote, 0.05, 0.6)

    # Proportional velocity toward the target point
    kp = 0.8
    vx = kp * (tx - robotPose[0])
    vy = kp * (ty - robotPose[1])
    vz = 0.5 * (tz - robotPose[2])

    if clock >= 60.0:
        trigger_land = True
        vx = vy = vz = 0.0
        led = (20, 20, 20)
    else:
        # Head (g_idx=0) glows warm yellow; body fades through the rainbow toward tail
        hue = (clock * 0.04 + g_idx / N_TOTAL) % 1.0
        led = hsv_to_rgb255(hue)

    # -----------------------
    return vx, vy, vz, trigger_land, led
# ====================================


# ====================================
# Control function for Crazyflie 2 drones
# should ONLY return (vx,vy,z_dist) for the robot command
# max useable numbers of drones = 3
# ====================================
def cf2_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
    global cf2_takeoff_done, consensus_omega, Time2Takeoff

    vx = vy = 0.0
    z_dist = Z_CF2
    trigger_takeoff = False
    trigger_land    = False
    led = (0, 0, 0)

    #  --- TO BE MODIFIED ---

    idx   = robotNo - 1                              # local CF2 index 0-2
    g_idx = idx + N_RMTT                             # global index 4-6
    phi_i = g_idx * SNAKE_SPACING

    # Takeoff: each drone manages its own flag to avoid thread-pool race conditions
    if not cf2_takeoff_done[idx] and robotPose[2] < 0.05:
        if robotNo == 1:
            time.sleep(Time2Takeoff)
        trigger_takeoff      = True
        cf2_takeoff_done[idx] = True

    elif cf2_takeoff_done[idx]:
        if clock >= 60.0:
            trigger_land = True
            vx = vy = 0.0
            z_dist = Z_CF2
            led = (20, 20, 20)
        else:
            # --- Same consensus + snake logic as RMTT ---
            omega    = np.mean(consensus_omega)
            s_i      = omega * clock - phi_i
            tx, ty, tz = snake_path(s_i)

            ax, ay = get_drone_xy(g_idx - 1, rmtt_poses, cf2_poses)
            gap    = np.hypot(robotPose[0] - ax, robotPose[1] - ay)
            omega_vote = OMEGA_BASE + 0.08 * (gap - DESIRED_GAP)
            consensus_omega[g_idx] = np.clip(omega_vote, 0.05, 0.6)

            kp  = 0.8
            vx  = kp * (tx - robotPose[0])
            vy  = kp * (ty - robotPose[1])
            z_dist = tz

            hue = (clock * 0.04 + g_idx / N_TOTAL) % 1.0
            led = hsv_to_rgb255(hue)

    # -----------------------
    return vx, vy, z_dist, trigger_takeoff, trigger_land, led


# ====================================
# Control function for dji rmep robots
# should ONLY return (vx,vy,wz) for the robot command
# max useable numbers of robots = 2
# ====================================
def rmep_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
# ====================================
    #  --- TO BE MODIFIED ---
    vx, vy, wz = 0.0, 0.0, 0.0
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
