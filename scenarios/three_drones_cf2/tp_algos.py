#!/usr/bin/python3
'''
    CentraleSupelec TP 2A/3A
    (all variables in SI unit)

    Scenario: three_drones_cf2
    Phase 0: take off from equilateral triangle → straight line on X axis at z=1m
    Phase 1: straight line on Z axis (vertical, x=0 y=0)
    Phase 2: straight line on Y axis at z=1m
    Phase 3: land and shut down
    Algorithm: PID controller toward fixed targets.
'''

import numpy as np
import time

# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ===================

N_CF2 = 3

global Time2Takeoff
Time2Takeoff = 3

cf2_takeoff_done = [False] * N_CF2

# Phase 0 — line on X axis at z=1m
TARGETS_X = [
    (-1.0, 0.0, 1.0),
    ( 0.0, 0.0, 1.0),
    ( 1.0, 0.0, 1.0),
]

# Phase 1 — vertical line on Z axis (x=0, y=0)
TARGETS_Z = [
    (0.0, 0.0, 0.7),
    (0.0, 0.0, 1.4),
    (0.0, 0.0, 2.1),
]

# Phase 2 — line on Y axis at z=1m
TARGETS_Y = [
    (0.0, -1.0, 1.0),
    (0.0,  0.0, 1.0),
    (0.0,  1.0, 1.0),
]

ARRIVAL_THRESH = 0.12  # meters

x_line_reached = [False] * N_CF2
z_line_reached = [False] * N_CF2
y_line_reached = [False] * N_CF2
phase = 0  # 0=X line, 1=Z line, 2=Y line, 3=land

# PID gains (XY plane)
KP   = 0.8
KI   = 0.05
KD   = 0.3
DT   = 0.05   # must match the simulation timestep
I_MAX = 1.5   # anti-windup clamp on the integral term

# Per-drone PID state — reset when the target changes phase
pid_integral   = [[0.0, 0.0] for _ in range(N_CF2)]  # [ix, iy]
pid_prev_error = [[0.0, 0.0] for _ in range(N_CF2)]  # [ex_prev, ey_prev]
pid_prev_phase = [-1] * N_CF2                         # detect phase change → reset

# ===================================================================================
def tb3B_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
    return 0.0, 0.0

def tb3W_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
    return 0.0, 0.0

def rmtt_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
    return 0.0, 0.0, 0.0, False, (0, 0, 0)

# ====================================
# Control function for Crazyflie 2 drones
# ====================================
def cf2_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
    global cf2_takeoff_done, Time2Takeoff
    global phase, x_line_reached, z_line_reached, y_line_reached
    global pid_integral, pid_prev_error, pid_prev_phase

    idx = robotNo - 1

    vx = 0.0
    vy = 0.0
    z_dist = 1.0
    trigger_takeoff = False
    trigger_land    = False
    led = (0, 0, 0)

    if not cf2_takeoff_done[idx] and robotPose[2] < 0.05:
        if robotNo == 1:
            time.sleep(Time2Takeoff)
        trigger_takeoff       = True
        cf2_takeoff_done[idx] = True

    elif cf2_takeoff_done[idx]:

        def dist_to(target):
            return np.sqrt(
                (robotPose[0] - target[0])**2 +
                (robotPose[1] - target[1])**2 +
                (robotPose[2] - target[2])**2
            )

        if phase == 0:
            tx, ty, tz = TARGETS_X[idx]
            if dist_to(TARGETS_X[idx]) < ARRIVAL_THRESH:
                x_line_reached[idx] = True
            if all(x_line_reached):
                phase = 1
            led = [(255, 60, 60), (60, 255, 60), (60, 60, 255)][idx]

        elif phase == 1:
            tx, ty, tz = TARGETS_Z[idx]
            if dist_to(TARGETS_Z[idx]) < ARRIVAL_THRESH:
                z_line_reached[idx] = True
            if all(z_line_reached):
                phase = 2
            led = [(255, 140, 0), (255, 255, 0), (255, 0, 200)][idx]

        elif phase == 2:
            tx, ty, tz = TARGETS_Y[idx]
            if dist_to(TARGETS_Y[idx]) < ARRIVAL_THRESH:
                y_line_reached[idx] = True
            if all(y_line_reached):
                phase = 3
            led = [(0, 220, 255), (0, 120, 255), (180, 0, 255)][idx]

        else:  # phase == 3 — land
            tx, ty, tz = robotPose[0], robotPose[1], robotPose[2]
            trigger_land = True
            led = (20, 20, 20)

        # Reset integrator on phase change
        if phase != pid_prev_phase[idx]:
            pid_integral[idx]   = [0.0, 0.0]
            pid_prev_error[idx] = [tx - robotPose[0], ty - robotPose[1]]
            pid_prev_phase[idx] = phase

        ex = tx - robotPose[0]
        ey = ty - robotPose[1]

        # Integral with anti-windup
        pid_integral[idx][0] = np.clip(pid_integral[idx][0] + ex * DT, -I_MAX, I_MAX)
        pid_integral[idx][1] = np.clip(pid_integral[idx][1] + ey * DT, -I_MAX, I_MAX)

        # Derivative
        dex = (ex - pid_prev_error[idx][0]) / DT
        dey = (ey - pid_prev_error[idx][1]) / DT
        pid_prev_error[idx] = [ex, ey]

        vx     = KP * ex + KI * pid_integral[idx][0] + KD * dex
        vy     = KP * ey + KI * pid_integral[idx][1] + KD * dey
        z_dist = tz

    return vx, vy, z_dist, trigger_takeoff, trigger_land, led

def rmep_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
    return 0.0, 0.0, 0.0


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
