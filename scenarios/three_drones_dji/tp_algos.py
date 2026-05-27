#!/usr/bin/python3
'''
    CentraleSupelec TP 2A/3A
    (all variables in SI unit)

    Scenario: three_drones_dji
    Phase 0: take off from equilateral triangle → straight line on X axis at z=0.8m
    Phase 1: straight line on Z axis (vertical, x=0 y=0)
    Phase 2: straight line on Y axis at z=0.8m
    Phase 3: land and shut down
    Algorithm: PID controller toward fixed targets.
'''

import numpy as np
import time

# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ===================

N_RMTT = 3

HOVER_Z = 0.8  # RMTT cruise altitude

# Phase 0 — line on X axis at z=0.8m
TARGETS_X = [
    (-1.0, 0.0, HOVER_Z),
    ( 0.0, 0.0, HOVER_Z),
    ( 1.0, 0.0, HOVER_Z),
]

# Phase 1 — vertical line on Z axis (x=0, y=0)
TARGETS_Z = [
    (0.0, 0.0, 0.4),
    (0.0, 0.0, 0.9),
    (0.0, 0.0, 1.5),
]

# Phase 2 — line on Y axis at z=0.8m
TARGETS_Y = [
    (0.0, -1.0, HOVER_Z),
    (0.0,  0.0, HOVER_Z),
    (0.0,  1.0, HOVER_Z),
]

ARRIVAL_THRESH = 0.12  # meters

x_line_reached = [False] * N_RMTT
z_line_reached = [False] * N_RMTT
y_line_reached = [False] * N_RMTT
phase = 0  # 0=X line, 1=Z line, 2=Y line, 3=land

# Ready flag: True once the drone has finished its auto-takeoff climb
rmtt_ready = [False] * N_RMTT

# PID gains
KP    = 0.8
KI    = 0.05
KD    = 0.3
KP_Z  = 0.6
KI_Z  = 0.03
KD_Z  = 0.2
DT    = 0.05
V_MAX   = 0.5   # per-axis velocity saturation limit (m/s) for XY
VZ_MAX  = 0.4   # per-axis velocity saturation limit (m/s) for Z
I_MAX   = V_MAX  / KI    # integral XY limit derived from saturation
I_MAX_Z = VZ_MAX / KI_Z  # integral Z  limit derived from saturation

# Per-drone PID state
pid_integral    = [[0.0, 0.0, 0.0] for _ in range(N_RMTT)]  # [ix, iy, iz]
pid_prev_error  = [[0.0, 0.0, 0.0] for _ in range(N_RMTT)]  # [ex, ey, ez]
pid_prev_phase  = [-1] * N_RMTT

# ===================================================================================
def tb3B_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
    return 0.0, 0.0

def tb3W_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
    return 0.0, 0.0

# ====================================
# Control function for DJI RMTT drones
# ====================================
def rmtt_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
    global rmtt_ready, phase
    global x_line_reached, z_line_reached, y_line_reached
    global pid_integral, pid_prev_error, pid_prev_phase

    idx = robotNo - 1

    vx = 0.0
    vy = 0.0
    vz = 0.0
    trigger_land = False
    led = (0, 0, 0)

    # Wait until auto-takeoff is complete before activating PID
    if not rmtt_ready[idx]:
        if robotPose[2] >= HOVER_Z - 0.05:
            rmtt_ready[idx] = True
        return vx, vy, vz, trigger_land, led

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
        trigger_land = True
        led = (20, 20, 20)
        return vx, vy, vz, trigger_land, led

    # Reset integrator on phase change
    if phase != pid_prev_phase[idx]:
        pid_integral[idx]   = [0.0, 0.0, 0.0]
        pid_prev_error[idx] = [tx - robotPose[0], ty - robotPose[1], tz - robotPose[2]]
        pid_prev_phase[idx] = phase

    ex = tx - robotPose[0]
    ey = ty - robotPose[1]
    ez = tz - robotPose[2]

    # Derivatives
    dex = (ex - pid_prev_error[idx][0]) / DT
    dey = (ey - pid_prev_error[idx][1]) / DT
    dez = (ez - pid_prev_error[idx][2]) / DT
    pid_prev_error[idx] = [ex, ey, ez]

    # Raw PID outputs (using integrals from previous step)
    vx_raw = KP   * ex + KI   * pid_integral[idx][0] + KD   * dex
    vy_raw = KP   * ey + KI   * pid_integral[idx][1] + KD   * dey
    vz_raw = KP_Z * ez + KI_Z * pid_integral[idx][2] + KD_Z * dez

    # Output saturation
    vx = np.clip(vx_raw, -V_MAX,  V_MAX)
    vy = np.clip(vy_raw, -V_MAX,  V_MAX)
    vz = np.clip(vz_raw, -VZ_MAX, VZ_MAX)

    # Conditional anti-windup: only accumulate integral when output is NOT saturated
    if abs(vx_raw) < V_MAX:
        pid_integral[idx][0] = np.clip(pid_integral[idx][0] + ex * DT, -I_MAX,   I_MAX)
    if abs(vy_raw) < V_MAX:
        pid_integral[idx][1] = np.clip(pid_integral[idx][1] + ey * DT, -I_MAX,   I_MAX)
    if abs(vz_raw) < VZ_MAX:
        pid_integral[idx][2] = np.clip(pid_integral[idx][2] + ez * DT, -I_MAX_Z, I_MAX_Z)

    return vx, vy, vz, trigger_land, led

def cf2_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
    return 0.0, 0.0, 1.0, False, False, (0, 0, 0)

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
