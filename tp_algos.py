#!/usr/bin/python3

import random
import numpy as np
import math, time


# ============== GLOBAL VARIABLES ===================
global TAKEOFF_DONE, Time2Takeoff
TAKEOFF_DONE = False
Time2Takeoff = 0


# ============================================================
# Choose the final six-drone performance here
# "flower"          
# "double_triangle" 
# "wave"            
# "vortex"          
# ============================================================
FINAL_SHOW_MODE = "vortex"


# ============================================================
# Mission timing
# ============================================================
FINAL_TRANSITION_START = 44.0
FINAL_SHOW_START = 48.0
FINAL_SHOW_END = 64.0
MISSION_END = 72.0

K_CONS = 0.18
K_TRACK = 0.78
MAX_U = 0.55


def _clamp01(s):
    return max(0.0, min(1.0, s))


def _linear_interpolation(p0, p1, s):
    s = _clamp01(s)
    return (1.0 - s) * np.array(p0) + s * np.array(p1)


def _rotate_z(v, angle):
    c = math.cos(angle)
    s = math.sin(angle)

    return np.array([
        c * v[0] - s * v[1],
        s * v[0] + c * v[1],
        v[2]
    ])


def _all_air_positions(rmtt_poses, cf2_poses):
    if rmtt_poses.shape[1] == 0 and cf2_poses.shape[1] == 0:
        return np.zeros((3, 0))
    elif rmtt_poses.shape[1] == 0:
        return cf2_poses.copy()
    elif cf2_poses.shape[1] == 0:
        return rmtt_poses.copy()
    else:
        return np.hstack((rmtt_poses, cf2_poses))


# ============================================================
# Basic formation before final show
# ============================================================
def _hex_offset(idx, t, N):
    R = 0.45 if t < 8.0 else 0.42
    angle = 2.0 * math.pi * idx / max(N, 1)

    return np.array([
        R * math.cos(angle),
        R * math.sin(angle),
        0.0
    ])


# ============================================================
# Final show 1: Flower breathing rotation
# ============================================================
def _flower_offset(idx, t):
    base_R = 0.62
    breathe = 0.18 * math.sin(2.0 * math.pi * (t - FINAL_SHOW_START) / 4.0)
    R = base_R + breathe

    angle = 2.0 * math.pi * idx / 6.0
    rotation = 2.0 * math.pi * (t - FINAL_SHOW_START) / 8.0

    return np.array([
        R * math.cos(angle + rotation),
        R * math.sin(angle + rotation),
        0.12 * math.sin(3.0 * angle + rotation)
    ])


# ============================================================
# Final show 2: Double triangle counter-rotation
# ============================================================
def _double_triangle_offset(idx, t):
    R = 0.62

    # Top triangle: drones 0,1,2
    # Bottom triangle: drones 3,4,5
    local_idx = idx % 3
    base_angle = 2.0 * math.pi * local_idx / 3.0

    omega = 2.0 * math.pi * (t - FINAL_SHOW_START) / 8.0

    if idx < 3:
        angle = base_angle + omega
        z = 0.28
    else:
        angle = base_angle - omega + math.pi / 3.0
        z = -0.28

    return np.array([
        R * math.cos(angle),
        R * math.sin(angle),
        z
    ])


# ============================================================
# Final show 3: Wave curtain
# ============================================================
def _wave_offset(idx, t):
    spacing = 0.32
    x = (idx - 2.5) * spacing
    y = 0.0

    phase = 2.0 * math.pi * idx / 6.0
    z = 0.45 * math.sin(2.0 * math.pi * (t - FINAL_SHOW_START) / 4.0 + phase)

    return np.array([x, y, z])


# ============================================================
# Final show 4: Vortex
# ============================================================
def _vortex_offset(idx, t):
    R = 0.35 + 0.06 * idx
    angle = 2.0 * math.pi * idx / 6.0
    rotation = 2.0 * math.pi * (t - FINAL_SHOW_START) / 5.0

    z = 0.35 * math.sin(rotation + angle)

    return np.array([
        R * math.cos(angle + rotation),
        R * math.sin(angle + rotation),
        z
    ])


def _final_show_offset(idx, t):
    if FINAL_SHOW_MODE == "flower":
        return _flower_offset(idx, t)

    elif FINAL_SHOW_MODE == "double_triangle":
        return _double_triangle_offset(idx, t)

    elif FINAL_SHOW_MODE == "wave":
        return _wave_offset(idx, t)

    elif FINAL_SHOW_MODE == "vortex":
        return _vortex_offset(idx, t)

    else:
        return _double_triangle_offset(idx, t)


def _desired_offset(idx, t, N):
    if N != 6:
        return _hex_offset(idx, t, N)

    if t < FINAL_TRANSITION_START:
        return _hex_offset(idx, t, N)

    elif t < FINAL_SHOW_START:
        s = (t - FINAL_TRANSITION_START) / (FINAL_SHOW_START - FINAL_TRANSITION_START)
        old_d = _hex_offset(idx, FINAL_TRANSITION_START, N)
        new_d = _final_show_offset(idx, FINAL_SHOW_START)
        return _linear_interpolation(old_d, new_d, s)

    elif t < FINAL_SHOW_END:
        return _final_show_offset(idx, t)

    else:
        return _final_show_offset(idx, FINAL_SHOW_END)


# ============================================================
# Center trajectories: heart and infinity are kept
# ============================================================
def _heart_center(t):
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


def _infinity_center(t):
    T = 16.0
    tau = 2.0 * math.pi * ((t - 28.0) / T)

    x = 1.15 * math.sin(tau)
    y = 0.10
    z = 2.00 + 0.55 * math.sin(2.0 * tau)

    return np.array([x, y, z])


def _reference_center(t):
    launch_center = np.array([0.0, -2.75, 1.10])
    start_draw = np.array([0.0, -0.45, 2.10])
    final_show_center = np.array([0.0, 0.20, 2.00])
    landing_center = np.array([0.0, -2.75, 0.85])

    if t < 3.0:
        return launch_center

    elif t < 8.0:
        s = (t - 3.0) / 5.0
        return _linear_interpolation(launch_center, start_draw, s)

    elif t < 24.0:
        return _heart_center(t)

    elif t < FINAL_TRANSITION_START:
        if t < 28.0:
            s = (t - 24.0) / 4.0
            return _linear_interpolation(_heart_center(24.0), _infinity_center(28.0), s)
        else:
            return _infinity_center(t)

    elif t < FINAL_SHOW_START:
        s = (t - FINAL_TRANSITION_START) / (FINAL_SHOW_START - FINAL_TRANSITION_START)
        return _linear_interpolation(_infinity_center(FINAL_TRANSITION_START), final_show_center, s)

    elif t < FINAL_SHOW_END:
        return final_show_center

    elif t < MISSION_END:
        s = (t - FINAL_SHOW_END) / (MISSION_END - FINAL_SHOW_END)
        return _linear_interpolation(final_show_center, landing_center, s)

    else:
        return landing_center


# ============================================================
# Shared consensus + formation controller
# ============================================================
def _formation_control(agent_id, robotPose, rmtt_poses, cf2_poses, clock):
    all_pos = _all_air_positions(rmtt_poses, cf2_poses)
    N = all_pos.shape[1]

    if N == 0:
        return np.zeros(3)

    p_i = np.array([robotPose[0], robotPose[1], robotPose[2]])
    d_i = _desired_offset(agent_id, clock, N)
    q_i = p_i - d_i

    u_cons = np.zeros(3)

    for j in range(N):
        if j == agent_id:
            continue

        p_j = all_pos[:, j]
        d_j = _desired_offset(j, clock, N)
        q_j = p_j - d_j

        u_cons += -(q_i - q_j)

    if N > 1:
        u_cons = K_CONS * u_cons / (N - 1)

    c_ref = _reference_center(clock)
    p_des = c_ref + d_i
    u_track = K_TRACK * (p_des - p_i)

    u = u_cons + u_track

    norm_u = np.linalg.norm(u)
    if norm_u > MAX_U:
        u = u * MAX_U / norm_u

    return u


# ===================================================================================
# Turtlebot3 Burger
# ===================================================================================
def tb3B_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
    vx = 0.0
    vy = 0.0
    return vx, vy


# ===================================================================================
# Turtlebot3 Waffle
# ===================================================================================
def tb3W_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
    vx = 0.0
    vy = 0.0
    return vx, vy


# ===================================================================================
# RMTT drones
# ===================================================================================
def rmtt_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
    led = (0, 0, 0)
    trigger_land = False

    agent_id = robotNo - 1

    if clock >= MISSION_END:
        vx = 0.0
        vy = 0.0
        vz = 0.0
        trigger_land = True
        led = (0, 255, 0)
        return vx, vy, vz, trigger_land, led

    u = _formation_control(agent_id, robotPose, rmtt_poses, cf2_poses, clock)

    vx = u[0]
    vy = u[1]
    vz = u[2]

    if clock < 24.0:
        led = (255, 0, 0)
    elif clock < 44.0:
        led = (0, 80, 255)
    elif clock < 64.0:
        led = (255, 220, 0)
    else:
        led = (255, 255, 255)

    return vx, vy, vz, trigger_land, led


# ===================================================================================
# Crazyflie 2 drones
# ===================================================================================
def cf2_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
    global TAKEOFF_DONE, Time2Takeoff

    led = (0, 0, 0)

    vx = 0.0
    vy = 0.0
    z_dist = 1.0
    trigger_takeoff = False
    trigger_land = False

    nbRMTT = len(rmtt_poses[0])
    agent_id = nbRMTT + robotNo - 1

    if robotPose[2] < 0.05 and clock < 8.0:
        trigger_takeoff = True
        led = (255, 255, 255)
        return vx, vy, z_dist, trigger_takeoff, trigger_land, led

    if clock >= MISSION_END:
        vx = 0.0
        vy = 0.0
        z_dist = 0.0
        trigger_land = True
        led = (0, 255, 0)
        return vx, vy, z_dist, trigger_takeoff, trigger_land, led

    u = _formation_control(agent_id, robotPose, rmtt_poses, cf2_poses, clock)

    vx = u[0]
    vy = u[1]

    z_dist = robotPose[2] + u[2]
    z_dist = max(0.7, min(3.0, z_dist))

    if clock < 24.0:
        led = (255, 0, 0)
    elif clock < 44.0:
        led = (0, 80, 255)
    elif clock < 64.0:
        led = (255, 220, 0)
    else:
        led = (255, 255, 255)

    return vx, vy, z_dist, trigger_takeoff, trigger_land, led


# ===================================================================================
# RMEP robots
# ===================================================================================
def rmep_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
    vx = 0.0
    vy = 0.0
    wz = 0.0
    return vx, vy, wz


# ======== DO NOT MODIFY ============
def tb3B_controller(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
    vx, vy = tb3B_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock)
    return vx, vy


def tb3W_controller(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
    vx, vy = tb3W_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock)
    return vx, vy


def rmtt_controller(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
    vx, vy, vz, trigger_land, led = rmtt_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock)
    return vx, vy, vz, trigger_land, led


def cf2_controller(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
    vx, vy, z_dist, trigger_takeoff, trigger_land, led = cf2_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock)
    return vx, vy, z_dist, trigger_takeoff, trigger_land, led


def rmep_controller(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock):
    vx, vy, wz = rmep_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, clock)
    return vx, vy, wz