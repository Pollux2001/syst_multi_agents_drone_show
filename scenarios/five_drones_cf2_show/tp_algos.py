#!/usr/bin/python3
'''
    CentraleSupelec TP 2A/3A
    (all variables in SI unit)

    Scenario: five_drones_cf2_show  —  5 Crazyflie 2 drones
    Phase 0  : takeoff to 1 m
    Phase 1  : form horizontal pentagon
    Phase 2  : one full rotation of the pentagon
    Phase 3  : partial consensus   (ring 1→2→3→4→5→1, drones converge)
    Phase 4  : inverse consensus   (same ring, drones diverge)
    Phase 5  : straight line on Y axis
    Phase 6  : Z shape in the XZ plane (viewed from Y)
    Phase 7  : vertical pentagon in the XZ plane
    Phase 8  : rotating vertical pentagon (one full turn)
    Phase 9  : straight line on Y axis again
    Phase 10 : each drone lands individually
'''

import numpy as np
import math

# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ===================

global Time2Takeoff
Time2Takeoff = 1   # seconds before first takeoff trigger

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
    global Time2Takeoff

    nbCF2 = len(cf2_poses[0])   # 5
    i     = robotNo - 1          # drone index 0-4

    # ============================================================
    # Show parameters
    # ============================================================
    z_hover             = 1.0
    z_takeoff_threshold = 0.9 * z_hover

    PENTA_R           = 1.2    # horizontal pentagon radius (m)
    VERT_R            = 1.0    # vertical pentagon radius (m)
    ROTATION_DURATION = 12.0   # seconds for one full 360° rotation
    FORMATION_TIMEOUT = 10.0   # max time to wait for a formation to settle
    ARRIVAL_TOL       = 0.15   # 3-D arrival threshold (m)

    LINE_Y_SPACING = 1.0   # distance between consecutive drones in the Y-line (m)
    VERT_Z_OFFSET  = 0.4   # extra height added to the vertical pentagon center (m)

    Z_HALF_W = 0.8   # half-width of the Z shape along X (m)
    Z_Z_TOP  = 1.8   # Z height of the top bar of the Z shape (m)
    Z_Z_BOT  = 0.5   # Z height of the bottom bar of the Z shape (m)

    K_FORM = 0.5    # displacement-based formation gain (relative position errors)
    K_ABS  = 0.35   # absolute anchoring gain (centroid stabilization)

    CONSENSUS_THRESH  = 0.5    # max pairwise distance to end consensus phase (m)
    CONSENSUS_TIMEOUT = 10.0   # max duration of consensus phase (s)
    INVERSE_THRESH    = 1.3    # min pairwise distance to end inverse consensus (m)
    INVERSE_TIMEOUT   = 8.0    # max duration of inverse consensus phase (s)

    MIN_DIST  = 0.5    # hard minimum distance between drones (m)
    SAFE_DIST = 0.95   # repulsion activation distance (m) — must be > MIN_DIST
    K_REP     = 0.55   # APF repulsion gain

    # Pentagon angles for 5 drones
    penta_angles = [k * 2.0 * math.pi / 5 for k in range(5)]

    # Phase 1 — horizontal pentagon targets
    penta_targets = [
        (PENTA_R * math.cos(penta_angles[k]),
         PENTA_R * math.sin(penta_angles[k]),
         z_hover)
        for k in range(5)
    ]

    # Phase 5 & 9 — Y-axis line targets (x=0, z=z_hover), spacing = LINE_Y_SPACING
    y_line_targets = [
        (0.0, (-2 + k) * LINE_Y_SPACING, z_hover)
        for k in range(5)
    ]

    # Phase 6 — Z shape in XZ plane (y=0), viewed from Y direction
    #   0---1          top bar   (top-left, top-right)
    #      /
    #     2            diagonal center
    #    /
    #   3---4          bottom bar (bottom-left, bottom-right)
    z_shape_targets = [
        (-Z_HALF_W, 0.0, Z_Z_TOP),                       # 0 top-left
        ( Z_HALF_W, 0.0, Z_Z_TOP),                       # 1 top-right
        ( 0.0,      0.0, (Z_Z_TOP + Z_Z_BOT) / 2.0),    # 2 diagonal center
        (-Z_HALF_W, 0.0, Z_Z_BOT),                       # 3 bottom-left
        ( Z_HALF_W, 0.0, Z_Z_BOT),                       # 4 bottom-right
    ]

    # Phase 7 — vertical pentagon in XZ plane (y=0), center raised by VERT_Z_OFFSET
    vert_targets = [
        (VERT_R * math.cos(penta_angles[k]),
         0.0,
         z_hover + VERT_Z_OFFSET + VERT_R * math.sin(penta_angles[k]))
        for k in range(5)
    ]

    # PD gains
    kp_xy  = 1.2
    kd_xy  = 0.20
    kp_z   = 1.2
    kd_z   = 0.18
    deriv_tau    = 0.35   # low-pass filter time constant for derivative
    max_vxy_cmd  = 0.55
    max_vz_cmd   = 0.50

    # ============================================================
    # Persistent state — initialised once for the whole simulation
    # ============================================================
    if not hasattr(cf2_control_fn, "initialized"):
        cf2_control_fn.initialized       = True
        cf2_control_fn.show_phase        = 0
        cf2_control_fn.phase_start_clock = clock
        cf2_control_fn.mission_state     = {k: 0           for k in range(1, 6)}
        cf2_control_fn.prev_error        = {k: np.zeros(3) for k in range(1, 6)}
        cf2_control_fn.filt_deriv        = {k: np.zeros(3) for k in range(1, 6)}
        cf2_control_fn.prev_clock        = {k: clock       for k in range(1, 6)}
        cf2_control_fn.last_phase_seen   = {k: 0           for k in range(1, 6)}
        cf2_control_fn.y_assignment       = list(range(5))
        cf2_control_fn.y_assignment_phase = -1

    # ============================================================
    # Default outputs
    # ============================================================
    vx = 0.0;  vy = 0.0;  z_dist = z_hover
    trigger_takeoff = False;  trigger_land = False
    led = (0, 0, 0)

    # ============================================================
    # Helper — PD controller with low-pass filtered derivative
    # ============================================================
    def pd_control(target):
        pos = np.array([robotPose[0], robotPose[1], robotPose[2]])
        err = np.array(target, dtype=float) - pos

        dt = clock - cf2_control_fn.prev_clock[robotNo]
        if dt < 1e-6:
            dt = 0.05

        if cf2_control_fn.last_phase_seen[robotNo] != cf2_control_fn.show_phase:
            cf2_control_fn.prev_error[robotNo]      = err.copy()
            cf2_control_fn.filt_deriv[robotNo]      = np.zeros(3)
            cf2_control_fn.last_phase_seen[robotNo] = cf2_control_fn.show_phase

        raw_d = (err - cf2_control_fn.prev_error[robotNo]) / dt
        alpha = deriv_tau / (deriv_tau + dt)
        d     = alpha * cf2_control_fn.filt_deriv[robotNo] + (1.0 - alpha) * raw_d

        cf2_control_fn.prev_error[robotNo] = err.copy()
        cf2_control_fn.filt_deriv[robotNo] = d.copy()
        cf2_control_fn.prev_clock[robotNo] = clock

        def sat(v, lim): return max(-lim, min(lim, v))

        vx_c = sat(kp_xy * err[0] + kd_xy * d[0], max_vxy_cmd)
        vy_c = sat(kp_xy * err[1] + kd_xy * d[1], max_vxy_cmd)
        vz_c = sat(kp_z  * err[2] + kd_z  * d[2], max_vz_cmd)

        z_c = max(0.1, min(2.5, robotPose[2] + vz_c))
        return vx_c, vy_c, z_c

    # ============================================================
    # Helper — Displacement-Based Formation Control (Q2.6 style)
    # u_i = K_FORM * Σ_{j≠i} [(p_j-p_i) - (t_j-t_i)]  ← shape term
    #      + K_ABS  * (t_i - p_i)                        ← centroid anchor
    # ============================================================
    def displacement_control(targets):
        vx_c = vy_c = vz_c = 0.0
        for j in range(nbCF2):
            if j == i:
                continue
            dx = (cf2_poses[0,j] - robotPose[0]) - (targets[j][0] - targets[i][0])
            dy = (cf2_poses[1,j] - robotPose[1]) - (targets[j][1] - targets[i][1])
            dz = (cf2_poses[2,j] - robotPose[2]) - (targets[j][2] - targets[i][2])
            vx_c += K_FORM * dx
            vy_c += K_FORM * dy
            vz_c += K_FORM * dz
        vx_c += K_ABS * (targets[i][0] - robotPose[0])
        vy_c += K_ABS * (targets[i][1] - robotPose[1])
        vz_c += K_ABS * (targets[i][2] - robotPose[2])

        def sat(v, lim): return max(-lim, min(lim, v))
        vx_c = sat(vx_c, max_vxy_cmd)
        vy_c = sat(vy_c, max_vxy_cmd)
        vz_c = sat(vz_c, max_vz_cmd)
        z_c  = max(0.1, min(2.5, robotPose[2] + vz_c))
        return vx_c, vy_c, z_c

    # ============================================================
    # Helpers — fleet-wide checks (reads cf2_poses snapshot)
    # ============================================================
    def all_above_threshold():
        return all(cf2_poses[2, k] >= z_takeoff_threshold for k in range(nbCF2))

    def all_at(targets):
        for k in range(nbCF2):
            tx, ty, tz = targets[k]
            if math.sqrt((cf2_poses[0,k]-tx)**2 +
                         (cf2_poses[1,k]-ty)**2 +
                         (cf2_poses[2,k]-tz)**2) > ARRIVAL_TOL:
                return False
        return True

    def max_dist_cf2():
        max_d = 0.0
        for a in range(nbCF2):
            for b in range(a + 1, nbCF2):
                d = math.sqrt((cf2_poses[0,a]-cf2_poses[0,b])**2 +
                              (cf2_poses[1,a]-cf2_poses[1,b])**2 +
                              (cf2_poses[2,a]-cf2_poses[2,b])**2)
                if d > max_d:
                    max_d = d
        return max_d

    def min_dist_cf2():
        min_d = math.inf
        for a in range(nbCF2):
            for b in range(a + 1, nbCF2):
                d = math.sqrt((cf2_poses[0,a]-cf2_poses[0,b])**2 +
                              (cf2_poses[1,a]-cf2_poses[1,b])**2 +
                              (cf2_poses[2,a]-cf2_poses[2,b])**2)
                if d < min_d:
                    min_d = d
        return min_d

    # ============================================================
    # Helper — inter-drone repulsion (APF)
    # Force ∝ (1/dist − 1/SAFE_DIST) → grows fast as dist → 0
    # ============================================================
    def repulsion():
        vx_r = vy_r = vz_r = 0.0
        min_d = float('inf')
        pos = np.array([robotPose[0], robotPose[1], robotPose[2]])
        for k in range(nbCF2):
            if k == i:
                continue
            other = np.array([cf2_poses[0, k], cf2_poses[1, k], cf2_poses[2, k]])
            diff  = pos - other
            dist  = np.linalg.norm(diff)
            min_d = min(min_d, dist)
            if 1e-6 < dist < SAFE_DIST:
                unit     = diff / dist
                strength = K_REP * (1.0 / dist - 1.0 / SAFE_DIST)
                vx_r += strength * unit[0]
                vy_r += strength * unit[1]
                vz_r += strength * unit[2]
        return vx_r, vy_r, vz_r, min_d

    def change_phase(new_phase):
        cf2_control_fn.show_phase        = new_phase
        cf2_control_fn.phase_start_clock = clock

    # ============================================================
    # Y-line target assignment — computed once when entering phase 5 or 9.
    # Drones sorted by y-position → no path crossings.
    # ============================================================
    if cf2_control_fn.show_phase in (5, 9) and \
            cf2_control_fn.y_assignment_phase != cf2_control_fn.show_phase:
        y_vals = [cf2_poses[1, k] for k in range(nbCF2)]
        order  = sorted(range(nbCF2), key=lambda k: y_vals[k])
        asgn   = [0] * nbCF2
        for rank, dk in enumerate(order):
            asgn[dk] = rank
        cf2_control_fn.y_assignment       = asgn
        cf2_control_fn.y_assignment_phase = cf2_control_fn.show_phase
    y_assigned = [y_line_targets[cf2_control_fn.y_assignment[k]] for k in range(nbCF2)]

    # ============================================================
    # Phase transitions
    # ============================================================
    phase = cf2_control_fn.show_phase
    t     = clock - cf2_control_fn.phase_start_clock

    if   phase == 0 and all_above_threshold():
        change_phase(1)
    elif phase == 1 and (all_at(penta_targets) or t >= FORMATION_TIMEOUT):
        change_phase(2)
    elif phase == 2 and t >= ROTATION_DURATION:
        change_phase(3)
    elif phase == 3 and (max_dist_cf2() < CONSENSUS_THRESH or t >= CONSENSUS_TIMEOUT):
        change_phase(4)
    elif phase == 4 and (min_dist_cf2() > INVERSE_THRESH or t >= INVERSE_TIMEOUT):
        change_phase(5)
    elif phase == 5 and (all_at(y_assigned) or t >= FORMATION_TIMEOUT):
        change_phase(6)
    elif phase == 6 and (all_at(z_shape_targets) or t >= FORMATION_TIMEOUT):
        change_phase(7)
    elif phase == 7 and (all_at(vert_targets) or t >= FORMATION_TIMEOUT):
        change_phase(8)
    elif phase == 8 and t >= ROTATION_DURATION:
        change_phase(9)
    elif phase == 9 and (all_at(y_assigned) or t >= FORMATION_TIMEOUT):
        change_phase(10)

    # Refresh after possible transition
    phase = cf2_control_fn.show_phase
    t     = clock - cf2_control_fn.phase_start_clock

    # ============================================================
    # Phase 0 — takeoff
    # ============================================================
    if phase == 0:
        z_dist = z_hover
        led    = (0, 0, 255)
        if cf2_control_fn.mission_state[robotNo] == 0:
            if clock >= Time2Takeoff:
                trigger_takeoff = True
            if robotPose[2] >= z_takeoff_threshold:
                cf2_control_fn.mission_state[robotNo] = 1
        vx, vy = 0.0, 0.0

    # ============================================================
    # Phase 1 — form horizontal pentagon
    # ============================================================
    elif phase == 1:
        vx, vy, z_dist = displacement_control(penta_targets)
        led = (255, 100, 0)

    # ============================================================
    # Phase 2 — one full rotation of the pentagon
    # ============================================================
    elif phase == 2:
        angle = penta_angles[i] + 2.0 * math.pi * t / ROTATION_DURATION
        vx, vy, z_dist = pd_control((
            PENTA_R * math.cos(angle),
            PENTA_R * math.sin(angle),
            z_hover))
        led = (255, 165, 0)

    # ============================================================
    # Phase 3 — partial consensus (convergence)
    # Each drone navigates toward drone (i+1) on the directed ring.
    # All drones converge toward a common point.
    # ============================================================
    elif phase == 3:
        next_idx = (i + 1) % nbCF2
        target   = (cf2_poses[0, next_idx], cf2_poses[1, next_idx], z_hover)
        vx, vy, z_dist = pd_control(target)
        led = (255, 0, 255)

    # ============================================================
    # Phase 4 — inverse consensus (divergence)
    # Each drone moves away from drone (i+1) on the same ring.
    # All drones spread out from the center.
    # ============================================================
    elif phase == 4:
        next_idx = (i + 1) % nbCF2
        dx = robotPose[0] - cf2_poses[0, next_idx]
        dy = robotPose[1] - cf2_poses[1, next_idx]
        target   = (robotPose[0] + dx, robotPose[1] + dy, z_hover)
        vx, vy, z_dist = pd_control(target)
        led = (255, 255, 0)

    # ============================================================
    # Phase 5 — straight line on Y axis
    # ============================================================
    elif phase == 5:
        vx, vy, z_dist = pd_control(y_assigned[i])
        led = (255, 60, 60)

    # ============================================================
    # Phase 6 — Z shape in XZ plane (viewed from Y direction)
    # ============================================================
    elif phase == 6:
        vx, vy, z_dist = pd_control(z_shape_targets[i])
        led = (60, 220, 60)

    # ============================================================
    # Phase 7 — vertical pentagon in XZ plane
    # ============================================================
    elif phase == 7:
        vx, vy, z_dist = displacement_control(vert_targets)
        led = (0, 180, 255)

    # ============================================================
    # Phase 8 — vertical pentagon rotating around Z axis (one full turn)
    # x = VERT_R*cos(θ_i)*cos(φ),  y = VERT_R*cos(θ_i)*sin(φ),  z = z_hover + VERT_Z_OFFSET + VERT_R*sin(θ_i)
    # ============================================================
    elif phase == 8:
        phi    = 2.0 * math.pi * t / ROTATION_DURATION
        tx     = VERT_R * math.cos(penta_angles[i]) * math.cos(phi)
        ty     = VERT_R * math.cos(penta_angles[i]) * math.sin(phi)
        tz     = z_hover + VERT_Z_OFFSET + VERT_R * math.sin(penta_angles[i])
        vx, vy, z_dist = pd_control((tx, ty, tz))
        led = (160, 0, 255)

    # ============================================================
    # Phase 9 — straight line on Y axis (again)
    # ============================================================
    elif phase == 9:
        vx, vy, z_dist = pd_control(y_assigned[i])
        led = (255, 255, 60)

    # ============================================================
    # Phase 10 — individual landing
    # ============================================================
    elif phase == 10:
        vx, vy  = 0.0, 0.0
        z_dist  = 0.0
        trigger_land = True
        led = (0, 255, 0)
        cf2_control_fn.mission_state[robotNo] = 2
        if robotPose[2] < 0.15:
            cf2_control_fn.mission_state[robotNo] = 3

    # ============================================================
    # Safety stop
    # ============================================================
    else:
        vx, vy  = 0.0, 0.0
        z_dist  = 0.0
        led = (0, 255, 0)

    # ============================================================
    # Inter-drone collision avoidance — applied to every phase
    #   alpha = 1  →  full navigation  (dist >= SAFE_DIST)
    #   alpha = 0  →  navigation off   (dist <= MIN_DIST)
    # ============================================================
    if not trigger_land:
        vx_r, vy_r, vz_r, min_d = repulsion()
        alpha  = max(0.0, min(1.0, (min_d - MIN_DIST) / (SAFE_DIST - MIN_DIST)))
        vx     = alpha * vx + vx_r
        vy     = alpha * vy + vy_r
        z_dist = max(0.1, min(2.5, z_dist + vz_r))

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
