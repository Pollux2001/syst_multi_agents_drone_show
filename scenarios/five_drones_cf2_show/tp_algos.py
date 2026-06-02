#!/usr/bin/python3
'''
    CentraleSupelec TP 2A/3A
    (all variables in SI unit)

    Scenario: five_drones_cf2_show  —  5 Crazyflie 2 drones
    Phase 0  : takeoff to 1 m  (drones start in pentagon on the ground)
    Phase 1  : one full 360 rotation of the horizontal pentagon
    Phase 2  : partial consensus   (ring, drones converge)
    Phase 3  : inverse consensus   (same ring, drones diverge)
    Phase 4  : split — 3 drones to y=+Y_SPLIT, 2 to y=-Y_SPLIT
    Phase 5  : rise to individual vertical-pentagon heights  (Z only)
    Phase 6  : move to XY positions of vertical pentagon    (XY only)
    Phase 7  : vertical pentagon rotates one full 360 turn  (Z fixed)
    Phase 8  : split back to side positions                 (XY only)
    Phase 9  : descend to 1 m                               (Z only)
    Phase 10 : form center line at 1 m
    Phase 11 : land
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
    # Show parameters  —  modify these to tune the choreography
    # ============================================================
    z_hover             = 1.0
    z_takeoff_threshold = 0.9 * z_hover

    PENTA_R    = 1.2    # horizontal pentagon radius (m)

    VERT_Z_MAX = 2.5    # height of the highest drone in vertical pentagon (m)
    VERT_Z_MIN = 0.5    # height of the lowest  drone in vertical pentagon (m)
    _sin72     = math.sin(2.0 * math.pi / 5)            # sin(72 deg) ~ 0.9511
    z_center   = (VERT_Z_MAX + VERT_Z_MIN) / 2.0        # 1.5 m
    VERT_R     = (VERT_Z_MAX - VERT_Z_MIN) / (2.0 * _sin72)  # ~ 1.051 m

    T_ROTATE    = 14.0   # duration of one full 360 rotation (s)
    T_CONSENSUS = 18.0   # max duration of consensus phase (s)
    T_INVERSE   = 15.0   # max duration of inverse consensus phase (s)
    T_FORMATION = 25.0   # max time to settle into any formation (s)

    Y_SPLIT       = 2.5   # |y| of the two split lines (m)
    SPLIT_X       = 1.0   # x spacing between drones within a split group (m)
    LINE_Y_SPACING = 0.9  # spacing between drones in the center line (m)

    ARRIVAL_TOL   = 0.15  # 3-D position tolerance (m)
    ARRIVAL_TOL_Z = 0.12  # altitude-only tolerance for Z-only phases (m)

    CONSENSUS_THRESH = 0.5   # max pairwise dist to exit consensus (m)
    INVERSE_THRESH   = 1.3   # min pairwise dist to exit inverse consensus (m)

    MIN_DIST  = 0.5
    SAFE_DIST = 0.95
    K_REP     = 0.55

    ARENA_X     = 2.5    # arena half-width in X (m)
    ARENA_Y     = 4.5    # arena half-width in Y (m)
    WALL_MARGIN = 0.8    # wall repulsion activation distance (m)
    K_WALL      = 0.8    # wall repulsion gain

    CMD_RATE = 0.5   # minimum interval between Z command updates (s)

    K_FORM = 0.5
    K_ABS  = 0.35

    kp_xy  = 1.2;  kd_xy = 0.20
    kp_z   = 1.2;  kd_z  = 0.18
    deriv_tau   = 0.35
    max_vxy_cmd = 0.55
    max_vz_cmd  = 0.50

    # ============================================================
    # Formation targets
    # ============================================================
    penta_angles = [k * 2.0 * math.pi / 5 for k in range(5)]

    # Horizontal pentagon at z_hover (phase 1 rotation reference)
    penta_targets = [
        (PENTA_R * math.cos(penta_angles[k]),
         PENTA_R * math.sin(penta_angles[k]),
         z_hover)
        for k in range(5)
    ]

    # Vertical pentagon: all at y=0, heights derived from VERT_Z_MAX / VERT_Z_MIN
    vert_targets = [
        (VERT_R * math.cos(penta_angles[k]),
         0.0,
         z_center + VERT_R * math.sin(penta_angles[k]))
        for k in range(5)
    ]

    # Split positions — drones 0,1,2 → side A (y=+Y_SPLIT); 3,4 → side B (y=-Y_SPLIT)
    split_positions = [
        (-SPLIT_X,        +Y_SPLIT, z_hover),
        ( 0.0,            +Y_SPLIT, z_hover),
        (+SPLIT_X,        +Y_SPLIT, z_hover),
        (-SPLIT_X / 2.0,  -Y_SPLIT, z_hover),
        (+SPLIT_X / 2.0,  -Y_SPLIT, z_hover),
    ]

    # Rise targets: same XY as split, Z = individual vertical-pentagon height
    rise_targets = [
        (split_positions[k][0], split_positions[k][1], vert_targets[k][2])
        for k in range(5)
    ]

    # Center-line targets (phase 10)
    line_targets = [
        (0.0, (-2 + k) * LINE_Y_SPACING, z_hover)
        for k in range(5)
    ]

    # ============================================================
    # Persistent state — initialised once for the whole simulation
    # ============================================================
    if not hasattr(cf2_control_fn, "initialized"):
        cf2_control_fn.initialized        = True
        cf2_control_fn.show_phase         = 0
        cf2_control_fn.phase_start_clock  = clock
        cf2_control_fn.mission_state      = {k: 0           for k in range(1, 6)}
        cf2_control_fn.prev_error         = {k: np.zeros(3) for k in range(1, 6)}
        cf2_control_fn.filt_deriv         = {k: np.zeros(3) for k in range(1, 6)}
        cf2_control_fn.prev_clock         = {k: clock       for k in range(1, 6)}
        cf2_control_fn.last_phase_seen    = {k: 0           for k in range(1, 6)}
        cf2_control_fn.y_assignment       = list(range(5))
        cf2_control_fn.y_assignment_phase = -1
        cf2_control_fn.last_z_clock       = {k: -999.0      for k in range(1, 6)}
        cf2_control_fn.last_z_dist        = {k: z_hover     for k in range(1, 6)}

    # ============================================================
    # Default outputs
    # ============================================================
    vx = 0.0;  vy = 0.0;  z_dist = z_hover
    trigger_takeoff = False;  trigger_land = False
    led = (0, 0, 0)

    # ============================================================
    # PD helpers
    # ============================================================
    def _pd_update(err3):
        dt = max(clock - cf2_control_fn.prev_clock[robotNo], 1e-6)
        if cf2_control_fn.last_phase_seen[robotNo] != cf2_control_fn.show_phase:
            cf2_control_fn.prev_error[robotNo]      = err3.copy()
            cf2_control_fn.filt_deriv[robotNo]      = np.zeros(3)
            cf2_control_fn.last_phase_seen[robotNo] = cf2_control_fn.show_phase
        raw_d = (err3 - cf2_control_fn.prev_error[robotNo]) / dt
        alpha = deriv_tau / (deriv_tau + dt)
        d = alpha * cf2_control_fn.filt_deriv[robotNo] + (1.0 - alpha) * raw_d
        cf2_control_fn.prev_error[robotNo] = err3.copy()
        cf2_control_fn.filt_deriv[robotNo] = d.copy()
        cf2_control_fn.prev_clock[robotNo] = clock
        return d

    def sat(v, lim): return max(-lim, min(lim, v))

    def pd_control(target):
        err = np.array(target, dtype=float) - np.array([robotPose[0], robotPose[1], robotPose[2]])
        d   = _pd_update(err)
        vx_c = sat(kp_xy * err[0] + kd_xy * d[0], max_vxy_cmd)
        vy_c = sat(kp_xy * err[1] + kd_xy * d[1], max_vxy_cmd)
        vz_c = sat(kp_z  * err[2] + kd_z  * d[2], max_vz_cmd)
        z_c  = max(0.1, min(2.8, robotPose[2] + vz_c))
        return vx_c, vy_c, z_c

    def pd_control_xy(tx, ty):
        err = np.array([tx - robotPose[0], ty - robotPose[1], 0.0])
        d   = _pd_update(err)
        vx_c = sat(kp_xy * err[0] + kd_xy * d[0], max_vxy_cmd)
        vy_c = sat(kp_xy * err[1] + kd_xy * d[1], max_vxy_cmd)
        return vx_c, vy_c

    # ============================================================
    # Fleet-wide checks (read cf2_poses snapshot)
    # ============================================================
    def all_above_threshold():
        return all(cf2_poses[2, k] >= z_takeoff_threshold for k in range(nbCF2))

    def all_at(targets):
        for k in range(nbCF2):
            if math.sqrt((cf2_poses[0,k]-targets[k][0])**2 +
                         (cf2_poses[1,k]-targets[k][1])**2 +
                         (cf2_poses[2,k]-targets[k][2])**2) > ARRIVAL_TOL:
                return False
        return True

    def all_at_z(targets):
        return all(abs(cf2_poses[2, k] - targets[k][2]) <= ARRIVAL_TOL_Z
                   for k in range(nbCF2))

    def all_at_xy(targets):
        for k in range(nbCF2):
            if math.sqrt((cf2_poses[0,k]-targets[k][0])**2 +
                         (cf2_poses[1,k]-targets[k][1])**2) > ARRIVAL_TOL:
                return False
        return True

    def max_dist_cf2():
        d = 0.0
        for a in range(nbCF2):
            for b in range(a + 1, nbCF2):
                d = max(d, math.sqrt((cf2_poses[0,a]-cf2_poses[0,b])**2 +
                                     (cf2_poses[1,a]-cf2_poses[1,b])**2 +
                                     (cf2_poses[2,a]-cf2_poses[2,b])**2))
        return d

    def min_dist_cf2():
        d = math.inf
        for a in range(nbCF2):
            for b in range(a + 1, nbCF2):
                d = min(d, math.sqrt((cf2_poses[0,a]-cf2_poses[0,b])**2 +
                                     (cf2_poses[1,a]-cf2_poses[1,b])**2 +
                                     (cf2_poses[2,a]-cf2_poses[2,b])**2))
        return d

    # ============================================================
    # APF inter-drone repulsion
    # Force grows as dist -> 0: strength = K_REP * (1/dist - 1/SAFE_DIST)
    # ============================================================
    def repulsion():
        vx_r = vy_r = vz_r = 0.0
        min_d = float('inf')
        pos = np.array([robotPose[0], robotPose[1], robotPose[2]])
        for k in range(nbCF2):
            if k == i:
                continue
            other = np.array([cf2_poses[0,k], cf2_poses[1,k], cf2_poses[2,k]])
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

    def wall_repulsion():
        """APF-style repulsion from arena boundaries — keeps drones inside the voliere."""
        vx_w = vy_w = 0.0
        x, y = robotPose[0], robotPose[1]
        d_xn = x - (-ARENA_X);  d_xp = ARENA_X - x   # dist to left / right wall
        d_yn = y - (-ARENA_Y);  d_yp = ARENA_Y - y   # dist to bottom / top wall
        for d, sign, acc in [(d_xn, +1, 'x'), (d_xp, -1, 'x'),
                             (d_yn, +1, 'y'), (d_yp, -1, 'y')]:
            d = max(d, 1e-3)
            if d < WALL_MARGIN:
                f = sign * K_WALL * (1.0 / d - 1.0 / WALL_MARGIN)
                if acc == 'x': vx_w += f
                else:          vy_w += f
        return vx_w, vy_w

    def change_phase(p):
        cf2_control_fn.show_phase        = p
        cf2_control_fn.phase_start_clock = clock

    # ============================================================
    # Y-line assignment — computed once when entering phase 10.
    # Sort drones by y-position to avoid path crossings.
    # ============================================================
    if cf2_control_fn.show_phase == 10 and cf2_control_fn.y_assignment_phase != 10:
        y_vals = [cf2_poses[1, k] for k in range(nbCF2)]
        order  = sorted(range(nbCF2), key=lambda k: y_vals[k])
        asgn   = [0] * nbCF2
        for rank, dk in enumerate(order):
            asgn[dk] = rank
        cf2_control_fn.y_assignment       = asgn
        cf2_control_fn.y_assignment_phase = 10
    line_assigned = [line_targets[cf2_control_fn.y_assignment[k]] for k in range(5)]

    # ============================================================
    # Phase transitions (evaluated every call)
    # ============================================================
    phase = cf2_control_fn.show_phase
    t     = clock - cf2_control_fn.phase_start_clock

    if   phase == 0  and all_above_threshold():                                     change_phase(1)
    elif phase == 1  and t >= T_ROTATE:                                             change_phase(2)
    elif phase == 2  and (max_dist_cf2() < CONSENSUS_THRESH or t >= T_CONSENSUS):  change_phase(3)
    elif phase == 3  and (min_dist_cf2() > INVERSE_THRESH   or t >= T_INVERSE):    change_phase(4)
    elif phase == 4  and (all_at(split_positions) or t >= T_FORMATION):            change_phase(5)
    elif phase == 5  and (all_at_z(rise_targets)  or t >= T_FORMATION):            change_phase(6)
    elif phase == 6  and (all_at_xy(vert_targets) or t >= T_FORMATION):            change_phase(7)
    elif phase == 7  and t >= T_ROTATE:                                             change_phase(8)
    elif phase == 8  and (all_at_xy(split_positions) or t >= T_FORMATION):         change_phase(9)
    elif phase == 9  and (all_at_z(split_positions)  or t >= T_FORMATION):         change_phase(10)
    elif phase == 10 and (all_at(line_assigned) or t >= T_FORMATION):              change_phase(11)

    # Refresh after possible transition
    phase = cf2_control_fn.show_phase
    t     = clock - cf2_control_fn.phase_start_clock

    # ============================================================
    # Phase 0 — takeoff
    # ============================================================
    if phase == 0:
        led    = (0, 0, 255)
        z_dist = z_hover
        vx, vy = 0.0, 0.0
        if cf2_control_fn.mission_state[robotNo] == 0:
            if clock >= Time2Takeoff:
                trigger_takeoff = True
            if robotPose[2] >= z_takeoff_threshold:
                cf2_control_fn.mission_state[robotNo] = 1

    # ============================================================
    # Phase 1 — full 360 rotation of the horizontal pentagon
    # Drones already start at pentagon positions, so no forming step needed.
    # ============================================================
    elif phase == 1:
        angle  = penta_angles[i] + 2.0 * math.pi * t / T_ROTATE
        vx, vy, z_dist = pd_control((PENTA_R * math.cos(angle),
                                      PENTA_R * math.sin(angle),
                                      z_hover))
        led = (255, 165, 0)

    # ============================================================
    # Phase 2 — partial consensus (convergence)
    # Each drone navigates toward drone (i+1) on the directed ring.
    # ============================================================
    elif phase == 2:
        next_idx = (i + 1) % nbCF2
        vx, vy, z_dist = pd_control((cf2_poses[0, next_idx],
                                      cf2_poses[1, next_idx],
                                      z_hover))
        led = (255, 0, 255)

    # ============================================================
    # Phase 3 — inverse consensus (divergence)
    # Each drone moves away from drone (i+1) with bounded velocity.
    # ============================================================
    elif phase == 3:
        next_idx = (i + 1) % nbCF2
        dx   = robotPose[0] - cf2_poses[0, next_idx]
        dy   = robotPose[1] - cf2_poses[1, next_idx]
        dist = math.sqrt(dx**2 + dy**2) + 1e-6
        vx, vy = 0.3 * dx / dist, 0.3 * dy / dist
        z_dist = z_hover
        led = (255, 255, 0)

    # ============================================================
    # Phase 4 — split: 3 drones to side A, 2 to side B (full 3-D PD)
    # ============================================================
    elif phase == 4:
        vx, vy, z_dist = pd_control(split_positions[i])
        led = (0, 220, 80)

    # ============================================================
    # Phase 5 — rise to individual vertical-pentagon heights (Z only)
    # vx = vy = 0; only a single altitude command is sent per CMD_RATE.
    # ============================================================
    elif phase == 5:
        vx, vy = 0.0, 0.0
        z_dist  = rise_targets[i][2]
        led = (0, 180, 255)

    # ============================================================
    # Phase 6 — move to XY positions of vertical pentagon (XY only)
    # Z stays fixed at the height reached in phase 5.
    # ============================================================
    elif phase == 6:
        vx, vy = pd_control_xy(vert_targets[i][0], vert_targets[i][1])
        z_dist  = vert_targets[i][2]
        led = (0, 100, 255)

    # ============================================================
    # Phase 7 — vertical pentagon rotates one full 360 turn (Z fixed)
    # x = VERT_R*cos(theta_i)*cos(phi),  y = VERT_R*cos(theta_i)*sin(phi)
    # z stays at vert_targets[i][2] — never updated.
    # ============================================================
    elif phase == 7:
        phi    = 2.0 * math.pi * t / T_ROTATE
        vx, vy = pd_control_xy(VERT_R * math.cos(penta_angles[i]) * math.cos(phi),
                                VERT_R * math.cos(penta_angles[i]) * math.sin(phi))
        z_dist  = vert_targets[i][2]
        led = (160, 0, 255)

    # ============================================================
    # Phase 8 — split back to XY of split positions (XY only, Z fixed)
    # ============================================================
    elif phase == 8:
        vx, vy = pd_control_xy(split_positions[i][0], split_positions[i][1])
        z_dist  = vert_targets[i][2]
        led = (255, 255, 60)

    # ============================================================
    # Phase 9 — descend to z_hover (Z only, XY fixed at split positions)
    # ============================================================
    elif phase == 9:
        vx, vy = 0.0, 0.0
        z_dist  = z_hover
        led = (255, 120, 0)

    # ============================================================
    # Phase 10 — form center line at z_hover
    # ============================================================
    elif phase == 10:
        vx, vy, z_dist = pd_control(line_assigned[i])
        led = (255, 60, 60)

    # ============================================================
    # Phase 11 — land
    # ============================================================
    elif phase == 11:
        vx, vy  = 0.0, 0.0
        z_dist  = 0.0
        trigger_land = True
        led = (0, 255, 0)
        if robotPose[2] < 0.15:
            cf2_control_fn.mission_state[robotNo] = 3

    else:
        vx, vy  = 0.0, 0.0
        z_dist  = 0.0
        led = (0, 255, 0)

    # ============================================================
    # APF repulsion — applied every phase
    # During XY-only phases (6,7,8): Z component of APF is suppressed
    # so the commanded height is never disturbed.
    # alpha = 1 → full navigation (dist >= SAFE_DIST)
    # alpha = 0 → navigation off  (dist <= MIN_DIST)
    # ============================================================
    if not trigger_land:
        vx_r, vy_r, vz_r, min_d = repulsion()
        vx_w, vy_w = wall_repulsion()
        alpha  = max(0.0, min(1.0, (min_d - MIN_DIST) / (SAFE_DIST - MIN_DIST)))
        vx     = alpha * vx + vx_r + vx_w
        vy     = alpha * vy + vy_r + vy_w
        if phase not in (6, 7, 8):
            z_dist = max(0.1, min(2.8, z_dist + vz_r))

    # ============================================================
    # Z command rate limiting — send new altitude at most every CMD_RATE s.
    # vx / vy are always fresh (updated every call for smooth motion).
    # ============================================================
    if not trigger_land:
        if clock - cf2_control_fn.last_z_clock[robotNo] >= CMD_RATE:
            cf2_control_fn.last_z_clock[robotNo] = clock
            cf2_control_fn.last_z_dist[robotNo]  = z_dist
        else:
            z_dist = cf2_control_fn.last_z_dist[robotNo]

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
