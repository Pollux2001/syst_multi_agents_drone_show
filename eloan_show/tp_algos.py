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
# max useable numbers of drones = 6
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
    z_1 = 0.80
    z_2 = 1.20
    z_3 = 1.60
    altitude_targets = np.array([z_1, z_1, z_2, z_2, z_3, z_3])
    z_takeoff_threshold_ratio = 0.9

    # The two drones of each layer are opposite ends of one spoke.
    # Viewed from above, all six starting locations make a wide hexagon.
    safe_radius = 1.20
    base_angles = np.array([
        0.0, math.pi,
        math.pi / 3.0, 4.0 * math.pi / 3.0,
        2.0 * math.pi / 3.0, 5.0 * math.pi / 3.0
    ])
    initial_positions = np.array([
        [
            safe_radius * math.cos(angle),
            safe_radius * math.sin(angle),
            altitude_targets[k]
        ]
        for k, angle in enumerate(base_angles)
    ])

    # Durations of the show phases
    carousel_duration = 14.0
    bloom_duration = 14.0
    weave_duration = 14.0
    crown_duration = 16.0
    finale_duration = 14.0
    return_duration_max = 12.0

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
        # 1 = layered carousel
        # 2 = blooming star
        # 3 = ribbon weave
        # 4 = travelling crown
        # 5 = double-spin finale
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
    pair_index = i // 2

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

    def layered_target(radius, angle_offset, center_xy=None):
        if center_xy is None:
            center_xy = np.zeros(2)
        angle = base_angles[i] + angle_offset
        return np.array([
            center_xy[0] + radius * math.cos(angle),
            center_xy[1] + radius * math.sin(angle),
            altitude_targets[i]
        ])

    def all_drones_above_takeoff_threshold():
        for a in range(nbCF2):
            if cf2_poses[2, a] < z_takeoff_threshold_ratio * altitude_targets[a]:
                return False
        return True

    def all_drones_close_to_initial_positions():
        for a in range(nbCF2):
            dx = cf2_poses[0, a] - initial_positions[a, 0]
            dy = cf2_poses[1, a] - initial_positions[a, 1]
            dz = cf2_poses[2, a] - altitude_targets[a]
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
        if phase_time >= carousel_duration:
            change_phase(2)

    elif phase == 2:
        if phase_time >= bloom_duration:
            change_phase(3)

    elif phase == 3:
        if phase_time >= weave_duration:
            change_phase(4)

    elif phase == 4:
        if phase_time >= crown_duration:
            change_phase(5)

    elif phase == 5:
        if phase_time >= finale_duration:
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
        z_dist = altitude_targets[i]
        led = (0, 0, 255)

        if cf2_control_fn.mission_state[robotNo] == 0:
            trigger_takeoff = True

            if robotPose[2] > z_takeoff_threshold_ratio * altitude_targets[i]:
                cf2_control_fn.mission_state[robotNo] = 1

        vx = 0.0
        vy = 0.0

    # ========================================================
    # Phase 1: layered carousel
    # Each altitude pair spins on opposite ends of the same wide diameter.
    # ========================================================

    elif phase == 1:
        cf2_control_fn.mission_state[robotNo] = 1

        turn = 2.0 * math.pi * phase_time / carousel_duration
        target = layered_target(safe_radius, turn)
        vx, vy, z_dist = pd_position_control(target)
        led = (255, 120, 0)

    # ========================================================
    # Phase 2: blooming star
    # The stacked spokes expand and contract while completing one turn.
    # ========================================================

    elif phase == 2:
        turn = 2.0 * math.pi * phase_time / bloom_duration
        pulse = math.sin(turn) ** 2
        radius = safe_radius + (0.16 + 0.08 * pair_index) * pulse
        layer_twist = (pair_index - 1) * 0.10 * math.sin(2.0 * turn)
        target = layered_target(radius, turn + layer_twist)
        vx, vy, z_dist = pd_position_control(target)
        led = (255, 0, 255)

    # ========================================================
    # Phase 3: ribbon weave
    # The three separated pair axes gently fan apart and reunite.
    # ========================================================

    elif phase == 3:
        turn = 2.0 * math.pi * phase_time / weave_duration
        layer_twist = (pair_index - 1) * 0.14 * math.sin(2.0 * turn)
        radius = safe_radius + 0.10 * (1.0 - math.cos(turn))
        target = layered_target(radius, turn + layer_twist)
        vx, vy, z_dist = pd_position_control(target)
        led = (255, 255, 0)

    # ========================================================
    # Phase 4: travelling crown
    # The whole layered star travels in a circle without collapsing inward.
    # ========================================================

    elif phase == 4:
        turn = 2.0 * math.pi * phase_time / crown_duration
        center_xy = np.array([
            0.38 * (math.cos(turn) - 1.0),
            0.38 * math.sin(turn)
        ])
        radius = safe_radius + 0.12 * math.sin(turn) ** 2
        target = layered_target(radius, turn, center_xy)
        vx, vy, z_dist = pd_position_control(target)
        led = (0, 255, 255)

    # ========================================================
    # Phase 5: double-spin finale
    # A bright final star pulse performs two turns before returning home.
    # ========================================================

    elif phase == 5:
        turn = 2.0 * math.pi * phase_time / finale_duration
        radius = safe_radius + 0.26 * math.sin(turn) ** 2
        target = layered_target(radius, 2.0 * turn)
        vx, vy, z_dist = pd_position_control(target)
        led = (0, 180, 255)

    # ========================================================
    # Phase 6: return to initial positions
    # ========================================================

    elif phase == 6:
        target = np.array([
            initial_positions[i, 0],
            initial_positions[i, 1],
            altitude_targets[i]
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
