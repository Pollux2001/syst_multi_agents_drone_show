import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from concurrent.futures import ThreadPoolExecutor
import tp_algos

# ==========================================
# 1. SIMULATION INPUTS & CONFIGURATION
# ==========================================
dt = 0.05

nbTb3B = 0
nbTb3W = 0
nbRMTT = 0
nbRMEP = 0
nbObstacle = 0

nbCF2 = 5
# Starting positions: line on X axis, on the ground
CF2_pose = [
    [-1.50, 0.0, 0.0],
    [-0.75, 0.0, 0.0],
    [ 0.00, 0.0, 0.0],
    [ 0.75, 0.0, 0.0],
    [ 1.50, 0.0, 0.0],
]

# ==========================================
# 2. HARDWARE SPECS
# ==========================================
MAX_V_CF2  = 0.6
RAD_CF2    = 0.05
CF2_HOVER_Z  = 1.0
TAKEOFF_TIME = 3.0
LANDING_TIME = 3.0
DRONE_POS_NOISE_STD = 0.005

# ==========================================
# 3. INITIALIZE STATE ARRAYS
# ==========================================
tb3B_poses = np.zeros((3, 0))
tb3W_poses = np.zeros((3, 0))
rmtt_poses = np.zeros((3, 0))
cf2_poses  = np.array(CF2_pose).T
rmep_poses = np.zeros((3, 0))
obs_poses  = np.zeros((3, 0))
obs_sizes  = np.zeros((3, 0))

cf2_states = [0] * nbCF2
cf2_timers = [0.0] * nbCF2

# Per-drone LED colors (updated live from controller output)
cf2_colors = [
    (1.0, 0.2, 0.2),   # drone 1 — red
    (1.0, 0.6, 0.0),   # drone 2 — orange
    (0.2, 1.0, 0.2),   # drone 3 — green
    (0.2, 0.2, 1.0),   # drone 4 — blue
    (0.8, 0.0, 0.8),   # drone 5 — purple
]

# ==========================================
# 4. FIGURE & AXES
# ==========================================
fig = plt.figure(figsize=(10, 9))
ax  = fig.add_subplot(111, projection='3d')
ax.set_title("Five Drones CF2 Show — 8-phase choreography")
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_navigate(False)

X_MIN, X_MAX = -2.5, 2.5
Y_MIN, Y_MAX = -2.5, 2.5
Z_MIN, Z_MAX =  0.0, 3.0

ax.set_xlim([X_MIN, X_MAX])
ax.set_ylim([Y_MIN, Y_MAX])
ax.set_zlim([Z_MIN, Z_MAX])
ax.set_box_aspect((5, 5, 3))

marker_colors = ['red', 'orange', 'green', 'blue', 'purple']
cf2_plots = [
    ax.plot([], [], [], marker='*', color=marker_colors[i], linestyle='',
            label=f'CF2 #{i+1}')[0]
    for i in range(nbCF2)
]
ax.legend(loc='upper right', framealpha=0.9, title="Drones")

cf2_globs  = [None] * nbCF2
clock_time = 0.0

# ==========================================
# 5. HELPERS
# ==========================================
def clamp_vel3d(vx, vy, vz, max_v):
    speed = np.linalg.norm([vx, vy, vz])
    return (vx * max_v / speed, vy * max_v / speed, vz * max_v / speed) if speed > max_v else (vx, vy, vz)

def draw_glob(ax, x, y, z, radius, color):
    u, v = np.mgrid[0:2*np.pi:12j, 0:np.pi:8j]
    X = x + radius * np.cos(u) * np.sin(v)
    Y = y + radius * np.sin(u) * np.sin(v)
    Z = z + radius * np.cos(v)
    return ax.plot_wireframe(X, Y, Z, color=color, alpha=0.15)

def check_boundary_collision(cx, cy, cz, r, drone_state=None):
    if cx - r < X_MIN or cx + r > X_MAX: return True
    if cy - r < Y_MIN or cy + r > Y_MAX: return True
    if cz + r > Z_MAX: return True
    if cz - r < Z_MIN and drone_state == 2: return True
    return False

last_log_time = {}

# ==========================================
# 6. ANIMATION LOOP
# ==========================================
executor     = ThreadPoolExecutor(max_workers=8)
task_futures = {}
last_cmds    = {}

def get_async_cmd(robot_type, idx, default_cmd, func, *args):
    key = f"{robot_type}_{idx}"
    if key not in task_futures or task_futures[key] is None:
        task_futures[key] = executor.submit(func, *args)
    if task_futures[key].done():
        try:
            res = task_futures[key].result()
            last_cmds[key] = res
        except Exception as e:
            print(f"[ERROR] Exception in {key} controller: {e}")
        task_futures[key] = None
    return last_cmds.get(key, default_cmd)

def update(frame):
    global clock_time, cf2_colors

    cf2_snap = cf2_poses.copy()

    for i in range(nbCF2):
        pose    = cf2_poses[:, i]
        default = (0.0, 0.0, pose[2], False, False, (0, 0, 0))
        vx, vy, z_dist, trigger_takeoff, trigger_land, led = get_async_cmd(
            'cf2', i, default, tp_algos.cf2_controller,
            i+1, pose.copy(), tb3B_poses, tb3W_poses, rmtt_poses, cf2_snap, rmep_poses,
            obs_poses, obs_sizes, clock_time)

        if led != (0, 0, 0):
            cf2_colors[i] = (led[0] / 255.0, led[1] / 255.0, led[2] / 255.0)

        if cf2_states[i] == 0:
            if trigger_takeoff:
                cf2_states[i] = 1
                cf2_timers[i] = TAKEOFF_TIME
        elif cf2_states[i] == 1:
            cf2_poses[2, i] += (CF2_HOVER_Z / TAKEOFF_TIME) * dt
            cf2_timers[i]   -= dt
            if cf2_timers[i] <= 0:
                cf2_poses[2, i] = CF2_HOVER_Z
                cf2_states[i]   = 2
        elif cf2_states[i] == 2:
            if trigger_land:
                cf2_states[i] = 3
                cf2_timers[i] = LANDING_TIME
            else:
                vz = (z_dist - cf2_poses[2, i])
                vx, vy, vz = clamp_vel3d(vx, vy, vz, MAX_V_CF2)
                cf2_poses[0, i] += vx * dt + np.random.normal(0, DRONE_POS_NOISE_STD)
                cf2_poses[1, i] += vy * dt + np.random.normal(0, DRONE_POS_NOISE_STD)
                cf2_poses[2, i] += vz * dt + np.random.normal(0, DRONE_POS_NOISE_STD)
        elif cf2_states[i] == 3:
            cf2_poses[2, i] -= (CF2_HOVER_Z / LANDING_TIME) * dt
            cf2_timers[i]   -= dt
            if cf2_timers[i] <= 0 or cf2_poses[2, i] <= 0:
                cf2_poses[2, i] = 0.0
                cf2_states[i]   = 0

    for i in range(nbCF2):
        cx, cy, cz = cf2_poses[0, i], cf2_poses[1, i], cf2_poses[2, i]
        r = RAD_CF2 * 2
        collision = check_boundary_collision(cx, cy, cz, r, cf2_states[i])
        if collision:
            log_key = f"CF2_{i+1}_boundary"
            if clock_time - last_log_time.get(log_key, -1.0) >= 1.0:
                print(f"[WARNING] CF2_{i+1} is colliding with the environment boundary!")
                last_log_time[log_key] = clock_time
        cf2_plots[i].set_data([cx], [cy])
        cf2_plots[i].set_3d_properties([cz])
        color = 'red' if collision else cf2_colors[i]
        if cf2_globs[i]: cf2_globs[i].remove()
        cf2_globs[i] = draw_glob(ax, cx, cy, cz, r, color)

    clock_time += dt
    return cf2_plots

ani = animation.FuncAnimation(fig, update, interval=int(dt * 1000), blit=False, cache_frame_data=False)
plt.show()
