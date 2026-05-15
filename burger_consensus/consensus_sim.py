import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

import burger_consensus.consensus as tp_algos


DT = 0.05
MAX_V_TB3B = 0.15
MAX_V_TB3W = 0.18
MAX_W_TB3 = 2.84
RAD_TB3B = 0.08
RAD_TB3W = 0.18
X_MIN, X_MAX = -2.5, 2.5
Y_MIN, Y_MAX = -4.5, 4.5
Z_MIN, Z_MAX = 0.0, 3.5
OBSTACLE_SIZE = [[1.0, 0.5, 2.5], [0.5, 1.1, 2.5]]
OBSTACLE_POSE = [[0.0, 0.0, 0.0], [3.0, -3.0, 0.0]]


def make_starting_poses(count, radius=1.7, heading_offset=0.0):
    poses = []
    if count <= 0:
        return poses
    for i in range(count):
        angle = 2 * np.pi * i / count
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        theta = angle + np.pi + heading_offset
        poses.append([float(x), float(y), float(theta)])
    return poses


def unicycle_kinematics(vx, vy, theta, max_v, max_w):
    v_mag = np.hypot(vx, vy)
    if v_mag < 1e-3:
        return 0.0, 0.0

    theta_des = np.arctan2(vy, vx)
    e_theta = np.arctan2(np.sin(theta_des - theta), np.cos(theta_des - theta))
    wz = np.clip(2.5 * e_theta, -max_w, max_w)
    v = min(v_mag * max(0.0, np.cos(e_theta)), max_v)
    return v, wz


def draw_glob(ax, x, y, z, radius, color):
    u, v = np.mgrid[0:2*np.pi:12j, 0:np.pi:8j]
    sphere_x = x + radius * np.cos(u) * np.sin(v)
    sphere_y = y + radius * np.sin(u) * np.sin(v)
    sphere_z = z + radius * np.cos(v)
    return ax.plot_wireframe(sphere_x, sphere_y, sphere_z, color=color, alpha=0.15)


def check_boundary_collision(cx, cy, r):
    return cx - r < X_MIN or cx + r > X_MAX or cy - r < Y_MIN or cy + r > Y_MAX


def check_obstacle_collision(cx, cy, cz, r, obs_poses, obs_sizes):
    for i in range(obs_poses.shape[1]):
        ox, oy, _ = obs_poses[:, i]
        dx, dy, dz = obs_sizes[:, i]
        closest_x = max(ox - dx / 2, min(cx, ox + dx / 2))
        closest_y = max(oy - dy / 2, min(cy, oy + dy / 2))
        closest_z = max(0, min(cz, dz))
        if (closest_x - cx)**2 + (closest_y - cy)**2 + (closest_z - cz)**2 < r**2:
            return True
    return False


def check_robot_collisions(all_robots, obs_poses, obs_sizes):
    collision_states = {robot[0]: False for robot in all_robots}

    for robot in all_robots:
        name, x, y, z, radius = robot[:5]
        if check_boundary_collision(x, y, radius) or check_obstacle_collision(x, y, z, radius, obs_poses, obs_sizes):
            collision_states[name] = True

    for i in range(len(all_robots)):
        for j in range(i + 1, len(all_robots)):
            name1, x1, y1, z1, r1 = all_robots[i][:5]
            name2, x2, y2, z2, r2 = all_robots[j][:5]
            dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
            if dist < r1 + r2:
                collision_states[name1] = True
                collision_states[name2] = True

    return collision_states


def build_sim(nb_tb3b, nb_tb3w):
    tb3B_pose = make_starting_poses(nb_tb3b, radius=1.8)
    tb3W_pose = make_starting_poses(nb_tb3w, radius=1.2, heading_offset=0.35)

    tb3B_poses = np.array(tb3B_pose).T if nb_tb3b > 0 else np.zeros((3, 0))
    tb3W_poses = np.array(tb3W_pose).T if nb_tb3w > 0 else np.zeros((3, 0))
    empty3 = np.zeros((3, 0))
    obs_poses = np.array(OBSTACLE_POSE).T
    obs_sizes = np.array(OBSTACLE_SIZE).T

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("TurtleBot Consensus Simulation (Standalone Room)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_navigate(False)
    ax.set_xlim([X_MIN, X_MAX])
    ax.set_ylim([Y_MIN, Y_MAX])
    ax.set_zlim([Z_MIN, Z_MAX])
    ax.set_box_aspect((5, 9, 3.5))

    for i in range(obs_poses.shape[1]):
        x, y, _ = obs_poses[:, i]
        dx, dy, dz = obs_sizes[:, i]
        ax.bar3d(x - dx / 2, y - dy / 2, 0, dx, dy, dz, color="k", alpha=0.3)

    tb3B_plots = [
        ax.plot([], [], [], marker="s", color="blue", markersize=9, linestyle="", label="TB3 Burger" if i == 0 else "")[0]
        for i in range(nb_tb3b)
    ]
    tb3W_plots = [
        ax.plot([], [], [], marker="D", color="cyan", markersize=9, linestyle="", label="TB3 Waffle" if i == 0 else "")[0]
        for i in range(nb_tb3w)
    ]
    if nb_tb3b + nb_tb3w > 0:
        ax.legend(loc="upper right", framealpha=0.9, title="Robots")

    tb3B_globs = [None] * nb_tb3b
    tb3W_globs = [None] * nb_tb3w
    clock_time = 0.0

    def update(frame):
        nonlocal clock_time

        tb3B_snap = tb3B_poses.copy() if nb_tb3b > 0 else tb3B_poses
        tb3W_snap = tb3W_poses.copy() if nb_tb3w > 0 else tb3W_poses

        for i in range(nb_tb3b):
            pose = tb3B_poses[:, i]
            vx, vy = tp_algos.tb3B_controller(
                i + 1, pose.copy(), tb3B_snap, tb3W_snap, empty3, empty3,
                empty3, obs_poses, obs_sizes, [], clock_time
            )
            v, wz = unicycle_kinematics(vx, vy, pose[2], MAX_V_TB3B, MAX_W_TB3)
            tb3B_poses[0, i] += v * np.cos(pose[2]) * DT
            tb3B_poses[1, i] += v * np.sin(pose[2]) * DT
            tb3B_poses[2, i] += wz * DT

        for i in range(nb_tb3w):
            pose = tb3W_poses[:, i]
            vx, vy = tp_algos.tb3W_controller(
                i + 1, pose.copy(), tb3B_snap, tb3W_snap, empty3, empty3,
                empty3, obs_poses, obs_sizes, [], clock_time
            )
            v, wz = unicycle_kinematics(vx, vy, pose[2], MAX_V_TB3W, MAX_W_TB3)
            tb3W_poses[0, i] += v * np.cos(pose[2]) * DT
            tb3W_poses[1, i] += v * np.sin(pose[2]) * DT
            tb3W_poses[2, i] += wz * DT

        all_robots = []
        for i in range(nb_tb3b):
            all_robots.append([
                f"TB3B_{i + 1}", tb3B_poses[0, i], tb3B_poses[1, i],
                0.1, RAD_TB3B * 2, "blue", tb3B_globs, i, tb3B_plots
            ])
        for i in range(nb_tb3w):
            all_robots.append([
                f"TB3W_{i + 1}", tb3W_poses[0, i], tb3W_poses[1, i],
                0.15, RAD_TB3W * 2, "cyan", tb3W_globs, i, tb3W_plots
            ])

        collision_states = check_robot_collisions(all_robots, obs_poses, obs_sizes)

        for robot in all_robots:
            name, x, y, z, radius, default_color, glob_list, idx, plot_list = robot
            color = "red" if collision_states[name] else default_color
            plot_list[idx].set_data([x], [y])
            plot_list[idx].set_3d_properties([z])
            plot_list[idx].set_color(color)
            if glob_list[idx]:
                glob_list[idx].remove()
            glob_list[idx] = draw_glob(ax, x, y, z, radius, color)

        clock_time += DT
        return tb3B_plots + tb3W_plots

    ani = animation.FuncAnimation(fig, update, interval=int(DT * 1000), blit=False, cache_frame_data=False)
    return fig, ani


def main():
    parser = argparse.ArgumentParser(description="Run a TurtleBot consensus simulation.")
    parser.add_argument("--burgers", type=int, default=4, help="Number of TurtleBot3 Burger robots.")
    parser.add_argument("--waffles", type=int, default=0, help="Number of TurtleBot3 Waffle robots.")
    args = parser.parse_args()

    fig, ani = build_sim(args.burgers, args.waffles)
    plt.show()


if __name__ == "__main__":
    main()
