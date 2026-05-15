#!/usr/bin/python3
"""Simple consensus controllers for TurtleBot3 robots.

The consensus law is:

    velocity_i = gain * (average_position - position_i)

with a small separation term so robots do not intentionally drive into the
same point during real-life tests.
"""

import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import tp_algos


CONSENSUS_GAIN = 0.25
SEPARATION_GAIN = 0.08
STOP_RADIUS = 0.05
SEPARATION_RADIUS = 0.35
MAX_SPEED_BURGER = 0.10
MAX_SPEED_WAFFLE = 0.12
LIDAR_STOP_DISTANCE = 0.22
STOP_BEFORE_COLLISION_RADIUS = 0.45
STOP_TIME = 1.0
CONSENSUS_STOP_UNTIL = 0.0


def _safe_pose_array(poses):
    if poses is None or len(poses) == 0:
        return np.zeros((3, 0))
    return np.asarray(poses)


def _turtlebot_positions(tb3B_poses, tb3W_poses):
    tb3B_poses = _safe_pose_array(tb3B_poses)
    tb3W_poses = _safe_pose_array(tb3W_poses)
    parts = []
    if tb3B_poses.shape[1] > 0:
        parts.append(tb3B_poses[:2, :].T)
    if tb3W_poses.shape[1] > 0:
        parts.append(tb3W_poses[:2, :].T)
    if not parts:
        return np.zeros((0, 2))
    return np.vstack(parts)


def _lidar_too_close(lidar_scan):
    if lidar_scan is None or len(lidar_scan) == 0:
        return False
    ranges = [r for r in lidar_scan if r is not None and math.isfinite(r) and r > 0.0]
    return bool(ranges) and min(ranges) < LIDAR_STOP_DISTANCE


def _robots_too_close(all_positions):
    for i in range(len(all_positions)):
        for j in range(i + 1, len(all_positions)):
            dist = np.linalg.norm(all_positions[i] - all_positions[j])
            if dist < STOP_BEFORE_COLLISION_RADIUS:
                return True
    return False


def _stop_timer_active(all_positions, clock):
    global CONSENSUS_STOP_UNTIL
    if _robots_too_close(all_positions):
        CONSENSUS_STOP_UNTIL = max(CONSENSUS_STOP_UNTIL, clock + STOP_TIME)
    return clock < CONSENSUS_STOP_UNTIL


def _limit_speed(vx, vy, max_speed):
    speed = math.hypot(vx, vy)
    if speed <= max_speed:
        return float(vx), float(vy)
    return float(vx * max_speed / speed), float(vy * max_speed / speed)


def _consensus_velocity(robotPose, all_positions):
    if len(all_positions) <= 1:
        return 0.0, 0.0

    own = np.asarray(robotPose[:2], dtype=float)
    centroid = np.mean(all_positions, axis=0)
    error = centroid - own

    if np.linalg.norm(error) < STOP_RADIUS:
        vx = 0.0
        vy = 0.0
    else:
        vx = CONSENSUS_GAIN * error[0]
        vy = CONSENSUS_GAIN * error[1]

    for other in all_positions:
        delta = own - other
        dist = np.linalg.norm(delta)
        if dist < 1e-6 or dist >= SEPARATION_RADIUS:
            continue
        push = (SEPARATION_RADIUS - dist) / SEPARATION_RADIUS
        vx += SEPARATION_GAIN * push * delta[0] / dist
        vy += SEPARATION_GAIN * push * delta[1] / dist

    return vx, vy


def tb3B_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
    all_positions = _turtlebot_positions(tb3B_poses, tb3W_poses)
    if _lidar_too_close(lidar_scan) or _stop_timer_active(all_positions, clock):
        return 0.0, 0.0

    vx, vy = _consensus_velocity(robotPose, all_positions)
    return _limit_speed(vx, vy, MAX_SPEED_BURGER)


def tb3W_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
    all_positions = _turtlebot_positions(tb3B_poses, tb3W_poses)
    if _lidar_too_close(lidar_scan) or _stop_timer_active(all_positions, clock):
        return 0.0, 0.0

    vx, vy = _consensus_velocity(robotPose, all_positions)
    return _limit_speed(vx, vy, MAX_SPEED_WAFFLE)


def tb3B_controller(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
    return tb3B_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock)


def tb3W_controller(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock):
    return tb3W_control_fn(robotNo, robotPose, tb3B_poses, tb3W_poses, rmtt_poses, cf2_poses, rmep_poses, obstacle_pose, obstacle_size, lidar_scan, clock)


def install_into_tp_algos():
    """Patch tp_algos so existing launch/simulator code uses this consensus law."""
    tp_algos.tb3B_control_fn = tb3B_control_fn
    tp_algos.tb3W_control_fn = tb3W_control_fn
    tp_algos.tb3B_controller = tb3B_controller
    tp_algos.tb3W_controller = tb3W_controller


rmtt_control_fn = tp_algos.rmtt_control_fn
cf2_control_fn = tp_algos.cf2_control_fn
rmep_control_fn = tp_algos.rmep_control_fn
rmtt_controller = tp_algos.rmtt_controller
cf2_controller = tp_algos.cf2_controller
rmep_controller = tp_algos.rmep_controller
