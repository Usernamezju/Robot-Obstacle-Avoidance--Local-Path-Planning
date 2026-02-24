from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

from cost_fn import cost_fn


@dataclass
class SimConfig:
    start: Tuple[float, float] = (0.0, 0.0)
    goal: Tuple[float, float] = (15.0, 15.0)
    robot_radius: float = 0.6
    dt: float = 0.1
    horizon: float = 1.5
    max_time: float = 80.0
    world_x: Tuple[float, float] = (-1.0, 16.0)
    world_y: Tuple[float, float] = (-1.0, 16.0)

    vx_range: Tuple[float, float] = (-2.0, 3.0)
    vw_range: Tuple[float, float] = (-1.5, 1.5)
    n_vx: int = 7
    n_vw: int = 7


@dataclass
class RobotState:
    x: float
    y: float
    yaw: float


def wrap_angle(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi


def build_velocity_candidates(cfg: SimConfig) -> Tuple[np.ndarray, np.ndarray]:
    vx = np.linspace(cfg.vx_range[0], cfg.vx_range[1], cfg.n_vx)
    vw = np.linspace(cfg.vw_range[0], cfg.vw_range[1], cfg.n_vw)
    gx, gw = np.meshgrid(vx, vw, indexing="ij")
    return gx.ravel(), gw.ravel()


def generate_obstacles(
    rng: np.random.Generator,
    cfg: SimConfig,
    n_obs: int = 16,
    min_clearance: float = 1.2,
) -> np.ndarray:
    """随机生成圆心坐标，半径统一使用 cfg.robot_radius 作为碰撞阈值参考。"""
    obstacles: List[Tuple[float, float]] = []
    sx, sy = cfg.start
    gx, gy = cfg.goal

    while len(obstacles) < n_obs:
        ox = rng.uniform(cfg.world_x[0], cfg.world_x[1])
        oy = rng.uniform(cfg.world_y[0], cfg.world_y[1])

        if np.hypot(ox - sx, oy - sy) < min_clearance:
            continue
        if np.hypot(ox - gx, oy - gy) < min_clearance:
            continue
        obstacles.append((ox, oy))

    return np.array(obstacles, dtype=float)


def rollout_candidate(
    state: RobotState,
    vx: float,
    vw: float,
    horizon: float,
    dt: float,
    obstacles: np.ndarray,
) -> Tuple[RobotState, float, float]:
    """前向模拟单个候选速度，返回: 末状态、最小障碍距离、轨迹长度。"""
    x, y, yaw = state.x, state.y, state.yaw
    min_dist = np.inf
    steps = int(horizon / dt)

    for _ in range(steps):
        x += vx * np.cos(yaw) * dt
        y += vx * np.sin(yaw) * dt
        yaw = wrap_angle(yaw + vw * dt)

        if obstacles.size:
            d = np.hypot(obstacles[:, 0] - x, obstacles[:, 1] - y)
            min_dist = min(min_dist, float(np.min(d)))

    if not np.isfinite(min_dist):
        min_dist = 999.0
    return RobotState(x, y, yaw), min_dist, abs(vx) * horizon


def evaluate_candidates(
    state: RobotState,
    vel_x: np.ndarray,
    vel_w: np.ndarray,
    obstacles: np.ndarray,
    cfg: SimConfig,
) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
    n = vel_x.shape[0]
    goal = np.array(cfg.goal, dtype=float)

    obstacle_min_dist = np.zeros(n, dtype=float)
    goal_dist = np.zeros(n, dtype=float)
    heading_diff = np.zeros(n, dtype=float)

    for i in range(n):
        s_end, dmin, _ = rollout_candidate(
            state, vel_x[i], vel_w[i], cfg.horizon, cfg.dt, obstacles
        )

        obstacle_min_dist[i] = dmin
        goal_vec = goal - np.array([s_end.x, s_end.y], dtype=float)
        goal_dist[i] = np.linalg.norm(goal_vec)

        goal_dir = np.arctan2(goal_vec[1], goal_vec[0])
        heading_diff[i] = abs(wrap_angle(goal_dir - s_end.yaw))

    cost = cost_fn(vel_x, vel_w, obstacle_min_dist, goal_dist, heading_diff)
    features = {
        "obstacle_min_dist": obstacle_min_dist,
        "goal_dist": goal_dist,
        "heading_diff": heading_diff,
    }
    return cost, features


def collision_happened(state: RobotState, obstacles: np.ndarray, robot_radius: float) -> bool:
    if obstacles.size == 0:
        return False
    d = np.hypot(obstacles[:, 0] - state.x, obstacles[:, 1] - state.y)
    return bool(np.any(d < robot_radius))


def reached_goal(state: RobotState, goal: Tuple[float, float], tol: float = 0.2) -> bool:
    return np.hypot(state.x - goal[0], state.y - goal[1]) < tol


def step_state(state: RobotState, vx: float, vw: float, dt: float) -> RobotState:
    return RobotState(
        x=state.x + vx * np.cos(state.yaw) * dt,
        y=state.y + vx * np.sin(state.yaw) * dt,
        yaw=wrap_angle(state.yaw + vw * dt),
    )


def run_episode_with_trace(
    seed: int, cfg: SimConfig
) -> Tuple[Dict[str, float | bool], np.ndarray, List[Tuple[float, float]]]:
    """运行单局并返回轨迹，便于可视化调试。"""
    rng = np.random.default_rng(seed)
    obstacles = generate_obstacles(rng, cfg)
    vel_x, vel_w = build_velocity_candidates(cfg)

    state = RobotState(cfg.start[0], cfg.start[1], yaw=0.0)
    trace: List[Tuple[float, float]] = [(state.x, state.y)]
    t = 0.0
    survive_time = 0.0

    while t < cfg.max_time:
        cost, _ = evaluate_candidates(state, vel_x, vel_w, obstacles, cfg)
        idx = int(np.argmin(cost))
        state = step_state(state, float(vel_x[idx]), float(vel_w[idx]), cfg.dt)
        trace.append((state.x, state.y))

        t += cfg.dt
        survive_time = t

        if collision_happened(state, obstacles, cfg.robot_radius):
            result = {
                "success": False,
                "collision": True,
                "time": t,
                "survive_time": survive_time,
            }
            return result, obstacles, trace
        if reached_goal(state, cfg.goal):
            result = {
                "success": True,
                "collision": False,
                "time": t,
                "survive_time": survive_time,
            }
            return result, obstacles, trace

    result = {
        "success": False,
        "collision": False,
        "time": cfg.max_time,
        "survive_time": survive_time,
    }
    return result, obstacles, trace


def run_episode(seed: int, cfg: SimConfig) -> Dict[str, float | bool]:
    result, _, _ = run_episode_with_trace(seed, cfg)
    return result


def run_benchmark(n_trials: int = 10, base_seed: int = 2025) -> None:
    cfg = SimConfig()
    results = []
    for i in range(n_trials):
        res = run_episode(base_seed + i, cfg)
        results.append(res)

    success_count = sum(r["success"] for r in results)
    mean_time = np.mean([r["time"] for r in results])
    mean_survive = np.mean([r["survive_time"] for r in results if not r["success"]]) if success_count < n_trials else np.nan

    print("=== Benchmark Summary ===")
    print(f"Trials: {n_trials}")
    print(f"Success: {success_count}/{n_trials}")
    print(f"Mean time: {mean_time:.2f}s")
    if not np.isnan(mean_survive):
        print(f"Mean survive time (failed cases): {mean_survive:.2f}s")


if __name__ == "__main__":
    run_benchmark()
