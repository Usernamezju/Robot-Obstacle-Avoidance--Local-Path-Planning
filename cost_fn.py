from __future__ import annotations

import numpy as np


def cost_fn(
    vel_x: np.ndarray,
    vel_w: np.ndarray,
    obstacle_min_dist: np.ndarray,
    goal_dist: np.ndarray,
    heading_diff: np.ndarray,
) -> np.ndarray:
    """评分函数（考核核心函数）。

    参数均为长度 49 的 numpy 数组，返回每个速度组合的总代价，值越小越好。

    说明：
    - 这里给了一个可运行的基线版本，便于你直接联调。
    - 你可以只修改这个函数中的权重或公式来做考核优化。
    """

    eps = 1e-6

    # 1) 目标趋近（越近越好）
    goal_term = goal_dist

    # 2) 航向对齐（车头越对准目标越好）
    heading_term = heading_diff

    # 3) 障碍安全（越靠近障碍，惩罚越大）
    # 将 < 0.6 视为强碰撞惩罚。
    collision_mask = obstacle_min_dist < 0.6
    safety_term = 1.0 / (obstacle_min_dist + eps)

    # 4) 速度策略：鼓励前进，抑制倒车和过大角速度抖动
    reverse_penalty = np.clip(-vel_x, 0.0, None)
    angular_penalty = np.abs(vel_w)

    cost = (
        1.5 * goal_term
        + 1.2 * heading_term
        + 1.8 * safety_term
        + 0.8 * reverse_penalty
        + 0.25 * angular_penalty
    )

    # 硬碰撞强惩罚
    cost = np.where(collision_mask, cost + 1e4, cost)
    return cost.astype(float)
