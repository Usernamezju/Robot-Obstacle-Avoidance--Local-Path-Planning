from __future__ import annotations

import argparse

import matplotlib.pyplot as plt
import numpy as np

from main_sim import SimConfig, run_episode_with_trace


def plot_episode(seed: int, save_path: str | None = None) -> None:
    cfg = SimConfig()
    result, obstacles, trace = run_episode_with_trace(seed, cfg)

    path = np.array(trace, dtype=float)
    fig, ax = plt.subplots(figsize=(7, 7))

    # 障碍物（碰撞阈值半径 0.6）
    for ox, oy in obstacles:
        circle = plt.Circle((ox, oy), cfg.robot_radius, color="tab:red", alpha=0.35)
        ax.add_patch(circle)

    # 起点 / 终点 / 轨迹
    ax.plot(path[:, 0], path[:, 1], color="tab:blue", linewidth=2.0, label="trajectory")
    ax.scatter([cfg.start[0]], [cfg.start[1]], color="tab:green", s=80, marker="o", label="start")
    ax.scatter([cfg.goal[0]], [cfg.goal[1]], color="tab:orange", s=100, marker="*", label="goal")

    status = "SUCCESS" if result["success"] else ("COLLISION" if result["collision"] else "TIMEOUT")
    ax.set_title(f"Seed={seed} | {status} | time={result['time']:.1f}s")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_xlim(cfg.world_x)
    ax.set_ylim(cfg.world_y)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best")

    if save_path:
        fig.savefig(save_path, dpi=140, bbox_inches="tight")
        print(f"Saved figure to: {save_path}")
    else:
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="局部路径规划单局可视化")
    parser.add_argument("--seed", type=int, default=2025, help="随机种子")
    parser.add_argument("--save", type=str, default=None, help="保存图片路径（可选）")
    args = parser.parse_args()

    plot_episode(seed=args.seed, save_path=args.save)
