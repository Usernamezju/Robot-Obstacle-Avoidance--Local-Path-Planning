from __future__ import annotations

import argparse
import os
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle

from main_sim import SimConfig, run_episode_with_trace


def _can_use_gui() -> bool:
    """检查当前环境是否具备弹窗显示能力。"""
    return bool(matplotlib.get_backend()) and bool(os.environ.get("DISPLAY"))


def _draw_scene(ax: plt.Axes, cfg: SimConfig, obstacles: np.ndarray, result: dict, seed: int) -> None:
    for ox, oy in obstacles:
        ax.add_patch(Circle((ox, oy), cfg.robot_radius, color="tab:red", alpha=0.35))

    status = "SUCCESS" if result["success"] else ("COLLISION" if result["collision"] else "TIMEOUT")
    ax.set_title(f"Seed={seed} | {status} | time={result['time']:.1f}s")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_xlim(cfg.world_x)
    ax.set_ylim(cfg.world_y)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", alpha=0.4)


def plot_episode(
    seed: int,
    save_path: str | None = None,
    show: bool = False,
    animate: bool = False,
    delay: float = 0.05,
) -> Path | None:
    """绘制单局轨迹：支持静态图导出和逐步动态播放。"""
    cfg = SimConfig()
    result, obstacles, trace = run_episode_with_trace(seed, cfg)
    path = np.array(trace, dtype=float)

    fig, ax = plt.subplots(figsize=(7, 7))
    _draw_scene(ax, cfg, obstacles, result, seed)

    # 起点终点
    ax.scatter([cfg.start[0]], [cfg.start[1]], color="tab:green", s=80, marker="o", label="start")
    ax.scatter([cfg.goal[0]], [cfg.goal[1]], color="tab:orange", s=120, marker="*", label="goal")

    if animate:
        line, = ax.plot([], [], color="tab:blue", linewidth=2.0, label="trajectory")
        for i in range(1, len(path) + 1):
            line.set_data(path[:i, 0], path[:i, 1])
            if show and _can_use_gui():
                plt.pause(max(delay, 0.001))
    else:
        ax.plot(path[:, 0], path[:, 1], color="tab:blue", linewidth=2.0, label="trajectory")

    ax.legend(loc="best")

    saved_file: Path | None = None
    if save_path:
        saved_file = Path(save_path)
        fig.savefig(saved_file, dpi=140, bbox_inches="tight")
        print(f"Saved figure to: {saved_file}")

    if show:
        if _can_use_gui():
            plt.show()
        else:
            print("Warning: 当前环境无图形界面，已跳过弹窗显示。请使用 --save 导出图片。")

    plt.close(fig)
    return saved_file


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="局部路径规划单局可视化")
    parser.add_argument("--seed", type=int, default=2025, help="随机种子")
    parser.add_argument("--save", type=str, default=None, help="保存图片路径（可选）")
    parser.add_argument("--show", action="store_true", help="是否弹窗显示")
    parser.add_argument("--animate", action="store_true", help="逐步动态绘制轨迹")
    parser.add_argument("--delay", type=float, default=0.05, help="动态绘制每步延迟(秒)")
    args = parser.parse_args()

    # 默认行为：即便不传 --save，也会落一张图，确保“有图可见”
    output = args.save or f"episode_seed_{args.seed}.png"
    plot_episode(
        seed=args.seed,
        save_path=output,
        show=args.show,
        animate=args.animate,
        delay=args.delay,
    )
