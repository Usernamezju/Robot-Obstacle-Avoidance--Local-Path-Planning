# Robot-Obstacle-Avoidance--Local-Path-Planning

本仓库提供一个可本地运行的特种巡检机器人局部路径规划仿真框架，方便你专注调 `cost_fn.py`。

## 文件说明

- `cost_fn.py`：考核核心函数（你可以在这里改代价公式）
- `main_sim.py`：非考核部分（候选速度生成、障碍物模拟、轨迹预测、失败判定、评分统计）
- `simulation_exam.py`：兼容入口，直接运行进行 10 次随机测试
- `visualize_sim.py`：单局轨迹可视化（障碍物、起终点、轨迹）

## 快速开始

```bash
python simulation_exam.py
```

## 可视化调试

```bash
# 默认直接生成图片（不依赖图形桌面）
python visualize_sim.py --seed 2025

# 指定导出文件名
python visualize_sim.py --seed 2025 --save episode_seed_2025.png

# 本地桌面环境弹窗 + 逐步动态绘制
python visualize_sim.py --seed 2025 --show --animate --delay 0.08
```

> 可视化依赖 `matplotlib`，若环境未安装请先本地安装。

## 任务衔接点

系统每个控制周期会：
1. 离散化速度空间生成 49 个 `(vel_x, vel_w)` 候选；
2. 前向预测每个候选 1.5 秒，得到：`obstacle_min_dist / goal_dist / heading_diff`；
3. 调用 `cost_fn` 返回 49 个代价值；
4. 选择最小代价候选执行 0.1 秒。

你只需优化 `cost_fn` 的数学设计，即可影响安全性、成功率、平滑性和到达时间。


## 为什么改了参数，轨迹可能不变？

这是正常现象，常见原因：
- 决策是离散的 49 组速度，参数小改动若不改变 `argmin` 排名，控制输出就完全一样；
- 你改的是某个代价项权重，但当前场景下该代价项数值范围很小，被其它项“淹没”；
- 你可视化时用的是同一个 `seed`，同场景下若最优序列不变，轨迹就不会变。

建议：先固定 seed，做明显幅度的权重调整（如 x2 或 x0.5），再对比导出的两张图。
