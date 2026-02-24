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
# 弹窗显示（本地桌面环境）
python visualize_sim.py --seed 2025

# 或保存图片
python visualize_sim.py --seed 2025 --save episode_seed_2025.png
```

> 可视化依赖 `matplotlib`，若环境未安装请先本地安装。

## 任务衔接点

系统每个控制周期会：
1. 离散化速度空间生成 49 个 `(vel_x, vel_w)` 候选；
2. 前向预测每个候选 1.5 秒，得到：`obstacle_min_dist / goal_dist / heading_diff`；
3. 调用 `cost_fn` 返回 49 个代价值；
4. 选择最小代价候选执行 0.1 秒。

你只需优化 `cost_fn` 的数学设计，即可影响安全性、成功率、平滑性和到达时间。
