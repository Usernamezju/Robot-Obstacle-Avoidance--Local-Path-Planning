# Robot-Obstacle-Avoidance--Local-Path-Planning

本仓库用GPT codex提供一个可本地运行的特种巡检机器人局部路径规划仿真框架，方便你专注调 `cost_fn.py`，为了验证我们这个题目的实用性，你可以先运行示例代码再尝试调试。

## 文件说明

- `cost_fn.py`：考核核心函数（你可以在这里改代价公式）
- `main_sim.py`：非考核部分（候选速度生成、障碍物模拟、轨迹预测、失败判定、评分统计）
- `simulation_exam.py`：兼容入口，直接运行进行 10 次随机测试

## 快速开始

```bash
python simulation_exam.py
```

## 任务衔接点

系统每个控制周期会：
1. 离散化速度空间生成 49 个 `(vel_x, vel_w)` 候选；
2. 前向预测每个候选 1.5 秒，得到：`obstacle_min_dist / goal_dist / heading_diff`；
3. 调用 `cost_fn` 返回 49 个代价值；
4. 选择最小代价候选执行 0.1 秒。

你只需优化 `cost_fn` 的数学设计，即可影响安全性、成功率、平滑性和到达时间。
