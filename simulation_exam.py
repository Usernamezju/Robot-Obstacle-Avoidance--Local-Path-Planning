"""兼容入口：保留这个文件名，直接运行即可开始本地评测。"""

from main_sim import run_benchmark


if __name__ == "__main__":
    run_benchmark(n_trials=10, base_seed=2025)
