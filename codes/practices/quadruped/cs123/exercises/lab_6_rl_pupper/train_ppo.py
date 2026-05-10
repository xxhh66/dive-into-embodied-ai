"""PPO 训练入口。"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

EXERCISES_DIR = Path(__file__).resolve().parents[1]
if str(EXERCISES_DIR) not in sys.path:
    sys.path.insert(0, str(EXERCISES_DIR))

sys.path.insert(0, str(Path(__file__).resolve().parent))
from starter import train_ppo  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="Lab 6: PPO 训练 Pupper")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--total-steps", type=int, default=2_000_000)
    parser.add_argument("--n-envs", type=int, default=8)
    args = parser.parse_args()

    print(f"开始训练: seed={args.seed}, total_steps={args.total_steps}, n_envs={args.n_envs}")
    t0 = time.time()
    path = train_ppo(seed=args.seed, total_timesteps=args.total_steps, n_envs=args.n_envs)
    wall = time.time() - t0
    size_mb = path.stat().st_size / (1024 * 1024)
    print(f"训练完成: {wall / 60:.1f} min, checkpoint = {path} ({size_mb:.1f} MB)")


if __name__ == "__main__":
    main()
