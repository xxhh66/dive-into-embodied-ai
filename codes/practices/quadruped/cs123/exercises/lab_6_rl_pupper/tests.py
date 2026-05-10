"""Lab 6 数值断言（不依赖训练完成的 checkpoint，跑 < 30 s）。"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

EXERCISES_DIR = Path(__file__).resolve().parents[1]
if str(EXERCISES_DIR) not in sys.path:
    sys.path.insert(0, str(EXERCISES_DIR))

sys.path.insert(0, str(Path(__file__).resolve().parent))
from envs.pupper_env import PupperEnv, REWARD_WEIGHTS  # noqa: E402


def test_spaces():
    env = PupperEnv()
    assert env.observation_space.shape == (49,), f"obs shape 不对: {env.observation_space.shape}"
    assert env.action_space.shape == (12,), f"action shape 不对: {env.action_space.shape}"
    obs, _ = env.reset(seed=0)
    assert np.all(np.isfinite(obs)), "reset 后 obs 含有 inf/nan"
    print("  test_spaces 通过")


def test_reset_obs():
    env = PupperEnv()
    obs, _ = env.reset(seed=0)
    assert obs.shape == (49,), f"reset obs shape: {obs.shape}"
    assert obs.dtype == np.float32, f"reset obs dtype: {obs.dtype}"
    gravity_z = obs[5]  # base_omega(3) + gravity(3), gravity z is index 5
    assert gravity_z <= -0.9, f"静止站立时 gravity z 分量应 <= -0.9: {gravity_z}"
    gravity_xy = obs[3:5]
    assert np.all(np.abs(gravity_xy) < 0.1), f"静止站立时 gravity xy 应接近 0: {gravity_xy}"
    print("  test_reset_obs 通过")


def test_step_rewards():
    env = PupperEnv()
    env.reset(seed=0)
    action = np.zeros(12, dtype=np.float32)
    _, reward, _, _, info = env.step(action)
    assert info["r_alive"] > 0, f"r_alive 应 > 0: {info['r_alive']}"
    assert 0 < info["r_vel"] <= 1.0, f"r_vel 应在 (0, 1]: {info['r_vel']}"
    assert info["r_torque"] <= 0, f"r_torque 应 <= 0: {info['r_torque']}"
    assert info["r_action_rate"] <= 0, f"r_action_rate 应 <= 0: {info['r_action_rate']}"
    assert np.isfinite(reward), f"总 reward 不是 finite: {reward}"
    print("  test_step_rewards 通过")


def test_reward_weights_and_command():
    expected_keys = {"vel", "alive", "torque", "action_rate", "ori", "height"}
    assert set(REWARD_WEIGHTS.keys()) == expected_keys, f"REWARD_WEIGHTS keys 不对: {set(REWARD_WEIGHTS.keys())}"
    for k, v in REWARD_WEIGHTS.items():
        assert v > 0, f"REWARD_WEIGHTS[{k!r}] 应为正数: {v}"

    env = PupperEnv()
    env.reset(seed=0)
    vx_samples, vy_samples, wz_samples = [], [], []
    for _ in range(100):
        cmd = env._sample_command()
        vx_samples.append(cmd[0])
        vy_samples.append(cmd[1])
        wz_samples.append(cmd[2])
    vx_arr = np.array(vx_samples)
    vy_arr = np.array(vy_samples)
    wz_arr = np.array(wz_samples)
    assert np.all((-0.4 <= vx_arr) & (vx_arr <= 0.6)), f"vx 超出范围"
    assert np.all((-0.2 <= vy_arr) & (vy_arr <= 0.2)), f"vy 超出范围"
    assert np.all((-0.6 <= wz_arr) & (wz_arr <= 0.6)), f"wz 超出范围"
    print("  test_reward_weights_and_command 通过")


def main() -> None:
    print("Lab 6 数值断言:")
    test_spaces()
    test_reset_obs()
    test_step_rewards()
    test_reward_weights_and_command()
    print("Lab 6 检查全部通过。")


if __name__ == "__main__":
    main()
