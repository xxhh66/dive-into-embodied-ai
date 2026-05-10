"""填完 starter 三处空白后运行的 Lab 5 数值检查。"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

EXERCISES_DIR = Path(__file__).resolve().parents[1]
if str(EXERCISES_DIR) not in sys.path:
    sys.path.insert(0, str(EXERCISES_DIR))

from starter import GAITS, foot_trajectory, leg_phase, simulate_single_gait  # noqa: E402


def test_phase_semantics() -> tuple[tuple[bool, float], tuple[bool, float], tuple[bool, float]]:
    gait = GAITS["trot"]
    duty = 0.5
    T_cycle = 0.4
    fl0 = leg_phase(0.0, "FL", offsets=gait["offsets"], duty=duty, T_cycle=T_cycle)
    rr_half = leg_phase(0.5 * T_cycle, "RR", offsets=gait["offsets"], duty=duty, T_cycle=T_cycle)
    fr_half = leg_phase(0.5 * T_cycle, "FR", offsets=gait["offsets"], duty=duty, T_cycle=T_cycle)
    assert fl0 == (True, 0.0), f"FL t=0 相位不对：{fl0}"
    assert rr_half[0] is True, f"trot 对角腿 RR 在半周期边界应仍按上一段 stance 处理：{rr_half}"
    assert fr_half[0] is False, f"FR 在半周期边界应处于上一段 swing 末端：{fr_half}"
    return fl0, rr_half, fr_half


def test_swing_endpoint_height() -> tuple[float, float]:
    kwargs = {"step_length": 0.055, "step_height": 0.04, "stand_height": 0.17}
    z0 = float(foot_trajectory(0.0, False, **kwargs)[2])
    z1 = float(foot_trajectory(1.0, False, **kwargs)[2])
    assert np.isclose(z0, -kwargs["stand_height"]), f"swing 起点 z 不等于 -stand_height：{z0}"
    assert np.isclose(z1, -kwargs["stand_height"]), f"swing 终点 z 不等于 -stand_height：{z1}"
    return z0, z1


def test_welded_base_and_roll_distribution() -> tuple[float, float, float]:
    trot = simulate_single_gait("trot", seconds=8.0)
    pace = simulate_single_gait("pace", seconds=8.0)
    trot_z_std = float(np.std(trot.base_z))
    trot_roll = trot.roll_excitation_std
    pace_roll = pace.roll_excitation_std
    assert trot_z_std < 0.005, f"weld 场景下 trot base z 抖动过大：{trot_z_std * 1000:.2f} mm"
    assert pace_roll > trot_roll + 0.35, f"pace 的侧滚激励不够明显：pace={pace_roll:.3f}, trot={trot_roll:.3f}"
    return trot_z_std, trot_roll, pace_roll


def test_gantt_offsets() -> dict[str, bool]:
    assert GAITS["trot"]["offsets"]["FL"] == GAITS["trot"]["offsets"]["RR"]
    assert GAITS["trot"]["offsets"]["FR"] == GAITS["trot"]["offsets"]["RL"]
    assert GAITS["trot"]["offsets"]["FL"] != GAITS["trot"]["offsets"]["FR"]

    assert GAITS["pace"]["offsets"]["FL"] == GAITS["pace"]["offsets"]["RL"]
    assert GAITS["pace"]["offsets"]["FR"] == GAITS["pace"]["offsets"]["RR"]
    assert GAITS["pace"]["offsets"]["FL"] != GAITS["pace"]["offsets"]["FR"]

    assert GAITS["bound"]["offsets"]["FL"] == GAITS["bound"]["offsets"]["FR"]
    assert GAITS["bound"]["offsets"]["RL"] == GAITS["bound"]["offsets"]["RR"]
    assert GAITS["bound"]["offsets"]["FL"] != GAITS["bound"]["offsets"]["RL"]
    return {"trot_X": True, "pace_equals": True, "bound_Z": True}


def main() -> None:
    phase = test_phase_semantics()
    endpoints = test_swing_endpoint_height()
    trot_z_std, trot_roll, pace_roll = test_welded_base_and_roll_distribution()
    gantt = test_gantt_offsets()
    print("Lab 5 检查全部通过。")
    print(f"phase samples = {phase}")
    print(f"swing endpoint z = {endpoints}")
    print(f"trot base z std = {trot_z_std * 1000:.3f} mm")
    print(f"roll excitation std: trot={trot_roll:.3f}, pace={pace_roll:.3f}")
    print(f"Gantt sanity = {gantt}")


if __name__ == "__main__":
    main()
