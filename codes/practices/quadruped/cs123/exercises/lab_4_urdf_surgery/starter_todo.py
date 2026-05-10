"""Lab 4 TODO starter：Pupper URDF 手术 + PD 甜点扫描。

本文件是学生侧骨架。作者侧 filled 版在 `starter.py`，交付前会抽走。
只留三处算法空白：变体 MJCF factory、stand_pose 求解、PD 甜点扫描。
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import mujoco
import numpy as np

EXERCISES_DIR = Path(__file__).resolve().parents[1]
if str(EXERCISES_DIR) not in sys.path:
    sys.path.insert(0, str(EXERCISES_DIR))

from shared.controllers.pd_controller import PDGains  # noqa: E402

# 这些常量和 filled starter 保持一致。粘合代码（渲染、plot、pickle）在 filled 版里
# 已经写完；学生只需要把下面三处 TODO 补齐即可。
LAB_DIR = Path(__file__).resolve().parent
MODEL_DIR = LAB_DIR / "models"
PORTFOLIO_DIR = LAB_DIR / "portfolio"
SKELETON_PATH = EXERCISES_DIR / "shared" / "models" / "skeleton.xml"

LEG_ORDER = ("FL", "FR", "RL", "RR")
JOINT_SUFFIXES = ("HAA", "HFE", "KFE")
THIGH_LEN = 0.08
CALF_LEN = 0.11
THIGH_MASS = 0.186
CALF_MASS = 0.050
TORSO_MASS = 1.506
FOOT_RADIUS = 0.018
DT = 0.004
MAX_TORQUE = 18.0
KP_GRID = np.array((10.0, 30.0, 60.0, 120.0), dtype=float)
KD_GRID = np.array((0.5, 1.0, 2.0, 5.0), dtype=float)


def make_variant(name: str, *, leg_scale: float = 1.0, torso_mass_scale: float = 1.0) -> Path:
    """TODO 1：写出一份 include skeleton 的变体 MJCF。

    要改的量：
    - `variant_thigh` / `variant_calf` 的 `geom fromto` 长度；
    - thigh / calf 的 `mass`，按 leg_scale 近似线性缩放；
    - `variant_torso` 的 `mass` 按 torso_mass_scale 缩放；
    - heavy 可以用 visual-only 的 `variant_ballast` 背包块标出来，不要把 torso 变成大白盒；
    - foot site / foot geom 的 `pos`，跟 calf 长度保持一致。

    输出文件必须仍然只有一行：
    `<include file="../../shared/models/skeleton.xml"/>`
    """

    raise NotImplementedError("TODO 1：生成 pupper_v3 / pupper_longleg / pupper_heavy 的 MJCF")


def find_stand_pose(model: mujoco.MjModel, leg_scale: float = 1.0) -> np.ndarray:
    """TODO 2：给单只 Pupper 求 HAA=0 的 12 维 stand_pose。

    提示：先把一条腿看成二维链。给定 hip 高度，搜索 HFE / KFE，让 foot 的
    hip-local z 接近 -0.14 m，同时 x 不要偏离 hip 太远。长腿 Pupper 可以让
    foot z 更低一点，对应更高的 base。
    """

    raise NotImplementedError("TODO 2：搜索 HFE/KFE 并 tile 成 12 维 stand_pose")


def pd_sweet_spot(
    model: mujoco.MjModel,
    stand_pose: np.ndarray,
    kp_grid: np.ndarray = KP_GRID,
    kd_grid: np.ndarray = KD_GRID,
) -> tuple[float, float, np.ndarray]:
    """TODO 3：扫描 PD 甜点。

    对每个 `(Kp, Kd)` 跑 6 秒站立，控制律写在这里：
    `tau = Kp * (qd - q) + Kd * (0 - qdot)`。
    返回 `(best_kp, best_kd, z_std_grid)`，其中 `z_std_grid[i, j]`
    是最后 1 秒 base z 的标准差。
    """

    raise NotImplementedError("TODO 3：实现 PD torque、仿真循环和 z_std heatmap")


if __name__ == "__main__":
    print("这是学生 TODO 版：请先补齐 TODO 1/2/3，再运行 tests.py。")
