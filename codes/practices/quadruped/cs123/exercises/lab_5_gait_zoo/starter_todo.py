"""Lab 5 TODO starter：步态动物园 (The Gait Zoo)。

学生只需要补三处 TODO：`leg_phase`、`foot_trajectory`、`gait_step`。
其余 MJCF 加载、PD、Gantt、GIF 压缩和 FFT 都已经接好。
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import mujoco
import numpy as np
from PIL import Image, ImageDraw

EXERCISES_DIR = Path(__file__).resolve().parents[1]
if str(EXERCISES_DIR) not in sys.path:
    sys.path.insert(0, str(EXERCISES_DIR))

from shared.kinematics.leg_kinematics import HIP_OFFSETS, LEG_ORDER, ik_pupper_leg  # noqa: E402
from shared.viz import gif_utils, plot_utils  # noqa: E402


LAB_DIR = Path(__file__).resolve().parent
MODEL_PATH = LAB_DIR / "models" / "pupper_zoo.xml"
SINGLE_MODEL_PATH = EXERCISES_DIR / "shared" / "models" / "pupper_v3_on_stand.xml"
PORTFOLIO_DIR = LAB_DIR / "portfolio"

JOINT_SUFFIXES = ("HAA", "HFE", "KFE")
GAIT_NAMES = ("trot", "pace", "bound")
DT = 0.004
MAX_TORQUE = 10.0
KP = np.tile(np.array((8.0, 18.0, 14.0), dtype=float), len(LEG_ORDER))
KD = np.tile(np.array((0.25, 0.55, 0.45), dtype=float), len(LEG_ORDER))
GIF_SECONDS = 12.0
GIF_FPS = 12
FRAME_SIZE = (1280, 480)
GANTT_WINDOW = 2.0


GAITS = {
    "trot": {
        "name": "trot",
        "offsets": {"FL": 0.0, "FR": 0.5, "RL": 0.5, "RR": 0.0},
        "duty": 0.5,
        "T_cycle": 0.4,
        "step_length": 0.055,
        "step_height": 0.04,
        "stand_height": 0.17,
    },
    "pace": {
        "name": "pace",
        "offsets": {"FL": 0.0, "FR": 0.5, "RL": 0.0, "RR": 0.5},
        "duty": 0.5,
        "T_cycle": 0.4,
        "step_length": 0.055,
        "step_height": 0.04,
        "stand_height": 0.17,
    },
    "bound": {
        "name": "bound",
        "offsets": {"FL": 0.0, "FR": 0.0, "RL": 0.5, "RR": 0.5},
        "duty": 0.4,
        "T_cycle": 0.4,
        "step_length": 0.055,
        "step_height": 0.05,
        "stand_height": 0.17,
    },
}


@dataclass
class GaitContext:
    gait_name: str
    gait: dict
    q_seed: dict[str, np.ndarray]


@dataclass
class GaitTrace:
    time: np.ndarray
    base_z: np.ndarray
    base_y: np.ndarray
    roll_excitation: np.ndarray

    @property
    def z_std(self) -> float:
        return float(np.std(self.base_z))

    @property
    def roll_excitation_std(self) -> float:
        return float(np.std(self.roll_excitation))


@dataclass
class ZooTrace:
    time: np.ndarray
    base_z: dict[str, np.ndarray]
    base_y: dict[str, np.ndarray]
    roll_excitation: dict[str, np.ndarray]
    frames: list[np.ndarray] | None = None


def make_context(gait_name: str) -> GaitContext:
    """给一个 gait 创建 warm-start IK context。"""

    if gait_name not in GAITS:
        raise ValueError(f"未知 gait {gait_name!r}")
    return GaitContext(
        gait_name=gait_name,
        gait=GAITS[gait_name],
        q_seed={leg: np.array((0.0, 0.18, -0.36), dtype=float) for leg in LEG_ORDER},
    )


def leg_phase(t: float, leg: str, *, offsets: dict[str, float], duty: float, T_cycle: float) -> tuple[bool, float]:
    """TODO 1：把全局时间映射成某条腿的 stance/swing 局部进度。"""

    if leg not in offsets:
        raise ValueError(f"offsets 缺少 {leg!r}")
    if not (0.0 < duty < 1.0):
        raise ValueError("duty 必须在 (0, 1) 内")
    if T_cycle <= 0.0:
        raise ValueError("T_cycle 必须为正数")

    # TODO 1:
    # 1. 先算全局相位 `(t / T_cycle) % 1.0`。
    # 2. 加上 `offsets[leg]` 得到局部相位。
    # 3. `t_local < duty` 是 stance，否则是 swing。
    # 4. 把当前段内进度归一化成 s in [0, 1)。
    raise NotImplementedError("TODO 1: 请实现 leg_phase")


def foot_trajectory(
    s: float,
    in_stance: bool,
    *,
    step_length: float,
    step_height: float,
    stand_height: float,
) -> np.ndarray:
    """TODO 2：返回 hip-local foot 目标 `(x, y, z)`。

    swing 段必须用 `sin(pi*s)`。普通抛物线端点垂直速度不为 0，会复现
    教程 §5.8 的"滑步"症状。
    """

    if step_length < 0.0 or step_height < 0.0:
        raise ValueError("step_length / step_height 必须非负")
    if stand_height <= 0.0:
        raise ValueError("stand_height 必须为正数")

    # TODO 2:
    # stance: x 从 +L/2 线性走到 -L/2，z 固定为 -stand_height。
    # swing: x 从 -L/2 走回 +L/2，z 用 `step_height * sin(pi*s)` 抬脚。
    # 不要写普通抛物线；教程 §5.8 的滑步症状正是端点速度不为 0。
    raise NotImplementedError("TODO 2: 请实现 foot_trajectory")


def gait_step(t: float, ctx: GaitContext) -> np.ndarray:
    """TODO 3：四条腿 phase → foot trajectory → 4-leg IK，返回 12 维目标 q。"""

    # TODO 3:
    # 对 FL/FR/RL/RR 循环：leg_phase -> foot_trajectory -> ik_pupper_leg。
    # `ik_pupper_leg` 已经处理 HAA mirror；这里不要重写 IK。
    # 返回顺序必须是 FL, FR, RL, RR，每条腿 3 个关节。
    raise NotImplementedError("TODO 3: 请实现 gait_step")


def load_model(path: Path = MODEL_PATH) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """加载 MJCF，并统一仿真步长。"""

    model = mujoco.MjModel.from_xml_path(str(path))
    model.opt.timestep = DT
    return model, mujoco.MjData(model)


def _joint_names(prefix: str = "") -> tuple[str, ...]:
    return tuple(f"{prefix}{leg}_{suffix}" for leg in LEG_ORDER for suffix in JOINT_SUFFIXES)


def _actuator_names(prefix: str = "") -> tuple[str, ...]:
    return tuple(f"{prefix}{leg}_{suffix}_motor" for leg in LEG_ORDER for suffix in JOINT_SUFFIXES)


def _joint_qpos_qvel_ids(model: mujoco.MjModel, prefix: str = "") -> tuple[np.ndarray, np.ndarray]:
    qpos_ids: list[int] = []
    qvel_ids: list[int] = []
    for name in _joint_names(prefix):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"缺少关节 {name!r}")
        qpos_ids.append(int(model.jnt_qposadr[joint_id]))
        qvel_ids.append(int(model.jnt_dofadr[joint_id]))
    return np.asarray(qpos_ids, dtype=int), np.asarray(qvel_ids, dtype=int)


def _actuator_ids(model: mujoco.MjModel, prefix: str = "") -> np.ndarray:
    ids: list[int] = []
    for name in _actuator_names(prefix):
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if actuator_id < 0:
            raise ValueError(f"缺少 actuator {name!r}")
        ids.append(int(actuator_id))
    return np.asarray(ids, dtype=int)


def _base_body_id(model: mujoco.MjModel, prefix: str = "") -> int:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{prefix}base")
    if body_id < 0:
        raise ValueError(f"缺少 body {prefix}base")
    return int(body_id)


def _set_initial_joint_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    contexts: dict[str, GaitContext],
    prefixes: dict[str, str],
) -> None:
    mujoco.mj_resetData(model, data)
    for gait_name, ctx in contexts.items():
        qpos_ids, _ = _joint_qpos_qvel_ids(model, prefixes[gait_name])
        data.qpos[qpos_ids] = gait_step(0.0, ctx)
    mujoco.mj_forward(model, data)


def _apply_pd(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    target_q: np.ndarray,
    *,
    prefix: str = "",
) -> None:
    qpos_ids, qvel_ids = _joint_qpos_qvel_ids(model, prefix)
    actuator_ids = _actuator_ids(model, prefix)
    q = data.qpos[qpos_ids]
    dq = data.qvel[qvel_ids]
    tau = KP * (target_q - q) - KD * dq
    data.ctrl[actuator_ids] = np.clip(tau, -MAX_TORQUE, MAX_TORQUE)


def _roll_excitation(t: float, gait: dict) -> float:
    """stance 分布的侧滚激励 proxy：pace 大，trot/bound 会互相抵消。"""

    side = {"FL": 1.0, "RL": 1.0, "FR": -1.0, "RR": -1.0}
    total = 0.0
    for leg in LEG_ORDER:
        in_stance, _ = leg_phase(t, leg, offsets=gait["offsets"], duty=gait["duty"], T_cycle=gait["T_cycle"])
        total += side[leg] * (1.0 if in_stance else -1.0)
    return total / len(LEG_ORDER)


def simulate_single_gait(gait_name: str, *, seconds: float = 8.0) -> GaitTrace:
    """在单只 welded Pupper 上跑一个 gait，用于 tests 和 FFT。"""

    model, data = load_model(SINGLE_MODEL_PATH)
    ctx = make_context(gait_name)
    _set_initial_joint_pose(model, data, {gait_name: ctx}, {gait_name: ""})
    base_id = _base_body_id(model)

    ts: list[float] = []
    base_z: list[float] = []
    base_y: list[float] = []
    roll: list[float] = []
    steps = int(round(seconds / DT))
    for step in range(steps):
        t = step * DT
        target_q = gait_step(t, ctx)
        _apply_pd(model, data, target_q)
        mujoco.mj_step(model, data)
        ts.append(t)
        base_z.append(float(data.xpos[base_id, 2]))
        base_y.append(float(data.xpos[base_id, 1]))
        roll.append(_roll_excitation(t, ctx.gait))
    return GaitTrace(np.asarray(ts), np.asarray(base_z), np.asarray(base_y), np.asarray(roll))


def simulate_zoo(*, seconds: float = GIF_SECONDS, capture_frames: bool = False) -> ZooTrace:
    """三只 Pupper 在同一个 MJCF 场景里并行跑 trot / pace / bound。"""

    model, data = load_model(MODEL_PATH)
    prefixes = {name: f"{name}/" for name in GAIT_NAMES}
    contexts = {name: make_context(name) for name in GAIT_NAMES}
    _set_initial_joint_pose(model, data, contexts, prefixes)

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
    camera.fixedcamid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "iso")
    renderer = mujoco.Renderer(model, height=FRAME_SIZE[1], width=FRAME_SIZE[0]) if capture_frames else None
    base_ids = {name: _base_body_id(model, prefixes[name]) for name in GAIT_NAMES}

    ts: list[float] = []
    base_z = {name: [] for name in GAIT_NAMES}
    base_y = {name: [] for name in GAIT_NAMES}
    roll = {name: [] for name in GAIT_NAMES}
    frames: list[np.ndarray] = []
    next_frame_time = 0.0
    steps = int(round(seconds / DT))

    try:
        for step in range(steps):
            t = step * DT
            data.ctrl[:] = 0.0
            for gait_name in GAIT_NAMES:
                target_q = gait_step(t, contexts[gait_name])
                _apply_pd(model, data, target_q, prefix=prefixes[gait_name])
            mujoco.mj_step(model, data)

            ts.append(t)
            for gait_name in GAIT_NAMES:
                base_z[gait_name].append(float(data.xpos[base_ids[gait_name], 2]))
                base_y[gait_name].append(float(data.xpos[base_ids[gait_name], 1]))
                roll[gait_name].append(_roll_excitation(t, contexts[gait_name].gait))

            if renderer is not None and t + 0.5 * DT >= next_frame_time:
                renderer.update_scene(data, camera=camera)
                frames.append(_caption_zoo_frame(renderer.render(), t=t))
                next_frame_time += 1.0 / GIF_FPS
    finally:
        if renderer is not None:
            renderer.close()

    return ZooTrace(
        time=np.asarray(ts),
        base_z={name: np.asarray(values) for name, values in base_z.items()},
        base_y={name: np.asarray(values) for name, values in base_y.items()},
        roll_excitation={name: np.asarray(values) for name, values in roll.items()},
        frames=frames if capture_frames else None,
    )


def _caption_zoo_frame(frame: np.ndarray, *, t: float) -> np.ndarray:
    scene = gif_utils.fit_scene_frame(
        frame,
        output_size=FRAME_SIZE,
        content_target=(0.5 * FRAME_SIZE[0], 286.0),
        background_rgb=(0, 0, 0),
    )
    image = Image.fromarray(scene)
    draw = ImageDraw.Draw(image, "RGBA")
    font = gif_utils.load_font(26)
    small = gif_utils.load_font(18)
    draw.rectangle((0, 0, image.width, 50), fill=(255, 255, 255, 228))
    panel_w = image.width / 3.0
    for i, name in enumerate(GAIT_NAMES):
        text_x = int(i * panel_w + 0.5 * panel_w - 34)
        draw.text((text_x, 12), name, fill=(18, 30, 44, 255), font=font)
    draw.text((18, image.height - 34), f"welded in air · t = {t:4.1f} s", fill=(235, 235, 235, 230), font=small)
    return np.asarray(image)


def save_gantts(path: Path) -> None:
    """保存 trot / pace / bound 三联 Gantt 图。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    plot_utils.apply_theme()
    fig = plt.figure(figsize=(11.5, 3.7), constrained_layout=True)
    subfigs = fig.subfigures(1, 3, wspace=0.06)
    for subfig, name in zip(subfigs, GAIT_NAMES):
        plot_utils.gait_gantt(subfig, GAITS[name], T_window=GANTT_WINDOW)
    fig.suptitle("Lab 5：Gantt 步态图（深色 = stance，白色 = swing）")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def save_base_z_fft(path: Path, traces: dict[str, GaitTrace] | None = None) -> dict[str, GaitTrace]:
    """Stretch：保存三种 gait 的 base z FFT 叠加谱。"""

    if traces is None:
        traces = {name: simulate_single_gait(name, seconds=8.0) for name in GAIT_NAMES}
    path.parent.mkdir(parents=True, exist_ok=True)
    plot_utils.apply_theme()
    fig, ax = plt.subplots(figsize=(8.5, 4.0))
    for name in GAIT_NAMES:
        trace = traces[name]
        mask = trace.time >= 1.0
        z = trace.base_z[mask] - np.mean(trace.base_z[mask])
        freqs = np.fft.rfftfreq(len(z), d=DT)
        amp = np.abs(np.fft.rfft(z))
        ax.plot(freqs, amp, label=name)
    ax.set_xlim(0.0, 12.0)
    ax.set_title("base z FFT（weld 场景下的微小抖动）")
    ax.set_xlabel("频率 [Hz]")
    ax.set_ylabel("幅值 [m]")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return traces


def run_experiment(*, capture_frames: bool = True) -> ZooTrace:
    """make_artifacts.py 的主入口。"""

    return simulate_zoo(seconds=GIF_SECONDS, capture_frames=capture_frames)


def main() -> None:
    trace = run_experiment(capture_frames=False)
    print("CS123 Lab 5：步态动物园")
    for name in GAIT_NAMES:
        print(
            f"{name}: base z std={np.std(trace.base_z[name]) * 1000:.3f} mm, "
            f"roll excitation std={np.std(trace.roll_excitation[name]):.3f}"
        )


if __name__ == "__main__":
    main()
