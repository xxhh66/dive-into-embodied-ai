"""Lab 5 starter：步态动物园 (The Gait Zoo)。

本文件是教程主线使用的完整脚本。建议先直接运行，确认三种步态
都能生成，再按 `leg_phase` -> `foot_trajectory` -> `gait_step`
的顺序阅读步态生成链路。
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

# 主观察 GIF 直接借用 lab4 trot-ground 的浮动基 MJCF（带棋盘地板 / skybox / spotlight /
# tracking_cam / STL 网格）和 lab4 的 IK / PD，纯把 pattern 换成 trot / pace / bound 三个。
# 视觉等同于 ./viewer.sh lab_4_viewer.py trot-ground，pace / bound 在地面上几步翻车也是
# zoo 的预期教学点。Lab 5 自己的 leg_phase / foot_trajectory / gait_step 仍然驱动 tests.py
# 和 FFT（用 shared 的简化 welded 模型，独立校验）。
LAB4_MODEL_FLOATING = EXERCISES_DIR / "shared" / "models" / "pupper_v3_floating.xml"

JOINT_SUFFIXES = ("HAA", "HFE", "KFE")
GAIT_NAMES = ("trot", "pace", "bound")
DT = 0.004
MAX_TORQUE = 3.0
KP = np.full(12, 15.0, dtype=float)
KD = np.full(12, 0.5, dtype=float)
SETTLE_SECONDS = 0.8
GIF_SECONDS = 6.0
GIF_FPS = 12
FRAME_SIZE = (1280, 480)
PANEL_WIDTHS = (426, 428, 426)
SCENE_TARGET_Y_PX = 286.0
GANTT_WINDOW = 2.0
LAB4_LOOP_RATE = 500
LAB4_DT = 1.0 / LAB4_LOOP_RATE
LAB4_CACHE_DT = 0.02
# 下面这一坨是 lab_4_viewer.cmd_trot_ground 的默认调参（kp/kv/period/stride/swing_z/settle）。
LAB4_KP = 15.0
LAB4_KV = 0.5
LAB4_SETTLE = 0.8
LAB4_PERIOD = 1.2
LAB4_STRIDE_CM = 7.0
LAB4_SWING_Z_CM = 9.0


GAITS = {
    "trot": {
        "name": "trot",
        "offsets": {"FL": 0.0, "FR": 0.5, "RL": 0.5, "RR": 0.0},
        "duty": 0.5,
        "T_cycle": 1.2,
        "step_length": 0.07,
        "step_height": 0.05,
        "stand_height": 0.14,
    },
    "pace": {
        "name": "pace",
        "offsets": {"FL": 0.0, "FR": 0.5, "RL": 0.0, "RR": 0.5},
        "duty": 0.5,
        "T_cycle": 1.2,
        "step_length": 0.07,
        "step_height": 0.05,
        "stand_height": 0.14,
    },
    "bound": {
        "name": "bound",
        "offsets": {"FL": 0.0, "FR": 0.0, "RL": 0.5, "RR": 0.5},
        "duty": 0.4,
        "T_cycle": 1.2,
        "step_length": 0.07,
        "step_height": 0.05,
        "stand_height": 0.14,
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
    """把全局时间映射成某条腿的 stance/swing 局部进度。"""

    if leg not in offsets:
        raise ValueError(f"offsets 缺少 {leg!r}")
    if not (0.0 < duty < 1.0):
        raise ValueError("duty 必须在 (0, 1) 内")
    if T_cycle <= 0.0:
        raise ValueError("T_cycle 必须为正数")

    # t_global 是整只机器人的统一时钟；offsets[leg] 决定这条腿相对
    # 统一时钟提前或滞后多少周期。trot/pace/bound 的差异主要就在这里。
    t_global = (t / T_cycle) % 1.0
    t_local = (t_global + float(offsets[leg])) % 1.0
    if t > 0.0 and np.isclose(t_local, 0.0, atol=1e-12):
        t_local = 1.0

    # duty 是支撑相占整个周期的比例。支撑相返回 s∈[0,1] 表示脚在地面
    # 上向后扫的进度；摆动相返回 s∈[0,1] 表示脚离地前摆的进度。
    if t_local < duty or np.isclose(t_local, duty, atol=1e-12):
        return True, min(t_local / duty, 1.0)
    return False, (t_local - duty) / (1.0 - duty)


def foot_trajectory(
    s: float,
    in_stance: bool,
    *,
    step_length: float,
    step_height: float,
    stand_height: float,
) -> np.ndarray:
    """返回 hip-local foot 目标 `(x, y, z)`。

    swing 段必须用 `sin(pi*s)`。普通抛物线端点垂直速度不为 0，会复现
    教程 §5.8 的"滑步"症状。
    """

    if step_length < 0.0 or step_height < 0.0:
        raise ValueError("step_length / step_height 必须非负")
    if stand_height <= 0.0:
        raise ValueError("stand_height 必须为正数")

    if in_stance:
        # 支撑相：脚相对身体向后扫，模拟脚踩住地面、身体向前经过它。
        x = step_length * (0.5 - s)
        z = -stand_height
    else:
        # 摆动相：脚从后方抬起并向前摆回。sin(pi*s) 让起点/终点高度
        # 都回到 -stand_height，中间最高，视觉上不会擦地。
        x = step_length * (s - 0.5)
        z = -stand_height + step_height * np.sin(np.pi * s)
    return np.array((x, 0.0, z), dtype=float)


def gait_step(t: float, ctx: GaitContext) -> np.ndarray:
    """四条腿 phase → foot trajectory → 4-leg IK，返回 12 维目标 q。"""

    gait = ctx.gait
    target_q = np.zeros(12, dtype=float)
    for k, leg in enumerate(LEG_ORDER):
        # 每条腿先从统一时钟取局部相位，再把相位变成 hip-local 足端目标。
        # 三种 gait 共用这一段，只是 offsets/duty 不同。
        in_stance, s = leg_phase(
            t,
            leg,
            offsets=gait["offsets"],
            duty=float(gait["duty"]),
            T_cycle=float(gait["T_cycle"]),
        )
        hip_local = foot_trajectory(
            s,
            in_stance,
            step_length=float(gait["step_length"]),
            step_height=float(gait["step_height"]),
            stand_height=float(gait["stand_height"]),
        )
        foot_xyz = HIP_OFFSETS[leg] + hip_local
        # ik_pupper_leg 需要的是机器人坐标系下的足端位置，因此 hip-local
        # 目标要加上这条腿的髋部安装偏移。
        q_leg = ik_pupper_leg(foot_xyz, leg=leg, q_seed=ctx.q_seed[leg])
        ctx.q_seed[leg] = q_leg
        target_q[3 * k : 3 * k + 3] = q_leg
    return target_q


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
    steps = int(round((seconds + SETTLE_SECONDS) / DT))
    for step in range(steps):
        wall_t = step * DT
        t = max(0.0, wall_t - SETTLE_SECONDS)
        target_q = gait_step(t, ctx)
        _apply_pd(model, data, target_q)
        mujoco.mj_step(model, data)
        if wall_t < SETTLE_SECONDS:
            continue
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
    steps = int(round((seconds + SETTLE_SECONDS) / DT))

    try:
        for step in range(steps):
            wall_t = step * DT
            t = max(0.0, wall_t - SETTLE_SECONDS)
            data.ctrl[:] = 0.0
            for gait_name in GAIT_NAMES:
                target_q = gait_step(t, contexts[gait_name])
                _apply_pd(model, data, target_q, prefix=prefixes[gait_name])
            mujoco.mj_step(model, data)
            if wall_t < SETTLE_SECONDS:
                continue

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


def _import_lab4():
    """惰性导入 shared/upstream/lab_4_mujoco，借它的 IK 缓存 + 关节顺序映射。"""

    from shared.upstream import lab_4_mujoco  # noqa: WPS433

    return lab_4_mujoco


def _solve_lab4_stance_joints(L) -> np.ndarray:
    """复刻 lab_4_viewer._solve_stance2_joints：得到一组让脚尖贴地的 12 维启动关节角。"""

    stance2 = L.GAIT_KEY_POINTS[2]
    q = np.zeros(12, dtype=float)
    for i, leg in enumerate(L.LEG_NAMES):
        target = stance2 + L.LEG_EE_OFFSETS[leg]
        theta, _ = L.inverse_kinematics_single_leg(
            target,
            leg,
            initial_guess=(0.0, 0.0, 0.0),
            max_iter=500,
            tol=1e-4,
        )
        q[3 * i : 3 * i + 3] = theta
    return q


def _override_pd_lab4(model: mujoco.MjModel, kp: float, kv: float) -> None:
    """复刻 lab_4_viewer._override_pd：把 actuator gainprm/biasprm 改写成 (kp, kv)。"""

    model.actuator_gainprm[:, 0] = kp
    model.actuator_biasprm[:, 1] = -kp
    model.actuator_biasprm[:, 2] = -kv


def _build_lab4_cache(L, gait_name: str, cache_dt: float):
    """复刻 lab_4_viewer._cached_trot：临时改写 GAIT_KEY_POINTS / GAIT_PERIOD 后跑 cache。"""

    orig_pts = L.GAIT_KEY_POINTS.copy()
    orig_period = L.GAIT_PERIOD
    scaled = orig_pts.copy()
    scaled[:, 0] = orig_pts[:, 0] * (LAB4_STRIDE_CM / 5.0)  # ±5 cm → ±stride/2
    scaled[5, 2] = -LAB4_SWING_Z_CM / 100.0
    L.GAIT_KEY_POINTS = scaled
    L.GAIT_PERIOD = LAB4_PERIOD
    try:
        target_joints, _ = L.cache_target_joint_positions(
            pattern=gait_name, period=LAB4_PERIOD, dt=cache_dt,
        )
    finally:
        L.GAIT_KEY_POINTS = orig_pts
        L.GAIT_PERIOD = orig_period
    return target_joints


def _simulate_lab4_gait_panel(
    L,
    gait_name: str,
    *,
    seconds: float,
    panel_width: int,
    stance_q: np.ndarray,
    cache_dt: float,
) -> list[np.ndarray]:
    """单只狗、单个 gait 跑一遍 lab4 trot-ground 场景，按 GIF_FPS 抓 panel 帧。"""

    target_joints = _build_lab4_cache(L, gait_name, cache_dt)
    n_cache = target_joints.shape[0]

    model = mujoco.MjModel.from_xml_path(str(LAB4_MODEL_FLOATING))
    model.opt.timestep = LAB4_DT
    _override_pd_lab4(model, LAB4_KP, LAB4_KV)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    # floating base：前 7 维是 free joint，把 base 抬到 z=0.14、姿态归零再放 12 关节。
    data.qpos[0:3] = (0.0, 0.0, 0.14)
    data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
    data.qpos[7:19] = stance_q
    data.qvel[:] = 0.0
    data.ctrl[:] = stance_q
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, height=FRAME_SIZE[1], width=panel_width)
    # lab4 viewer 默认 free 相机：azimuth=90, elevation=-20。distance 比 MuJoCo 默认的
    # 3 m 拉近到 1.1 m，狗才不会变成芝麻粒。lookat 每帧跟着 base 走，等同 MJCF
    # `<camera mode="targetbody" target="base_link"/>` 但保留我们想要的视角。
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.distance = 1.10
    camera.elevation = -20.0
    camera.azimuth = 90.0
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")

    panels: list[np.ndarray] = []
    next_frame_time = 0.0
    steps = int(round((seconds + LAB4_SETTLE) / LAB4_DT))
    try:
        for step in range(steps):
            wall_t = step * LAB4_DT
            in_settle = wall_t < LAB4_SETTLE
            t_gait = 0.0 if in_settle else (wall_t - LAB4_SETTLE)
            idx = int(t_gait / cache_dt) % n_cache
            data.ctrl[:] = stance_q if in_settle else target_joints[idx]
            mujoco.mj_step(model, data)

            if in_settle:
                continue
            if t_gait + 0.5 * LAB4_DT >= next_frame_time:
                # 让相机始终对着 base，避免 trot 走远后只剩个点。
                camera.lookat[:] = data.xpos[base_id]
                renderer.update_scene(data, camera=camera)
                panels.append(_caption_panel(renderer.render(), label=gait_name, t=t_gait, width=panel_width))
                next_frame_time += 1.0 / GIF_FPS
    finally:
        renderer.close()
    return panels


def render_panel_gif_frames(*, seconds: float = GIF_SECONDS) -> list[np.ndarray]:
    """主观察 GIF 直接复用 lab4 ``trot-ground`` 的 viewer 设定：

    - MJCF: ``lab4/models/pupper_v3_floating.xml``（浮基 + 棋盘地板 + skybox + spotlight +
      ``tracking_cam`` + STL 网格）。
    - PD: ``Kp = 15, Kv = 0.5``，settle = 0.8 s。
    - 步态形状: ``period = 1.2 s``、``stride = 7 cm``、``swing_z = 9 cm``。
    - IK: ``lab_4_mujoco.cache_target_joint_positions``。

    pattern ∈ {trot, pace, bound} 三个 panel 顺序跑（macOS 单 GL context），最后横向拼。
    pace / bound 在 floating + ground 上几步翻车是 zoo 的预期教学点。
    """

    L = _import_lab4()
    stance_q = _solve_lab4_stance_joints(L)

    panels_by_gait: dict[str, list[np.ndarray]] = {}
    for name, panel_width in zip(GAIT_NAMES, PANEL_WIDTHS):
        panels_by_gait[name] = _simulate_lab4_gait_panel(
            L,
            name,
            seconds=seconds,
            panel_width=panel_width,
            stance_q=stance_q,
            cache_dt=LAB4_CACHE_DT,
        )

    n_frames = min(len(panels_by_gait[name]) for name in GAIT_NAMES)
    frames: list[np.ndarray] = []
    for k in range(n_frames):
        frames.append(np.concatenate([panels_by_gait[name][k] for name in GAIT_NAMES], axis=1))
    return frames


def _caption_panel(frame: np.ndarray, *, label: str, t: float, width: int) -> np.ndarray:
    """缩放 lab4 渲染好的 panel 到目标 panel 宽度，叠上 gait 名字。"""

    image = Image.fromarray(frame).convert("RGB")
    target_h = FRAME_SIZE[1]
    if image.size != (width, target_h):
        image = image.resize((width, target_h), Image.Resampling.BILINEAR)
    draw = ImageDraw.Draw(image, "RGBA")
    font = gif_utils.load_font(26)
    draw.rectangle((0, 0, image.width, 50), fill=(255, 255, 255, 228))
    try:
        bbox = draw.textbbox((0, 0), label, font=font)
        text_w = bbox[2] - bbox[0]
    except AttributeError:
        text_w = 70
    draw.text(((image.width - text_w) / 2, 12), label, fill=(18, 30, 44, 255), font=font)
    return np.asarray(image)


def _caption_zoo_frame(frame: np.ndarray, *, t: float) -> np.ndarray:
    scene = gif_utils.fit_scene_frame(
        frame,
        output_size=FRAME_SIZE,
        content_target=(0.5 * FRAME_SIZE[0], SCENE_TARGET_Y_PX),
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
