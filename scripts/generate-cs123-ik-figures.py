from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
FIG_DIR = ROOT / "docs/practices/quadruped/cs123/figs"
L = (0.3, 0.25, 0.15)
KP = np.array([40.0, 40.0, 20.0])
KD = np.array([4.0, 4.0, 2.0])


def build_planar_xml() -> str:
    return f"""
<mujoco model="planar_3dof_ik_pd">
  <compiler angle="radian"/>
  <option gravity="0 0 0" timestep="0.002"/>
  <default>
    <joint damping="0.02" armature="0.003"/>
    <geom type="capsule" density="800" rgba="0.72 0.78 0.86 1"/>
    <motor ctrllimited="true" ctrlrange="-20 20"/>
  </default>
  <worldbody>
    <body name="link1" pos="0 0 0">
      <joint name="j1" type="hinge" axis="0 0 1"/>
      <geom fromto="0 0 0 {L[0]} 0 0" size="0.018"/>
      <body name="link2" pos="{L[0]} 0 0">
        <joint name="j2" type="hinge" axis="0 0 1"/>
        <geom fromto="0 0 0 {L[1]} 0 0" size="0.016"/>
        <body name="link3" pos="{L[1]} 0 0">
          <joint name="j3" type="hinge" axis="0 0 1"/>
          <geom fromto="0 0 0 {L[2]} 0 0" size="0.014"/>
          <site name="end_site" pos="{L[2]} 0 0" size="0.012"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="j1"/>
    <motor joint="j2"/>
    <motor joint="j3"/>
  </actuator>
</mujoco>
"""


def make_context() -> tuple[mujoco.MjModel, mujoco.MjData, mujoco.MjData, int]:
    model = mujoco.MjModel.from_xml_string(build_planar_xml())
    sim_data = mujoco.MjData(model)
    ik_data = mujoco.MjData(model)
    end_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_site")
    return model, sim_data, ik_data, end_id


def fk_mj(model: mujoco.MjModel, ik_data: mujoco.MjData, end_id: int, theta: np.ndarray) -> np.ndarray:
    ik_data.qpos[:3] = theta
    mujoco.mj_forward(model, ik_data)
    return ik_data.site_xpos[end_id, :2].copy()


def jac_mj(model: mujoco.MjModel, ik_data: mujoco.MjData, end_id: int, theta: np.ndarray) -> np.ndarray:
    ik_data.qpos[:3] = theta
    mujoco.mj_forward(model, ik_data)
    jacp = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, ik_data, jacp, None, end_id)
    return jacp[:2, :3].copy()


def ik_dls(
    model: mujoco.MjModel,
    ik_data: mujoco.MjData,
    end_id: int,
    theta0: np.ndarray,
    p_target: np.ndarray,
    lam: float = 0.05,
    step: float = 0.3,
    tol: float = 1e-4,
    max_iter: int = 80,
) -> tuple[np.ndarray, int]:
    theta = theta0.copy()
    for k in range(max_iter):
        p = fk_mj(model, ik_data, end_id, theta)
        e = p_target - p
        if np.linalg.norm(e) < tol:
            return theta, k

        J = jac_mj(model, ik_data, end_id, theta)
        JJt = J @ J.T
        dtheta = J.T @ np.linalg.solve(JJt + (lam**2) * np.eye(JJt.shape[0]), e)
        theta = theta + step * dtheta
    return theta, max_iter


def run_tracking(
    target_fn,
    duration: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    model, data, ik_data, end_id = make_context()

    # Start close to the first target so the final plots emphasize tracking
    # behavior instead of the one-time startup transient.
    theta, _ = ik_dls(model, ik_data, end_id, np.zeros(3), target_fn(0.0), max_iter=300)
    data.qpos[:3] = theta
    mujoco.mj_forward(model, data)

    t_log: list[float] = []
    target_log: list[np.ndarray] = []
    actual_log: list[np.ndarray] = []

    while data.time < duration:
        p_target = target_fn(data.time)
        theta, _ = ik_dls(model, ik_data, end_id, theta, p_target)

        q, dq = data.qpos[:3], data.qvel[:3]
        data.ctrl[:3] = KP * (theta - q) + KD * (-dq)
        mujoco.mj_step(model, data)

        t_log.append(float(data.time))
        target_log.append(p_target.copy())
        actual_log.append(data.site_xpos[end_id, :2].copy())

    return np.array(t_log), np.vstack(target_log), np.vstack(actual_log)


def run_triangle_context(
    duration: float = 8.0,
) -> tuple[mujoco.MjModel, mujoco.MjData, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    vertices = np.array([[0.45, 0.10], [0.30, -0.10], [0.55, -0.05]])

    def target_fn(t: float) -> np.ndarray:
        return interpolate_triangle(t, vertices, period=3.0)

    model, data, ik_data, end_id = make_context()
    theta, _ = ik_dls(model, ik_data, end_id, np.zeros(3), target_fn(0.0), max_iter=300)
    data.qpos[:3] = theta
    mujoco.mj_forward(model, data)

    t_log: list[float] = []
    target_log: list[np.ndarray] = []
    actual_log: list[np.ndarray] = []

    while data.time < duration:
        p_target = target_fn(data.time)
        theta, _ = ik_dls(model, ik_data, end_id, theta, p_target)

        q, dq = data.qpos[:3], data.qvel[:3]
        data.ctrl[:3] = KP * (theta - q) + KD * (-dq)
        mujoco.mj_step(model, data)

        t_log.append(float(data.time))
        target_log.append(p_target.copy())
        actual_log.append(data.site_xpos[end_id, :2].copy())

    return model, data, vertices, np.array(t_log), np.vstack(target_log), np.vstack(actual_log)


def interpolate_triangle(t: float, vertices: np.ndarray, period: float = 3.0) -> np.ndarray:
    seg = (t % period) / (period / 3)
    i = int(seg) % 3
    s = seg - int(seg)
    p0, p1 = vertices[i], vertices[(i + 1) % 3]
    return (1 - s) * p0 + s * p1


def style_axis(ax: plt.Axes) -> None:
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(0.20, 0.60)
    ax.set_ylim(-0.18, 0.18)
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    ax.grid(True, color="#e2e8f0", linewidth=0.8)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def draw_tracking_plot(
    target: np.ndarray,
    actual: np.ndarray,
    filename: str,
    title: str,
    note: str,
    vertices: np.ndarray | None = None,
) -> None:
    fig, ax = plt.subplots(figsize=(8.8, 5.4), dpi=180)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("#fbfdff")

    ax.plot(target[:, 0], target[:, 1], color="#2563eb", linewidth=2.4, label="target path")
    ax.plot(actual[:, 0], actual[:, 1], color="#ef4444", linewidth=2.0, label="actual end-effector")
    ax.scatter(target[0, 0], target[0, 1], s=42, c="#16a34a", edgecolors="white", linewidths=1.0, zorder=4, label="start")
    ax.scatter(actual[-1, 0], actual[-1, 1], s=42, c="#0f172a", edgecolors="white", linewidths=1.0, zorder=4, label="last actual")

    if vertices is not None:
        ax.scatter(vertices[:, 0], vertices[:, 1], s=54, c="#2563eb", edgecolors="white", linewidths=1.2, zorder=5)
        for idx, (x, y) in enumerate(vertices, start=1):
            ax.annotate(f"v{idx}", (x, y), xytext=(5, 7), textcoords="offset points", color="#1e40af", fontsize=9)

    ax.text(
        0.02,
        0.96,
        note,
        transform=ax.transAxes,
        va="top",
        fontsize=9,
        color="#334155",
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": "#cbd5e1"},
    )
    ax.set_title(title, pad=12)
    style_axis(ax)
    ax.legend(loc="lower right", frameon=True, facecolor="white", edgecolor="#cbd5e1")
    fig.tight_layout()
    fig.savefig(FIG_DIR / filename)
    plt.close(fig)


def write_circle_tracking() -> None:
    center = np.array([0.45, 0.0])
    radius, period = 0.08, 4.0

    def target_fn(t: float) -> np.ndarray:
        return center + radius * np.array(
            [np.cos(2 * np.pi * t / period), np.sin(2 * np.pi * t / period)]
        )

    t, target, actual = run_tracking(target_fn, duration=20.0)
    last_cycle = t >= t[-1] - period
    draw_tracking_plot(
        target[last_cycle],
        actual[last_cycle],
        "ik-circle-tracking.webp",
        "IK + PD circle tracking",
        "Blue: desired end-effector circle\nRed: simulated end-effector path\nSmall gap = IK residual + PD tracking lag",
    )


def write_triangle_tracking() -> None:
    vertices = np.array([[0.45, 0.10], [0.30, -0.10], [0.55, -0.05]])

    def target_fn(t: float) -> np.ndarray:
        return interpolate_triangle(t, vertices, period=3.0)

    period = 3.0
    t, target, actual = run_tracking(target_fn, duration=15.0)
    last_cycle = t >= t[-1] - period
    draw_tracking_plot(
        target[last_cycle],
        actual[last_cycle],
        "ik-triangle-tracking.webp",
        "IK + PD triangle tracking",
        "Corners are intentionally rounded\nPD cannot create infinite acceleration\nSharper turns expose control bandwidth",
        vertices=vertices,
    )


def write_viewer_debug_snapshot() -> None:
    model, data, vertices, t, target, actual = run_triangle_context()
    end_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_site")
    link2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link2")
    link3_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link3")

    base = np.array([0.0, 0.0])
    joint2 = data.xpos[link2_id, :2].copy()
    joint3 = data.xpos[link3_id, :2].copy()
    end = data.site_xpos[end_id, :2].copy()
    arm = np.vstack([base, joint2, joint3, end])

    recent = t >= t[-1] - 0.8
    fig, ax = plt.subplots(figsize=(8.8, 5.6), dpi=180)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("#f8fafc")

    triangle = np.vstack([vertices, vertices[0]])
    ax.plot(triangle[:, 0], triangle[:, 1], color="#94a3b8", linestyle="--", linewidth=1.6, label="triangle target path")
    ax.plot(actual[recent, 0], actual[recent, 1], color="#ef4444", linewidth=2.2, alpha=0.9, label="recent end-effector trace")
    ax.plot(arm[:, 0], arm[:, 1], color="#475569", linewidth=12, solid_capstyle="round", alpha=0.30)
    ax.plot(arm[:, 0], arm[:, 1], color="#334155", linewidth=2.8, marker="o", markersize=6)

    ax.scatter(target[-1, 0], target[-1, 1], s=180, c="#22c55e", edgecolors="white", linewidths=2.0, zorder=6, label="target ball")
    ax.scatter(end[0], end[1], s=180, c="#ef4444", edgecolors="white", linewidths=2.0, zorder=7, label="current end")
    ax.annotate(
        "green target",
        xy=target[-1],
        xytext=(target[-1, 0] - 0.12, target[-1, 1] + 0.075),
        arrowprops={"arrowstyle": "->", "color": "#16a34a", "lw": 1.4},
        color="#166534",
        fontsize=9,
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "edgecolor": "#bbf7d0"},
    )
    ax.annotate(
        "red current end",
        xy=end,
        xytext=(end[0] - 0.16, end[1] - 0.105),
        arrowprops={"arrowstyle": "->", "color": "#dc2626", "lw": 1.4},
        color="#991b1b",
        fontsize=9,
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "edgecolor": "#fecaca"},
    )

    ax.text(
        0.02,
        0.96,
        "Viewer debug overlay\nTarget and current end are shown together\nDistance between balls exposes IK/PD lag",
        transform=ax.transAxes,
        va="top",
        fontsize=9,
        color="#334155",
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": "#cbd5e1"},
    )
    ax.set_title("DLS viewer overlay: target vs current end-effector", pad=12)
    style_axis(ax)
    ax.set_xlim(-0.05, 0.65)
    ax.set_ylim(-0.30, 0.18)
    ax.legend(loc="lower left", frameon=True, facecolor="white", edgecolor="#cbd5e1")
    fig.tight_layout()
    fig.savefig(FIG_DIR / "ik-viewer-debug-overlay.webp")
    plt.close(fig)


def main() -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    write_circle_tracking()
    write_triangle_tracking()
    write_viewer_debug_snapshot()


if __name__ == "__main__":
    main()
