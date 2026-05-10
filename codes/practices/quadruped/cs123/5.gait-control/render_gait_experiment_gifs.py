from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[5]
FIG_DIR = ROOT / "docs/practices/quadruped/cs123/figs"
FIG_DIR.mkdir(parents=True, exist_ok=True)

LEGS = ("FL", "FR", "RL", "RR")
LEG_Y = {"FL": 0.07, "FR": -0.07, "RL": 0.07, "RR": -0.07}
LEG_X = {"FL": 0.105, "FR": 0.105, "RL": -0.105, "RR": -0.105}
PHASE_OFFSETS = {"FL": 0.0, "FR": 0.5, "RL": 0.5, "RR": 0.0}

THIGH = 0.105
CALF = 0.105
T_CYCLE = 0.4
STEP_HEIGHT = 0.04
STAND_HEIGHT = 0.17


def build_xml() -> str:
    legs = []
    for leg in LEGS:
        y = LEG_Y[leg]
        x = LEG_X[leg]
        side = 1 if y > 0 else -1
        rgba = "0.10 0.55 0.20 1" if leg in ("FL", "RR") else "0.10 0.32 0.75 1"
        legs.append(
            f"""
            <body name="{leg}_hip" pos="{x:.3f} {y:.3f} 0">
              <joint name="{leg}_hip" type="hinge" axis="1 0 0" range="-0.8 0.8" damping="1.0" armature="0.01"/>
              <geom type="sphere" size="0.017" rgba="{rgba}"/>
              <body name="{leg}_thigh" pos="0 {0.018 * side:.3f} 0">
                <joint name="{leg}_thigh" type="hinge" axis="0 1 0" range="-1.8 1.8" damping="1.0" armature="0.01"/>
                <geom type="capsule" fromto="0 0 0 0 0 {-THIGH:.3f}" size="0.011" rgba="0.55 0.55 0.55 1"/>
                <body name="{leg}_calf" pos="0 0 {-THIGH:.3f}">
                  <joint name="{leg}_calf" type="hinge" axis="0 1 0" range="-2.4 0.2" damping="1.0" armature="0.01"/>
                  <geom type="capsule" fromto="0 0 0 0 0 {-CALF:.3f}" size="0.009" rgba="{rgba}"/>
                  <body name="{leg}_foot" pos="0 0 {-CALF:.3f}">
                    <geom name="{leg}_foot_geom" type="sphere" size="0.018" friction="1.6 0.02 0.002" rgba="0.07 0.07 0.07 1"/>
                  </body>
                </body>
              </body>
            </body>
            """
        )

    return f"""
    <mujoco model="cs123_gait_demo">
      <compiler angle="radian"/>
      <option timestep="0.002" gravity="0 0 -9.81"/>
      <visual>
        <global offwidth="720" offheight="540"/>
        <headlight diffuse="0.7 0.7 0.7" ambient="0.35 0.35 0.35"/>
        <quality shadowsize="2048"/>
      </visual>
      <asset>
        <texture name="grid" type="2d" builtin="checker" rgb1="0.92 0.94 0.92" rgb2="0.78 0.83 0.78" width="512" height="512"/>
        <material name="ground" texture="grid" texrepeat="4 4" reflectance="0.12"/>
      </asset>
      <worldbody>
        <light pos="0 -2 3" dir="0 1 -1" diffuse="0.8 0.8 0.8"/>
        <geom name="floor" type="plane" size="8 8 0.1" material="ground"/>
        <body name="torso" pos="0 0 {STAND_HEIGHT:.3f}">
          <freejoint/>
          <geom name="torso_geom" type="box" size="0.135 0.052 0.035" rgba="0.72 0.72 0.72 1" mass="2.0"/>
          {''.join(legs)}
        </body>
      </worldbody>
      <actuator>
        {''.join(f'<motor joint="{leg}_{joint}" gear="1"/>' for leg in LEGS for joint in ("hip", "thigh", "calf"))}
      </actuator>
    </mujoco>
    """


def leg_phase(t: float, leg: str, t_cycle: float = T_CYCLE, duty: float = 0.5) -> tuple[bool, float]:
    t_global = (t / t_cycle) % 1.0
    t_local = (t_global + PHASE_OFFSETS[leg]) % 1.0
    if t_local < duty:
        return True, t_local / duty
    return False, (t_local - duty) / (1.0 - duty)


def foot_trajectory(s: float, in_stance: bool, step_length: float) -> np.ndarray:
    if in_stance:
        x = step_length * (0.5 - s)
        z = -STAND_HEIGHT
    else:
        x = step_length * (s - 0.5)
        z = -STAND_HEIGHT + STEP_HEIGHT * np.sin(np.pi * s)
    return np.array([x, z])


def ik_leg_2d(x: float, z: float) -> tuple[float, float]:
    down = -z
    reach = np.hypot(x, down)
    reach = np.clip(reach, 0.045, THIGH + CALF - 0.004)
    cos_knee = (reach * reach - THIGH * THIGH - CALF * CALF) / (2 * THIGH * CALF)
    cos_knee = np.clip(cos_knee, -0.98, 0.98)
    knee = -np.arccos(cos_knee)
    hip = np.arctan2(x, down) - np.arctan2(CALF * np.sin(knee), THIGH + CALF * np.cos(knee))
    return hip, knee


def gait_step(t: float, step_length: float) -> np.ndarray:
    target = np.zeros(12)
    for i, leg in enumerate(LEGS):
        in_stance, s = leg_phase(t, leg)
        foot_xz = foot_trajectory(s, in_stance, step_length)
        thigh, calf = ik_leg_2d(float(foot_xz[0]), float(foot_xz[1]))
        target[3 * i : 3 * i + 3] = [0.0, thigh, calf]
    return target


def add_label(frame: np.ndarray, text: str) -> Image.Image:
    image = Image.fromarray(frame)
    draw = ImageDraw.Draw(image, "RGBA")
    draw.rounded_rectangle((18, 18, 330, 68), radius=10, fill=(255, 255, 255, 210))
    draw.text((34, 34), text, fill=(20, 30, 40, 255))
    return image


def render_experiment(name: str, output: Path, step_length: float, base_speed: float) -> None:
    model = mujoco.MjModel.from_xml_string(build_xml())
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=540, width=720)

    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.18, 0.0, 0.09]
    camera.distance = 0.78
    camera.azimuth = 135
    camera.elevation = -18

    q0 = gait_step(0.0, step_length)
    data.qpos[:7] = [0.0, 0.0, STAND_HEIGHT, 1.0, 0.0, 0.0, 0.0]
    data.qpos[7:] = q0
    mujoco.mj_forward(model, data)

    duration = 3.2
    fps = 24
    frames: list[Image.Image] = []

    for frame_index in range(int(duration * fps)):
        t = frame_index / fps
        q_des = gait_step(t, step_length)
        # Keep the torso path prescribed so the GIF focuses on gait timing rather than
        # on model-specific balance tuning.
        data.qpos[:7] = [base_speed * t, 0.0, STAND_HEIGHT, 1.0, 0.0, 0.0, 0.0]
        data.qpos[7:] = q_des
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)

        camera.lookat[0] = base_speed * t + 0.05
        renderer.update_scene(data, camera=camera)
        frames.append(add_label(renderer.render(), name))

    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=1000 // fps,
        loop=0,
        optimize=True,
    )
    print(f"saved {output} ({len(frames)} frames)")


def main() -> None:
    render_experiment(
        name="Experiment 1: in-place trot",
        output=FIG_DIR / "lab5_inplace_trot.gif",
        step_length=0.0,
        base_speed=0.0,
    )
    render_experiment(
        name="Experiment 2: forward trot",
        output=FIG_DIR / "lab5_forward_trot.gif",
        step_length=0.10,
        base_speed=0.22,
    )


if __name__ == "__main__":
    main()
