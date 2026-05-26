# Lab 5：步态动物园 (The Gait Zoo)

教程 [§5.1](https://datawhalechina.github.io/dive-into-embodied-ai/docs/practices/quadruped/cs123/gait-control#51-常见步态) 列了 walk / trot / pace / bound / gallop 五种步态，但 [§5.6](https://datawhalechina.github.io/dive-into-embodied-ai/docs/practices/quadruped/cs123/gait-control#56-原地踏步实验) / [§5.7](https://datawhalechina.github.io/dive-into-embodied-ai/docs/practices/quadruped/cs123/gait-control#57-前进实验) 只把 trot 跑起来。这个目录把 trot / pace / bound 放进同一套脚本：同一只 Pupper、同一套足端轨迹、同一套 IK + PD，只改相位参数。

## 运行后看什么

- `portfolio/gait_zoo.gif`：trot / pace / bound 三联动图，直观看三种步态哪几条腿同相。
- `portfolio/gait_gantts.png`：0-2 秒 stance/swing Gantt 图，把三种步态从视频现象落到相位数据。
- `portfolio/base_z_fft.png`：三种步态的 base z 频谱，用来观察 welded 场景下的微小抖动。

## 起点

打开 `starter.py`。当前仓库把完整实现直接放在 `starter.py`，学习方式是：先运行脚本看三种步态，再按下面几个函数读代码。

| 阅读顺序 | 函数 / 数据 | 看懂什么 |
|---|---|---|
| 1 | `GAITS` | trot / pace / bound 只在 `offsets` 和 `duty` 上不同 |
| 2 | `leg_phase()` | 如何把全局时钟映射成单腿 stance/swing 局部进度 |
| 3 | `foot_trajectory()` | stance 贴地扫线段，swing 用 `sin(pi*s)` 抬脚 |
| 4 | `gait_step()` | 四条腿依次 phase -> trajectory -> IK，合成 12 维目标关节角 |
| 5 | `render_panel_gif_frames()` / `save_gantts()` | 如何把仿真结果变成三联动图和 Gantt 图 |

## 代码主线

```text
GAITS
  定义 trot / pace / bound 的 offsets 和 duty
  ↓
leg_phase()
  全局时间 -> 某条腿处于 stance 还是 swing
  ↓
foot_trajectory()
  局部进度 -> hip-local 足端目标
  ↓
gait_step()
  四条腿足端目标 -> IK -> 12 维关节角
  ↓
make_artifacts.py
  画出三联 GIF、Gantt 图和 base z FFT
```

## MuJoCo 场景

本 Lab 复用 Lab 4 的 `<asset><model>` + `<attach prefix>` 套路。Lab 4 用它并排展示三只 URDF 变体；这里用它并排展示三种 gait。

```xml
<asset>
  <model name="pupper_on_stand" file="../../shared/models/pupper_v3_on_stand.xml"/>
</asset>
<worldbody>
  <frame pos="-0.6 0 0"><attach model="pupper_on_stand" body="base" prefix="trot/"/></frame>
  <frame pos="0 0 0"><attach model="pupper_on_stand" body="base" prefix="pace/"/></frame>
  <frame pos="0.6 0 0"><attach model="pupper_on_stand" body="base" prefix="bound/"/></frame>
</worldbody>
```

`shared/models/pupper_v3_on_stand.xml` 在 `skeleton.xml` 上补了一条 base-world `<weld>`。焊住 base 是为了先观察相位差异：trot 对角腿同相，pace 同侧腿同相，bound 前后腿同相。删掉 weld、接到地面上之后，pace / bound 很快会暴露侧滚或俯仰问题，那是后续 RL 步态要处理的难点。

## 常见坑

1. 不要把 swing 写成普通抛物线；端点速度不为 0，会复现教程 [§5.8](https://datawhalechina.github.io/dive-into-embodied-ai/docs/practices/quadruped/cs123/gait-control#58-常见失败模式) 的"滑步"。
2. 不要重写 IK；Lab 3 已经做过 DLS，本 Lab 直接调 `ik_pupper_leg`。
3. 不要先在 ground 上观察 pace / bound；摔倒姿态会盖过 phase pattern 本身。
4. 不要开 viewer 录 GIF；本 Lab 统一走 offscreen renderer 和 `gif_utils.write_gif`。

## 运行

命令都从 `exercises/` 目录里跑：

```bash
uv run python shared/kinematics/test_leg_kinematics.py
uv run python lab_5_gait_zoo/starter.py
uv run python lab_5_gait_zoo/make_artifacts.py
```

想更深入时再运行 `uv run python lab_5_gait_zoo/tests.py`，它只是帮你确认相位边界、足端轨迹端点和三种步态差异。
