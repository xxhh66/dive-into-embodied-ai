"""运行 Lab 5 starter，并写出 portfolio 交付物。"""

from __future__ import annotations

import sys
import shutil
from pathlib import Path

import numpy as np

EXERCISES_DIR = Path(__file__).resolve().parents[1]
if str(EXERCISES_DIR) not in sys.path:
    sys.path.insert(0, str(EXERCISES_DIR))

from shared.viz import gif_utils  # noqa: E402
from starter import (  # noqa: E402
    GAIT_NAMES,
    GIF_FPS,
    PORTFOLIO_DIR,
    render_panel_gif_frames,
    run_experiment,
    save_base_z_fft,
    save_gantts,
    simulate_single_gait,
)


def main() -> None:
    PORTFOLIO_DIR.mkdir(parents=True, exist_ok=True)
    run_experiment(capture_frames=False)
    frames = render_panel_gif_frames()

    deliverable_gif = PORTFOLIO_DIR / "deliverable.gif"
    deliverable_png = PORTFOLIO_DIR / "deliverable.png"
    gif_utils.write_gif(
        frames,
        deliverable_gif,
        fps=GIF_FPS,
        max_frames=80,
        width=1280,
        palette_colors=12,
    )
    shutil.copyfile(deliverable_gif, PORTFOLIO_DIR / "gait_zoo.gif")
    save_gantts(deliverable_png)
    shutil.copyfile(deliverable_png, PORTFOLIO_DIR / "gait_gantts.png")
    traces = {name: simulate_single_gait(name, seconds=8.0) for name in GAIT_NAMES}
    save_base_z_fft(PORTFOLIO_DIR / "base_z_fft.png", traces=traces)

    gif_size_mb = deliverable_gif.stat().st_size / (1024 * 1024)
    print(f"Lab 5 交付物已写入 {PORTFOLIO_DIR}/")
    print(f"已写出 deliverable.gif(=gait_zoo.gif) / deliverable.png(=gait_gantts.png) / base_z_fft.png，GIF={gif_size_mb:.2f} MB")
    for name in GAIT_NAMES:
        print(
            f"{name}: base z std={np.std(traces[name].base_z) * 1000:.3f} mm, "
            f"roll excitation std={np.std(traces[name].roll_excitation):.3f}"
        )


if __name__ == "__main__":
    main()
