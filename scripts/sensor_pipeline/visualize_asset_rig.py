"""Create a traceable Stage 2 contact sheet from saved outputs only."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

parser = argparse.ArgumentParser()
parser.add_argument("--sample", type=Path, required=True)
args = parser.parse_args()
sample = args.sample.resolve()
result = json.loads((sample / "smoke.json").read_text())
xyz = np.load(sample / "lidar_xyz.npy")

fig, axes = plt.subplots(2, 2, figsize=(11, 7.5))
overview = sample / "assembly_render.png"
if overview.exists():
    axes[0, 0].imshow(Image.open(overview))
axes[0, 0].set_title("Assembly stage render (not GUI screenshot)")
axes[0, 0].axis("off")
for axis, name in [(axes[0, 1], "left_rgb.png"), (axes[1, 0], "right_rgb.png")]:
    axis.imshow(Image.open(sample / name))
    axis.set_title(name.replace("_", " ") + "; no matched depth")
    axis.axis("off")
plot = axes[1, 1].scatter(xyz[:, 0], xyz[:, 1], c=xyz[:, 2], s=0.35, cmap="viridis")
axes[1, 1].set_aspect("equal")
axes[1, 1].set_title(f"RTX measurement: {len(xyz):,} returns")
axes[1, 1].set_xlabel("sensor x / m")
axes[1, 1].set_ylabel("sensor y / m")
fig.colorbar(plot, ax=axes[1, 1], label="sensor z / m")
fig.suptitle("Go2-W Stage 2 sensor rig: appearance and measurement kept separate")
fig.tight_layout()
fig.savefig(sample / "stage2_contact_sheet.png", dpi=160)

checks = {
    "measurement_points_finite": bool(np.isfinite(xyz).all()),
    "point_count": int(len(xyz)),
    "left_shape": result["left_shape"],
    "right_shape": result["right_shape"],
    "imu_valid": result["imu_valid"],
    "appearance_prim_count": result["appearance_prim_count"],
    "collision_prim_count": len(result["collision_prims_under_rig"]),
    "points_inside_self_scan_guard": result["points_inside_self_scan_guard"],
    "gui_screenshot": result["gui_screenshot"],
    "stereo_depth_computed": result["stereo_depth_computed"],
}
(sample / "sample_checks.json").write_text(json.dumps(checks, indent=2))
print(json.dumps(checks))
