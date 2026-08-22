#!/usr/bin/env python3
import os
import sys
import cv2
import yaml
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from skimage.morphology import skeletonize

# NumPy 1.24+ monkey-patch
if not hasattr(np, 'int'):
    np.int = int
if not hasattr(np, 'float'):
    np.float = float
if not hasattr(np, 'bool'):
    np.bool = bool

repo_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(repo_root / "src" / "planning" / "global_planner"))

from global_planner.global_planner_utils import extract_centerline, smooth_centerline

map_dir = repo_root / "assets" / "maps" / "test" / "f"
out_dir = repo_root / "assets" / "maps_visualization" / "debug_steps_f"
os.makedirs(out_dir, exist_ok=True)

yaml_path = map_dir / "f.yaml"
with open(yaml_path, 'r') as f:
    ydata = yaml.safe_load(f)

img_name = ydata.get('image', 'f.png')
resolution = float(ydata.get('resolution', 0.05))
origin = ydata.get('origin', [0, 0, 0])
free_thresh = float(ydata.get('free_thresh', 0.196))

img_path = map_dir / img_name
raw_img = cv2.imread(str(img_path), 0)

pixel_threshold = int((1.0 - free_thresh) * 255.0)
filtered_map = np.where(raw_img >= pixel_threshold, 255, 0).astype(np.uint8)
filtered_map = cv2.flip(filtered_map, 0)

# STEP 1: Binary Map
fig, ax = plt.subplots(figsize=(6, 8))
ax.imshow(filtered_map, cmap='gray')
ax.set_title("Step 1: Cleaned Binary Map (White=Track, Black=Wall)", fontsize=11, fontweight='bold')
ax.axis('off')
plt.tight_layout()
fig.savefig(out_dir / "step1_binary_map.png", dpi=200, bbox_inches='tight')
plt.close(fig)

# STEP 2: Skeletonize
skeleton = skeletonize(filtered_map // 255, method='lee')

fig, ax = plt.subplots(figsize=(6, 8))
ax.imshow(filtered_map, cmap='gray')
sy, sx = np.where(skeleton > 0)
ax.scatter(sx, sy, c='red', s=1, label='Morphological Skeleton')
ax.set_title("Step 2: Skeletonization (Center Spines)", fontsize=11, fontweight='bold')
ax.legend(loc='upper right')
ax.axis('off')
plt.tight_layout()
fig.savefig(out_dir / "step2_skeleton.png", dpi=200, bbox_inches='tight')
plt.close(fig)

# STEP 3: Raw Centerline Contour
centerline_raw = extract_centerline(
    skeleton=skeleton,
    cent_length=0.0,
    map_resolution=resolution,
    map_editor_mode=True
)

fig, ax = plt.subplots(figsize=(6, 8))
ax.imshow(filtered_map, cmap='gray')
ax.plot(centerline_raw[:, 0], centerline_raw[:, 1], 'g-', linewidth=1.5, label='Raw Skeleton Contour')
ax.set_title("Step 3: Extracted Raw Contour (Unsmoothed)", fontsize=11, fontweight='bold')
ax.legend(loc='upper right')
ax.axis('off')
plt.tight_layout()
fig.savefig(out_dir / "step3_raw_centerline.png", dpi=200, bbox_inches='tight')
plt.close(fig)

# STEP 4: Smoothed Centerline (Showing Savitzky-Golay Overshoot)
centerline_smooth = smooth_centerline(centerline_raw)

fig, ax = plt.subplots(figsize=(6, 8))
ax.imshow(filtered_map, cmap='gray')
ax.plot(centerline_raw[:, 0], centerline_raw[:, 1], 'g--', alpha=0.5, label='Raw Contour')
ax.plot(centerline_smooth[:, 0], centerline_smooth[:, 1], 'b-', linewidth=2, label='Smoothed Centerline (SG Filter)')

top_y_idx = np.argmax(centerline_smooth[:, 1])
knot_x = centerline_smooth[top_y_idx, 0]
knot_y = centerline_smooth[top_y_idx, 1]
ax.plot(knot_x, knot_y, 'r*', markersize=15, label='Knot / Bump Location')

ax.set_title("Step 4: Smoothed Centerline & Knot Location", fontsize=11, fontweight='bold')
ax.legend(loc='upper right')
ax.axis('off')
plt.tight_layout()
fig.savefig(out_dir / "step4_smoothed_centerline.png", dpi=200, bbox_inches='tight')
plt.close(fig)

print(f"[SUCCESS] All step-by-step images generated in {out_dir}/")
