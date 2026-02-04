#!/usr/bin/env python3
import argparse
import glob
import os
import random
from pathlib import Path


def build_marker_model(name, pose, texture_path, size=0.4):
    return f"""
    <model name=\"{name}\">
      <static>true</static>
      <pose>{pose}</pose>
      <link name=\"link\">
        <visual name=\"visual\">
          <geometry><box><size>{size} {size} 0.01</size></box></geometry>
          <material>
            <pbr>
              <metal>
                <albedo_map>file://{texture_path}</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
      </link>
    </model>
"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--template", required=True, help="Path to base SDF template")
    parser.add_argument("--output", required=True, help="Path to output SDF")
    parser.add_argument("--img-dir", required=True, help="Directory containing 6 QR images")
    parser.add_argument("--plane-x", type=float, default=4.0, help="Plane X position (in front of UAV)")
    parser.add_argument("--size", type=float, default=0.4, help="Marker size")
    args = parser.parse_args()

    img_dir = Path(args.img_dir).expanduser()
    images = []
    for ext in ("*.png", "*.jpg", "*.jpeg", "*.bmp"):
        images.extend(glob.glob(str(img_dir / ext)))

    if len(images) < 5:
      raise SystemExit(f"Need at least 5 images in {img_dir}, found {len(images)}")

    random.shuffle(images)

    # Positions on the same vertical plane (Y-Z grid), facing camera (pitch=+90deg)
    y_positions = [-1.2, 0.0, 1.2]
    z_positions = [1.6, 2.4]
    positions = [(y, z) for z in z_positions for y in y_positions]
    random.shuffle(positions)

    markers = []
    count = min(len(images), len(positions))
    for i in range(count):
        img = images[i]
        y, z = positions[i]
        pose = f"{args.plane_x:.2f} {y:.2f} {z:.2f} 0 1.5708 0"
        markers.append(build_marker_model(f"qr_marker_{i}", pose, os.path.abspath(img), args.size))

    with open(args.template, "r", encoding="utf-8") as f:
        template = f.read()

    if "<!-- QR_MARKERS -->" not in template:
        raise SystemExit("Template missing <!-- QR_MARKERS --> placeholder")

    output = template.replace("<!-- QR_MARKERS -->", "\n".join(markers))
    with open(args.output, "w", encoding="utf-8") as f:
        f.write(output)


if __name__ == "__main__":
    main()
