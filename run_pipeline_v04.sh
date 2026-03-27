#!/bin/bash
set -e

# Step 1: Fix joint axes
python3 ldr-urdf-tools/scripts/joint_correction.py \
  ldr-harambe-v0.4/robot_harambe_v04_simplified.urdf \
  -o ldr-harambe-v0.4/robot_harambe_v04_fixed_axes.urdf \
  -c ldr-urdf-tools/configs/joint_correction.yaml

# Step 2: Simplify meshes
python3 ldr-urdf-tools/scripts/simplify_meshes.py \
  -i ldr-harambe-v0.4/robot_harambe_v04_fixed_axes.urdf \
  -o ldr-harambe-v0.4/robot_harambe_v04_fixed_axes_simplified.urdf \
  -r 0.2 \
  --convex-collision \
  --source-dir ldr-harambe-v0.4/assets \
  --visual-dir ldr-harambe-v0.4/meshes/visual \
  --collision-dir ldr-harambe-v0.4/meshes/collision

# Step 3: Strip collision
python3 ldr-urdf-tools/scripts/strip_collision.py \
  -i ldr-harambe-v0.4/robot_harambe_v04_fixed_axes_simplified.urdf \
  -o ldr-harambe-v0.4/robot_harambe_v04_fixed_axes_simplified_no_collision.urdf \
  -c ldr-urdf-tools/configs/strip_collision.yaml

# Step 4: Split into xacro
python3 ldr-urdf-tools/scripts/split_urdf.py \
  -i ldr-harambe-v0.4/robot_harambe_v04_fixed_axes_simplified_no_collision.urdf \
  -p full_robot_description \
  -o ldr-harambe-v0.4 \
  --fixed-legs
