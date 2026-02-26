# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

URDF processing pipeline for the **LD Robots Harambe** humanoid robot. Converts Onshape CAD exports into simulation-ready URDF/xacro files for a ROS2 robot_description package (Gazebo target).

The main pipeline code lives in the `ldr-urdf-tools` git submodule. The output deploys to `full_robot_description/`.

## Repository Layout

- `ldr-urdf-tools/` — Git submodule containing all pipeline scripts and YAML configs
  - `scripts/` — Python pipeline scripts (see Pipeline section)
  - `configs/` — YAML configs: joint limits, axis conventions, mesh strip patterns, collision strip patterns
- `full_robot_description/` — Target ROS2 package with generated xacro files (joints/ and links/ subdirs)
- `ldr-harambe-v0.3/` — Latest Onshape export (v0.3) with source URDF and STL assets
- `ldr-harambe-v0.3-procesed/` — Post-processing output for v0.3
- `ldr-harambe/` — v0.2 processed exports
- `ldr-harambe-1/` — v0.1 with simplified collision meshes

## Submodule Commands

```bash
# Clone with submodules
git clone --recurse-submodules https://github.com/ld-robots/robot-harambe-export

# Initialize after regular clone
git submodule update --init

# Pull submodule updates
git submodule update --remote
```

## Pipeline

The master script `ldr-urdf-tools/scripts/build_description.py` orchestrates 5 sequential steps:

```bash
# Full pipeline (from repo root, with ldr-urdf-tools as working context):
python ldr-urdf-tools/scripts/build_description.py /path/to/target_description --fixed-legs

# Custom mesh decimation ratio:
python ldr-urdf-tools/scripts/build_description.py /path/to/target_description -r 0.2 --fixed-legs

# Skip steps 1-3 if URDF already processed:
python ldr-urdf-tools/scripts/build_description.py /path/to/target_description --skip-simplify --skip-limits --fixed-legs
```

### Pipeline Steps

1. **urdf_simplify.py** — Merges sub-links/fixed joints, strips meshes matching patterns (motors, fasteners)
2. **joint_correction.py** — Reorients frames to Z-up/X-forward/Y-left, standardizes joint axes
3. **apply_joint_limits.py** — Applies joint limits from `configs/joint_limits.yaml`
4. **simplify_meshes.py** — Decimates visual meshes (Open3D quadric), computes convex hull collisions
5. **split_urdf.py** — Splits monolithic URDF into per-section xacro files with `xacro:include`

Each script can also be run individually (see `ldr-urdf-tools/README.md` for per-script arguments).

## Dependencies

- **Python 3** with `pyyaml` (all scripts) and `open3d` (mesh simplification only)
- **ROS2** with xacro for consuming the output package
- **Gazebo** as the simulation target

## Key Configuration Files

All in `ldr-urdf-tools/configs/`:

| File | Purpose |
|------|---------|
| `joint_correction.yaml` | Base frame orientation (CAD to world) and joint axis conventions (pitch=Y, roll=X, yaw=Z) |
| `joint_limits.yaml` | Per-joint effort, velocity, and position bounds |
| `simplify_config.yaml` | Substring patterns for meshes to strip during simplification |
| `strip_collision.yaml` | Substring patterns for links to remove collision geometry from |

## Architecture Notes

- All URDF processing uses `xml.etree.ElementTree` — no external URDF parsing libraries
- The xacro output uses `xacro:arg` for runtime configuration (`fixed_legs`, `only_left`) that switches joint types between revolute and fixed
- Mesh paths in xacro files use `package://` URIs relative to the ROS2 package name
- Body sections are classified as: Body, Left Arm, Right Arm, Left Foot, Right Foot
- The pipeline is designed to be re-run after each new Onshape export
