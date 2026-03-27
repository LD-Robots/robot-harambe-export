# Onshape-to-Robot Commands Reference

Reference for the [onshape-to-robot](https://onshape-to-robot.readthedocs.io/en/latest/index.html) tool used to export URDF/SDF/MuJoCo models from Onshape CAD assemblies.

---

## Installation

```bash
pip install onshape-to-robot
```

Optional dependencies:
```bash
pip install pymeshlab   # for STL simplification
pip install coacd       # for convex decomposition
```

---

## API Key Setup

Get your API keys from: https://cad.onshape.com → My Account → Developer menu → API Keys

### Option 1: `.bashrc`
```bash
export ONSHAPE_API=https://cad.onshape.com
export ONSHAPE_ACCESS_KEY=Your_Access_Key
export ONSHAPE_SECRET_KEY=Your_Secret_Key
```

### Option 2: `.env` file in project directory
```
ONSHAPE_API=https://cad.onshape.com
ONSHAPE_ACCESS_KEY=Your_Access_Key
ONSHAPE_SECRET_KEY=Your_Secret_Key
```

---

## Basic Usage

### Project Setup
```bash
mkdir my-robot
# Create my-robot/config.json (see Configuration section)
```

### Export Commands
```bash
# Run the exporter
onshape-to-robot my-robot

# Test with PyBullet
onshape-to-robot-bullet my-robot

# Test with MuJoCo
onshape-to-robot-mujoco my-robot
```

### Convert from Pickle (skip API calls, re-run processors only)
```bash
onshape-to-robot my-robot --convert
```
This loads from a saved `robot.pkl` file and re-runs processors + export without calling the Onshape API. Useful for iterating on processor settings.

---

## Cache / Pickle

The tool automatically caches most Onshape API request results to avoid redundant calls. Workspace-related requests are **not** cached since they rely on live versions that can change.

```bash
# Clear all cached API results
onshape-to-robot-clear-cache
```

### Pipeline Flags: `--retrieve`, `--save-pickle`, `--convert`

The export pipeline has 3 stages: **Retrieve** (call Onshape API → build robot object) → **Save pickle** → **Process & Export** (run processors → write URDF/SDF/MuJoCo). These flags control which stages run:

| Flag | What it does | Calls API? | Saves `robot.pkl`? | Runs processors & export? |
|------|-------------|:----------:|:-------------------:|:-------------------------:|
| *(no flags)* | Full pipeline: retrieve + process + export | Yes | No | Yes |
| `--retrieve` | **Only** fetch from Onshape API and save to `robot.pkl`, then stop | Yes | Yes | No |
| `--save-pickle` | Full pipeline, but also save `robot.pkl` along the way | Yes | Yes | Yes |
| `--convert` | **Only** load `robot.pkl`, run processors, and export — skips API entirely | No | No | Yes |

#### `--retrieve` — Fetch and save pickle only
```bash
onshape-to-robot my-robot --retrieve
```
Calls the Onshape API, builds the internal robot representation, saves it to `my-robot/robot.pkl`, and **stops**. No processors run, no URDF is generated. Use this to snapshot the current Onshape assembly state for later processing.

#### `--save-pickle` — Full pipeline + save pickle
```bash
onshape-to-robot my-robot --save-pickle
```
Runs the full pipeline (retrieve → process → export) **and** saves `robot.pkl`. Useful when you want the export now but also want to keep the pickle for future `--convert` runs.

#### `--convert` — Process and export from pickle only
```bash
onshape-to-robot my-robot --convert
```
Loads `robot.pkl` (previously saved with `--retrieve` or `--save-pickle`), runs all configured processors, and exports. **Does not call the Onshape API at all.** Use this to iterate on processor settings (mesh simplification, merge, convex decomposition, etc.) without waiting for API calls.

#### Typical workflow
```bash
# 1. Fetch from Onshape and save the pickle
onshape-to-robot my-robot --retrieve

# 2. Tweak config.json processors (simplify_stls, merge_stls, etc.)

# 3. Re-export from pickle (fast, no API calls)
onshape-to-robot my-robot --convert

# 4. Repeat step 2-3 until happy with the result
```

### Additional flags

| Flag | Description |
|------|-------------|
| `--safe` | Disables execution of `post_import_commands` and custom imports |
| `--version` | Show package version |

### Pickle Quick Reference

```bash
# Save pickle only (no export)
onshape-to-robot my-robot --retrieve

# Full export + save pickle
onshape-to-robot my-robot --save-pickle

# Export from existing pickle (no API calls, fast)
onshape-to-robot my-robot --convert

# Clear API response cache
onshape-to-robot-clear-cache
```

There is **no `config.json` parameter** to auto-save the pickle. To always save it, add a shell alias to your `.bashrc`:
```bash
alias onshape-to-robot='onshape-to-robot --save-pickle'
```

---

## Configuration (`config.json`)

### Required Parameters

| Parameter | Description |
|-----------|-------------|
| `url` | Onshape assembly URL |
| `output_format` | `"urdf"`, `"sdf"`, or `"mujoco"` |

### General Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `output_filename` | `"robot"` | Output file name (extension added automatically) |
| `assets_directory` | `"assets"` | Directory for mesh and asset storage |
| `robot_name` | directory name | Name used in exported file headers |
| `configuration` | `"default"` | Onshape configuration string (e.g. `"Configuration=BigFoot;RodLength=50mm"`) |
| `ignore_limits` | `false` | Skip joint limits from Onshape |
| `draw_frames` | `false` | Keep frame positioning parts in export |
| `no_dynamics` | `false` | Set all masses/inertia to 0 |
| `include_configuration_suffix` | `true` | Add config suffix to part filenames |
| `round_decimals` | `12` | Decimal precision for numerical values |
| `post_import_commands` | `[]` | Shell commands to execute after import |
| `color` | `null` | Override all part colors as `[r, g, b]` (0-1 range) |

### Alternative URL Parameters
Instead of `url`, you can specify components individually:
- `document_id`, `version_id`, `workspace_id`, `element_id`, `assembly_name`

### Ignore Parts
```json
{
  "ignore": {
    "PartName*": true,
    "!ImportantPart": false
  }
}
```
Supports wildcards (`*`) and negation with `!`.

---

## URDF-Specific Options

| Parameter | Default | Description |
|-----------|---------|-------------|
| `package_name` | `""` | ROS package name for `<robot>` tag |
| `additional_xml` | `""` | Path(s) to XML files to include in the URDF |
| `set_zero_mass_to_fixed` | `false` | Set mass to 0 for bodies fixed to world |

### Joint Properties
Configure per-joint or globally with `"*"`:
```json
{
  "joint_properties": {
    "*": {
      "max_effort": 1.0,
      "max_velocity": 10.0,
      "friction": 0.1
    },
    "dof_head_pitch": {
      "type": "continuous",
      "limits": [-1.57, 1.57]
    }
  }
}
```

### Geometry Properties
Configure collision/visual properties with wildcard pattern matching:
```json
{
  "geom_properties": {
    "wheel_*": {
      "collision": {
        "mu1": 1.0,
        "mu2": 1.0,
        "kp": 1e6,
        "kd": 1.0
      }
    }
  }
}
```

---

## Processors

Processors run after robot retrieval and before export. Enable them in `config.json`.

### Merge STLs
Merges all part meshes within each link into a single mesh.
```json
{
  "merge_stls": true
}
```
Options: `true` (all), `"visual"` (visual only), `"collision"` (collision only), `false` (disabled).

### Simplify STLs
Reduces mesh complexity. Requires `pymeshlab`.
```json
{
  "simplify_stls": true,
  "max_stl_size": 3
}
```
`max_stl_size`: max file size in MB (number), or `"visual"` / `"collision"` to target specific meshes.

### Convex Decomposition
Decomposes collision meshes using CoACD algorithm.
```json
{
  "convex_decomposition": true,
  "rainbow_colors": false
}
```

### Other Processors

| Processor | Config Key | Description |
|-----------|-----------|-------------|
| Ball to Euler | `ball_to_euler` | Converts ball/spherical joints to Euler angle representation |
| Dummy Base Link | `dummy_base_link` | Adds a dummy base reference link |
| No Collision Meshes | `no_collision_meshes` | Removes all collision geometry |
| Collision as Visual | `collision_as_visual` | Uses visual meshes as collision geometry |
| Fixed Links | `fixed_links` | Manages fixed joint connections |
| SCAD | `scad` | OpenSCAD format conversions |
| Custom | `processors` | User-defined processing operations |

---

## Onshape Assembly Design Conventions

### Mate Connector Prefixes

| Prefix | Purpose |
|--------|---------|
| `dof_` | Creates a degree of freedom (joint) |
| `frame_` | Generates a reference frame (dummy link in URDF) |
| `fix_` | Merges two linked components (fixed joint) |
| `closing_` | Closes kinematic loops |

### Joint Types (based on mate type)
- **Revolute / Cylindrical** mate → revolute joint
- **Slider** mate → prismatic joint
- **Fastened** mate → fixed joint

### Naming Conventions
- `dof_<name>` → joint named `<name>`
- `dof_<name>_inv` → inverted joint rotation axis
- `link_<name>` → link renamed to `<name>`
- Joint frames always revolve around / translate along the **Z axis**

### Assembly Rules
- First instance in the assembly = **base link**
- All instances become exported links
- Orphaned links are auto-fixed to base (with warning)
- Use Onshape's "Fixed" feature for fixed-base robots

---

## Example `config.json`

```json
{
  "url": "https://cad.onshape.com/documents/.../w/.../e/...",
  "output_format": "urdf",
  "output_filename": "robot_harambe",
  "robot_name": "harambe",
  "package_name": "full_robot_description",
  "assets_directory": "meshes",
  "merge_stls": true,
  "simplify_stls": true,
  "max_stl_size": 3,
  "joint_properties": {
    "*": {
      "max_effort": 50.0,
      "max_velocity": 5.0
    }
  }
}
```

---

## Typical Workflow

```bash
# 1. Setup project
mkdir my-robot && cd my-robot
# Create config.json with url and output_format

# 2. First export (calls Onshape API, saves robot.pkl)
onshape-to-robot .

# 3. Iterate on processors without re-calling API
onshape-to-robot . --convert

# 4. Clear cache if assembly changed
onshape-to-robot-clear-cache

# 5. Re-export with fresh API data
onshape-to-robot .
```
