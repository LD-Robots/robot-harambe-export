Useful commands:

  - Clone a repo that has submodules: git clone --recurse-submodules https://github.com/ld-robots/robot-harambe-export
  - Pull submodule updates: git submodule update --remote
  - Initialize submodules after a regular clone: git submodule update --init

  python ldr-urdf-tools/scripts/simplify_meshes.py -i ldr-harambe-v0.3-procesed-copy/robot_harambe_v03_fixed_axes_limits.urdf -r 0.2 --collision-ratio 0.1