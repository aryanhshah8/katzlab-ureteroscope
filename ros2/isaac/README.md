# Isaac Sim: the digital twin

`load_rig.py` imports `../katzlab_bridge/urdf/ureteroscope.urdf` -- the same
URDF RViz uses -- into Isaac Sim and mirrors `/ureteroscope/joint_states`
onto it in real time. One direction only: **rig -> sim**. It never publishes
back onto the ROS graph. See the big comment at the top of the script for
why, and for the deliberate path to add rig <- sim later if that's ever
wanted.

## Prerequisites

Everything in `ros2/README.md` section 2 (driver, Isaac Sim itself, sourcing
ROS *before* launching it, the ROS 2 Bridge extension enabled inside Isaac
Sim). Nothing here duplicates that -- read it there.

## Running

```bash
source /opt/ros/jazzy/setup.bash

# in one terminal: the real thing driving joint_states --
ros2 launch katzlab_bridge bridge.launch.py dry_run:=true   # or without dry_run, hardware attached

# in another: Isaac Sim
~/isaacsim/python.sh ros2/isaac/load_rig.py
```

A window opens with the ground plane and the imported rig. Move the linear
axis (`ros2 topic pub /ureteroscope/cmd_velocity ...` -- see the example in
`ros2/README.md`) and the carriage should slide in the Isaac Sim window
within a step or two.

## Known-uncertain: not run against a real Isaac Sim install

Nobody had Isaac Sim available to test this script against while writing
it -- it was written against NVIDIA's *documented* 6.x / Jazzy API, not
verified end to end. Read the top-of-file comment in `load_rig.py` before
debugging a failure; it names the two likeliest breakage points (the URDF
importer extension's module path, and `ImportConfig` field names), both of
which have moved across Isaac Sim releases before. If either breaks, the fix
almost always lives in that extension's own bundled example scripts
(`extension_examples/interactive_scripts` in the Isaac Sim install) --
compare against those rather than guessing.

## Next, once this loads

- Real meshes instead of the placeholder boxes/cylinders in the URDF, once
  any exist.
- A second script (not this one) that goes the other way -- sim plan ->
  `/ureteroscope/cmd_velocity` -- for rehearsing a motion in Isaac Sim
  before sending it at the real rig. Should reuse the bridge's existing
  safety path (the normal topic), not a side channel.
