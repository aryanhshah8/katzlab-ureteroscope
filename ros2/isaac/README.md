# Isaac Sim: the digital twin

Three scripts, all sharing the same scene-building code so none of them can
quietly drift apart from each other (`_isaac_common.py` -- URDF import, the
camera mount, the placeholder anatomy, all the placeholder constants):

- **`load_rig.py`** -- live mirror. Imports
  `../katzlab_bridge/urdf/ureteroscope.urdf` into Isaac Sim and mirrors
  `/ureteroscope/joint_states` onto it in real time. One direction only:
  **rig -> sim**. It never publishes back onto the ROS graph. See the big
  comment at the top of the script for why, and for the deliberate path to
  add rig <- sim later if that's ever wanted.

- **`preview_scene.py`** -- the setup step, read this before the next one.
  Builds the exact scene `generate_training_data.py` renders from (rig,
  camera, anatomy, a few stones) and just sits there with a window open so
  you can look at it -- is the camera actually pointed down the lumen, is
  the anatomy inside-out, are stones actually inside it -- before spending
  GPU time capturing from a scene that turns out to be wrong.

- **`generate_training_data.py`** -- synthetic dataset generator. The actual
  goal this is building toward: use the same digital twin to **mass-produce
  labeled training data for the CV stone-detection system**, rendered from
  the rig's own simulated camera, so the model that eventually runs on the
  real feed has seen far more varied stones/lighting/anatomy than any
  hand-annotated dataset could practically cover. Full detail, including the
  current placeholder camera/anatomy and exactly what's needed to close the
  sim-to-real gap, in `CAMERA-SPECS.md`.

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

## Setting up the scene before generating anything

```bash
~/isaacsim/python.sh ros2/isaac/preview_scene.py
```

A window opens with the rig, the placeholder anatomy, and a few stones. Walk
through the four checks in the script's own docstring (camera pointed the
right way at rest *and* through the flexion range, anatomy wall visible,
stones actually inside the lumen) before moving on. This step exists
because `generate_training_data.py` doesn't stop to show you anything -- it
runs straight to a bulk headless capture, and the first sign of a wrong
camera mount would otherwise be several thousand useless frames in.

## Generating a training dataset

```bash
~/isaacsim/python.sh ros2/isaac/generate_training_data.py \
    --num-frames 5000 --output-dir /data/katzlab-sdg --headless
```

Add `--dynamic` to let stones drift under rigid-body physics between
captures (at roughly the 1-10 mm/s clinical rate both this project's docs
and the comparison literature cite) instead of being re-placed instantly
each frame -- slower, more physically grounded, useful once the faster
"lots of random stills" mode has validated the pipeline.

Output per frame: an RGB image, a colorized + raw semantic segmentation
mask (stone vs. tissue vs. background), a depth map, and tight 2D bounding
boxes -- via Isaac Sim's Replicator `BasicWriter`, the same auto-labeling
mechanism NVIDIA ships worked examples for under
`standalone_examples/replicator/` in the Isaac Sim install.

**Read `CAMERA-SPECS.md` before trusting any of this output for real
training.** The camera intrinsics, mount offset, anatomy geometry, and
stone appearance are all placeholders right now (see the module docstring
in `generate_training_data.py` for which numbers and why) -- the pipeline
is real and runnable, the fidelity isn't yet.

## Known-uncertain: not run against a real Isaac Sim install

Nobody had Isaac Sim available to test either script against while writing
them -- both were written against NVIDIA's *documented* 6.x / Jazzy API
(Replicator included), not verified end to end. Read the top-of-file
comment in `load_rig.py` before debugging a `load_rig.py`/`_isaac_common.py`
failure; it names the two likeliest breakage points (the URDF importer
extension's module path, and `ImportConfig` field names), both of which
have moved across Isaac Sim releases before. For `generate_training_data.py`
specifically, the likeliest breakage points are the exact Replicator API
calls (`rep.randomizer.*`, `BasicWriter`'s constructor arguments, the
semantic-tagging API) -- these have also moved across Isaac Sim versions;
compare against the bundled `standalone_examples/replicator/` scripts
rather than guessing at a renamed argument.

## Next

- Work through `CAMERA-SPECS.md` -- real camera calibration first, it
  blocks almost everything else downstream.
- Real meshes instead of the placeholder boxes/cylinders/tube-and-sphere in
  the URDF and anatomy, once any exist.
- A script that goes the other way -- sim plan -> `/ureteroscope/cmd_velocity`
  -- for rehearsing a motion in Isaac Sim before sending it at the real rig.
  Should reuse the bridge's existing safety path (the normal topic), not a
  side channel.
