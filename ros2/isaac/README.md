# Isaac Sim: the digital twin

The real rig CAD now lives here (`assets/glidar_robot/`, see
`assets/ASSET-NOTES.md`) -- supplied directly, not built in this session,
and the one part of `ros2/isaac/` actually checked against something real
(plain USD, installed and run in this environment -- caught and fixed two
real bugs in the process; see ASSET-NOTES.md). Everything below defaults to
loading it; pass `--placeholder` to any script to fall back to the old
placeholder box/cylinder URDF instead (e.g. on a machine without the real,
large, binary assets checked out).

Five scripts, all sharing scene/rig-loading code so none of them can
quietly drift apart from each other (`_isaac_common.py` -- real-rig
loading and joint control, URDF import, the camera mount, the placeholder
anatomy, all the placeholder constants):

- **`load_rig.py`** -- live mirror. Loads the real rig (or placeholder
  URDF) and mirrors `/ureteroscope/joint_states` onto it in real time. One
  direction only: **rig -> sim**. It never publishes back onto the ROS
  graph. See the big comment at the top of the script for why, and for the
  deliberate path to add rig <- sim later if that's ever wanted.

- **`preview_scene.py`** -- the setup step, read this before generating
  anything. Real-rig mode: the actual CAD plus the real 8-stone benchtop
  grid. Placeholder mode (`--placeholder`): the old placeholder URDF +
  camera + fake anatomy + a few stones, for `generate_training_data.py`
  specifically (see below -- that pipeline still needs the placeholder
  scene since the real rig has no camera mount defined yet).

- **`generate_training_data.py`** -- synthetic CV dataset generator.
  **Still placeholder-only** (the real rig has no camera mount point
  defined -- that's the #1 item in `CAMERA-SPECS.md`). Mass-produces
  labeled training data (RGB + segmentation mask + depth + bounding box)
  for the stone-detection CV system from a simulated camera.

- **`training/reach_task.py` + `training/train_reach.py`** -- **the first
  training milestone**: get the tip to the stone. State-based (tip/target
  positions, not vision) on purpose -- it's the simpler problem, and the
  prerequisite for the vision-based version later (no point debugging an RL
  policy and a CV pipeline at the same time). Uses the real rig and the
  real 8-stone benchtop grid from the IROS 2026 paper. See
  `training/reach_task.py`'s docstring for exactly what's verified
  (the reward/observation/action math, `training/reach_math.py`, covered
  by `training/test_reach_math.py` -- runs and passes here) vs. not (the
  actual Isaac physics execution).

## Prerequisites

Everything in `ros2/README.md` section 2 (driver, Isaac Sim itself, sourcing
ROS *before* launching it, the ROS 2 Bridge extension enabled inside Isaac
Sim). Nothing here duplicates that -- read it there.

## Running the live mirror

```bash
source /opt/ros/jazzy/setup.bash

# in one terminal: the real thing driving joint_states --
ros2 launch katzlab_bridge bridge.launch.py dry_run:=true   # or without dry_run, hardware attached

# in another: Isaac Sim
~/isaacsim/python.sh ros2/isaac/load_rig.py                 # real rig (default)
~/isaacsim/python.sh ros2/isaac/load_rig.py --placeholder   # placeholder URDF
```

A window opens with the ground plane and the rig. Move the linear axis
(`ros2 topic pub /ureteroscope/cmd_velocity ...` -- see the example in
`ros2/README.md`) and the carriage should move in the Isaac Sim window
within a step or two.

## Setting up the scene before generating anything

```bash
~/isaacsim/python.sh ros2/isaac/preview_scene.py                 # real rig (default)
~/isaacsim/python.sh ros2/isaac/preview_scene.py --placeholder   # placeholder scene
```

Real-rig mode: check the scope's visual scale against the table/linear
stage (see the "composed bounding box looks short" open item in
`assets/ASSET-NOTES.md`), check the 7-segment flexible tip bends
plausibly when driven through its range, check the 8 stone markers sit
somewhere sensible relative to the tip.

Placeholder mode: the original four checks (camera pointed down the lumen
at rest *and* through the flexion range, anatomy wall visible, stones
inside the lumen) -- this step exists because
`generate_training_data.py` doesn't stop to show you anything; it runs
straight to a bulk headless capture.

## First training milestone: get the tip to the stone

```bash
~/isaacsim/python.sh ros2/isaac/training/reach_task.py   # smoke-test: random actions, no training
~/isaacsim/python.sh ros2/isaac/training/train_reach.py --total-timesteps 200000
```

Needs `gymnasium`, `numpy`, and `stable-baselines3` installed in Isaac
Sim's own Python (`~/isaacsim/python.sh -m pip install gymnasium numpy
stable-baselines3`). PPO was picked as the standard, well-documented
pairing with a Gym-shaped env, not because this task specifically needs
it -- see `training/train_reach.py`'s docstring for the Isaac Lab
alternative and why it wasn't used here.

Before spending GPU time training: `python3 -m pytest
ros2/isaac/training/test_reach_math.py -q` runs right now, no Isaac Sim
needed -- it's the one part of this task actually verified in this
environment (the reward/action/observation math). Run it after any edit to
`reach_math.py`.

## Generating a CV training dataset (placeholder scene only, for now)

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
is real and runnable, the fidelity isn't yet. This script hasn't been
ported to the real rig because the real rig has no defined camera mount
point to attach to -- that's `CAMERA-SPECS.md` item 1/2, and the blocker
for porting this script.

## Known-uncertain: not run against a real Isaac Sim install

Nobody had Isaac Sim available to test any of these scripts against while
writing them -- all written against NVIDIA's *documented* 6.x / Jazzy API
(Replicator and the RL stack included), not verified end to end. The one
exception is the real-rig loading/joint-control code in `_isaac_common.py`
and the reach-task math in `training/reach_math.py` -- both checked against
real tools in this environment (plain USD, plain Python/pytest
respectively); see `assets/ASSET-NOTES.md` and `training/test_reach_math.py`.

Read the top-of-file comment in `load_rig.py` before debugging a
`load_rig.py`/`_isaac_common.py` placeholder-path failure; it names the two
likeliest breakage points (the URDF importer extension's module path, and
`ImportConfig` field names), both of which have moved across Isaac Sim
releases before. For `generate_training_data.py`, the likeliest breakage
points are the exact Replicator API calls (`rep.randomizer.*`,
`BasicWriter`'s constructor arguments, the semantic-tagging API) -- also
moved across versions before; compare against the bundled
`standalone_examples/replicator/` scripts rather than guessing at a
renamed argument. For `training/train_reach.py`, the likeliest breakage
point is the exact stable-baselines3 / gymnasium API surface used.

## Next

- Work through `CAMERA-SPECS.md` -- real camera calibration first, it
  blocks porting `generate_training_data.py` to the real rig.
- Chase down the composed-bbox-looks-short question in
  `assets/ASSET-NOTES.md` -- look at it in the viewport.
- Once `training/reach_task.py` is confirmed actually working on real
  Isaac Sim: swap its ground-truth tip/target positions for the CV
  system's own detections, closing the loop from "reach toward a known
  position" to "reach toward what the camera sees."
- A script that goes the other way -- sim plan -> `/ureteroscope/cmd_velocity`
  -- for rehearsing a motion in Isaac Sim before sending it at the real rig.
  Should reuse the bridge's existing safety path (the normal topic), not a
  side channel.
