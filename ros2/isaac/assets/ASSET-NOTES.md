# The real rig CAD/USD assets

Source: supplied directly (not built by this session) — Sarvesh's Isaac Sim
export of the actual rig: `URDF-full-Assembly-ver5_2_no_action_graph.usd`
(the linear stage + rotation bearing + endoscope mount) and
`uretroscope_flexible_CY.usd` (the actual Wolf RIWO ureteroscope mesh,
payload-referenced into the assembly). Directory layout under
`glidar_robot/` is preserved exactly as received — the payload reference
inside the assembly file is a **relative path** that depends on it, so
don't flatten or rename these folders.

This replaces the placeholder box/cylinder URDF
(`../katzlab_bridge/urdf/ureteroscope.urdf`) as the thing Isaac Sim actually
loads and renders from. The placeholder stays as-is for RViz (ROS-side
`robot_state_publisher` wants a plain URDF, which this isn't).

## What's actually verified here (not "unverified like the rest of ros2/isaac")

This is the one part of `ros2/isaac/` checked against something real: plain
`pxr` (USD core), installed and run directly in this environment, not
Isaac Sim itself. Confirmed by actually opening the files and reading
attributes:

- **Loads cleanly from this repo path** with real mesh geometry (table: 96
  points, scope tip mesh: 1728 points) — one harmless console warning about
  an orphaned payload path (`3dmodels/Collected_ureteroscope_assembly/...`,
  which doesn't exist in what was supplied) that can be ignored: the second
  payload entry in the same list (`Ureteroscope_files/...`, which does
  exist) resolves fine and is what actually loads.
- **The cm→m and Y-up→Z-up fix the user flagged as a risk is already
  correct**, not a bug waiting to bite: the referencing prim
  (`.../Endoscope/uretroscope_flexible_CY`) carries explicit
  `xformOp:scale:unitsResolve = (0.01, 0.01, 0.01)` and a
  `rotateX:unitsResolve(90)` / `rotateXYZ(-90,0,0)` pair. Because the scale
  is uniform, it commutes with rotation, so the two rotations cancel
  cleanly and the net effect really is just the intended 0.01 unit
  correction — verified by matrix composition, not assumed.

## A real bug this caught, already fixed

Referencing the assembly USD into a stage that already exists (the normal
pattern — `stage.DefinePrim().GetReferences().AddReference(...)`) does
**not** load payloads introduced by that reference. The scope's visual
detail (`Endoscope/uretroscope_flexible_CY`, a payload arc) silently came
in as an empty Xform with zero children — no error, nothing printed, just
missing geometry. `load_real_rig()` now calls `stage.Load()` right after
adding the reference specifically because of this; confirmed by actually
hitting the empty-children case first, not assumed. If you ever reference
this asset a different way (not through `load_real_rig()`), re-check for
this.

## A real path mistake this caught, already fixed

The 7-segment flexible-tip joints do **not** live where the standalone
`uretroscope_flexible_CY.usd` file's own internal path
(`/Rhino/Geometry/Black/mesh{1..7}/midJoint`) would suggest once it's
composed into the assembly. Confirmed by actually traversing the composed
stage and searching for `midJoint`: they're under a separate, sibling prim,
`Endoscope_tip` (directly authored in the assembly's own layer, not
payloaded — a lightweight physics-only copy of the flexible segment,
distinct from `Endoscope/uretroscope_flexible_CY`'s full-detail visual
mesh). `REAL_JOINT_PATHS["flexion_segments"]` and `REAL_TIP_PRIM_PATH` in
`_isaac_common.py` point at the correct (`Endoscope_tip`) location; if
you're ever tempted to reconstruct these paths from the standalone file
instead of this one, don't — they're not the same subtree.

## What's NOT verified — check these on a real Isaac Sim install

- **Physics/rendering behavior.** Confirmed the *data* is correct, loads,
  and the joint/tip paths resolve (`_isaac_common.py`'s real-rig functions
  are covered by plain-`pxr` checks, no Isaac Sim needed for that part);
  never confirmed how it *looks or simulates* in Isaac Sim itself (no
  install available here, same caveat as the rest of ros2/isaac). In
  particular: setting a joint's `drive:*:physics:targetPosition` attribute
  (what `set_real_linear_target()` etc. do) only takes effect once PhysX
  actually steps the simulation — confirmed the attribute itself sets and
  clamps correctly, did NOT confirm the joint actually drives there in a
  running sim.
- **Two separate `ArticulationRootAPI` prims**: one on
  `.../root_joint` (linear stage + rotation bearing) and one on
  `.../uretroscope_flexible_CY/FixedRootJoint` (the payloaded visual
  scope's own internal root — not the same thing as `Endoscope_tip`, which
  has no articulation root of its own and is driven as plain joints rather
  than through Isaac's `Articulation` wrapper). They are NOT joined into
  one articulation. `_isaac_common.py` drives joints by setting USD drive
  attributes directly for exactly this reason, rather than assuming one
  `Articulation` handle covers everything.
- **Composed bounding box looks short.** The scope's own standalone file
  measures ~83 (cm) along its long axis — a plausible ~83cm full handheld
  ureteroscope length. Composed into the assembly, the same prim's world
  bbox only spans ~0.23m. That's about 3.6x shorter than the clean 0.01
  scaling would predict. Possible causes not chased down here: a `purpose`
  (guide/proxy) flag excluding part of the mesh from the bbox query, or a
  disabled/hidden sub-component. **Look at it in the Isaac Sim viewport
  before trusting the composed geometry's overall scale** — if the scope
  visually reads as way too short relative to the table/linear stage, this
  is why.

## Real numbers this unlocks (vs. the placeholder guesses elsewhere)

| | CAD (this asset) | IROS 2026 paper (Table I) | katzlab `system.yaml` |
|---|---|---|---|
| Linear range | mechanical ±123mm (246mm total) | software 0–177mm | `min_mm: 0, max_mm: 177` |
| Linear max rate | — | 2.0 mm/s | `max_rate_mm_s: 2.5` |
| Rotation range | mechanical ±153.55° (307.1° total) | software ±180° | unbounded (continuous) |
| Rotation max rate | — | 30°/s | `max_rate_deg_s: 20.0` |
| Flexion range | 7 unconstrained continuum joints (software applies the real limit) | ±270° tip | `min_deg: -270, max_deg: 270` |
| Flexion max rate | — | 35°/s | `max_rate_deg_s: 8.0` |

The CAD's mechanical rotation limit (±153.55°) is narrower than what the
paper's software thinks the range is (±180°) — flagged, not resolved. Could
be a conservative default on the CAD joint rather than the true physical
hard stop; worth a bench check rather than assuming either number is wrong.

Everything else lines up closely enough (linear range matches exactly;
rate differences are just different tuning passes, not contradictions) that
cross-checking against two independent real sources gives real confidence
in the general shape of these numbers, even where exact values drifted.

## Real joint paths (for anything driving this asset)

All relative to `REAL_RIG_ROOT` = `/World/URDF_full_Assembly_ver5` (under
whatever prim `load_real_rig()` mounted it at -- `/World/ureteroscope` by
default). These are also exactly what `_isaac_common.REAL_JOINT_PATHS` and
`REAL_TIP_PRIM_PATH` contain -- verified to resolve (`IsValid()`) against
the actual composed stage, not just read off the source file.

```
linear:    LA_Motor/LA_Slider_Prismatic                     (prismatic, meters)
rotation:  Rotatng_bearng_outer/Rotatng_bearng_inner_rev     (revolute, degrees)
flexion:   7x Endoscope_tip/Black/mesh{1..7}/midJoint        (revolute about Y, degrees, no individual limit)
tip prim:  Endoscope_tip/Black/mesh7   (most distal segment; use its world-space
                                         bbox centroid for tip position, NOT its
                                         prim transform -- these meshes bake
                                         position into vertex data, not xformOps)
```

`mesh1` is proximal (nearest the fixed shaft), `mesh7` is the most distal
segment — confirmed by the chain of `physics:body0`/`body1` relationships
(`mesh4` (fixed/metal) → `mesh1` → `mesh2` → ... → `mesh7`), not by naming
convention alone. `Endoscope_tip` is a separate, non-payloaded prim from
`Endoscope/uretroscope_flexible_CY` -- see "A real path mistake" above if
you're reading this out of order.

## Real stone target layout (from the IROS 2026 paper's benchtop experiment)

8 stones, 2×4 grid, 5cm between rows, 6cm between adjacent columns
(center-to-center), submerged in a water-filled container. Used as the
default target layout for the reach task (`../training/reach_task.py`)
instead of a made-up arrangement — real stone geometry/composition still
isn't specified anywhere, so target "stones" there are still placeholder
spheres, just at real positions.
