# What's needed to make the sim camera 1:1 with the real one

`generate_training_data.py` and the `camera_link` mount point in
`ureteroscope.urdf` are currently built on **placeholders** — none of this
has been measured against the actual rig. Sim-to-real transfer for a
CV model trained on synthetic data only works if the simulated camera and
scene are close enough to the real thing; right now they're close enough to
validate the *pipeline* (camera → labeled frames → trainable dataset) but
not close enough to trust the labels for real deployment. This is the punch
list to close that gap, roughly in the order it matters.

## 1. Camera optics (blocks everything else)

Find or measure, for the actual camera on the rig:

- **Resolution** (currently placeholder: 1920×1080)
- **Horizontal field of view**, or focal length + sensor width if FOV isn't
  specced directly (currently placeholder: 120°, borrowed from a published
  spec for this class of device, not this camera)
- **Lens distortion** — ureteroscope lenses are usually fisheye-ish; a
  pinhole model (what the script currently sets up) will be visibly wrong
  near the frame edges if the real lens has real barrel distortion. If you
  can run a standard OpenCV checkerboard calibration on the physical camera,
  that gives you both FOV and distortion coefficients in one shot.
- **Working distance / depth of field** — how close can something be and
  still be in focus? Matters for how close stones should spawn to the lens
  in sim.

If there's a datasheet or model number for the camera module, that alone
probably answers most of this without needing a bench calibration.

## 2. Camera mount offset (blocks accurate geometry)

Where does the camera actually sit relative to `flexion_link`'s tip, and
which way does it point? `camera_joint` in the URDF currently guesses "right
at the tip, dead ahead, zero offset" — almost certainly not exactly right.
A rough caliper measurement is enough to start; doesn't need to be exact.

## 3. Lighting

The ring light's real spec (color temperature, lux range) — currently
`LIGHT_INTENSITY_RANGE` in the script guesses 500-5000 (borrowed from the
same published reference as the FOV placeholder). If the real ring light's
datasheet gives a narrower or different range, domain randomization should
be centered on reality, not a guess.

## 4. Anatomy reference

Right now `_build_placeholder_anatomy()` is a tapered cylinder into a
sphere — nothing like real calyx geometry. Doesn't need to be exact for the
pipeline to work, but the more realistic the shape/scale, the better the
transfer. In rough order of effort: real photos/video of the target anatomy
(even just for texture/color reference) < a stock anatomical kidney/ureter
mesh < a real CT-segmented patient model.

## 5. Stone appearance reference

`STONE_COLOR_RANGE` is a generic yellow/tan guess. Real calculi vary a lot
by composition (uric acid, calcium oxalate, struvite, etc. all look
different). A handful of real endoscopic photos of actual stones — even
just a few from a past case, with any identifying info stripped — would let
the color/texture randomization bracket reality instead of guessing.

## 6. Validation, once 1-5 exist

The actual test of "is this good enough": train a small model on synthetic
data only, run it against a handful of **real** endoscopic frames (even
just stills, doesn't need to be from this rig), and see if it generalizes
at all. If it doesn't, the biggest offender is almost always #1 (camera
optics) or #5 (stone appearance) — fix those first before spending more
effort on anatomy fidelity.
