# KatzLab ureteroscope control

Python gamepad control for a Teensy 4.1 ureteroscope rig: three motion axes and
a Dornier laser interface. The controller plugs into the laptop; the firmware
executes commands and streams telemetry back.

```
  [gamepad] --USB--> [laptop, Python] --USB--> [Teensy 4.1] --> drivers, servo, relays
```

## Quick start

```bash
cd python
python3 -m venv .venv && ./.venv/bin/pip install -r requirements.txt
./.venv/bin/python scripts/calibrate_controller.py     # once, per controller
./.venv/bin/python -m katzlab bringup --yes            # flash, verify, run
```

Windows: **[docs/WINDOWS-SETUP.txt](docs/WINDOWS-SETUP.txt)**

## Controls

| input | action |
|---|---|
| Left stick | linear travel |
| Right stick up/down | rotation |
| Right stick left/right | flexion |
| LB / LT | Enable+Ready / Standby |
| RB / RT | Fire / Pause |
| X | **Emergency stop** |
| H (hold) | Home all axes |

## What's here

```
firmware/
  Motion_Velocity_Server/   flash this -- velocity motion + laser   (generated)
  Combined_Motion_Laser/    older command-per-move build            (generated)
  src/                      hand-maintained motion core
  original/                 the original sketches; source of truth
python/
  katzlab/                  the control system
  config/                   system.yaml (tuning), controller.yaml (mapping)
  scripts/                  calibration, diagnostics, firmware generation
  tests/                    80 tests against firmware simulators
docs/
```

**Firmware is generated, never hand-edited.** Edit the originals, then:

```bash
./.venv/bin/python scripts/build_velocity_firmware.py
```

The laser half is spliced from `original/shootingyesready_inoF-1` verbatim, and
the build fails if any of its safety features go missing.

## Recording

Every session writes `python/logs/motion-<timestamp>.csv` -- position on all
three axes, velocities, laser state, and an `event` column marking arm, fire,
home and e-stop. A live position line is shown while running.

## Safety

`READY_ON_NC = true`: pin 8 sits on the relay's NC terminal, so the ready
contact is **closed -- the laser armed -- whenever the coil is de-energised**. A
reset, replug, upload or loss of relay supply arms the laser with nobody
commanding it.

- **De-energise the relay supply before flashing.**
- Standby is a *software* disarm on this rig. The real disarm is the Dornier's
  own standby, or cutting the relay supply.

Full detail and the hardware fix: **[docs/LASER.md](docs/LASER.md)**

## Docs

- **[docs/STATUS.md](docs/STATUS.md)** -- current state, what was wrong, what was fixed
- **[docs/LASER.md](docs/LASER.md)** -- arming sequence and troubleshooting
- **[docs/PYTHON.md](docs/PYTHON.md)** -- architecture and configuration
- **[docs/WINDOWS-SETUP.txt](docs/WINDOWS-SETUP.txt)** -- Windows setup, in order
