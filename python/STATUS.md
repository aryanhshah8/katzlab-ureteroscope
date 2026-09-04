# KatzLab ureteroscope — status

Written 2026-09-03, updated 2026-09-04. **Working.**

---

## Works

**Motion — finished.** Velocity-mode firmware, all three axes. Smooth, continuous,
proportional to stick deflection, stoppable inside a millisecond. Tuned to feel:

| axis | max | deadzone | curve |
|---|---|---|---|
| linear | 2.5 mm/s | 0.04 | 0.0 (linear) |
| rotation | 20 °/s | 0.04 | 0.35 |
| flexion | 8 °/s | 0.015 | 0.8 |

Right stick: up/down = rotation, left/right = flexion. Left stick = linear.

**Laser arming** works. **Firing** works in the current configuration.

**Standby** stops firing without arming the machine.

## Solved: standby was arming the machine

The machine arms on the ready contact reaching its "open" level. Standby drove
that same level and HELD it, so Standby armed instead of disarmed. Arming works
because it passes *through* that level and settles back.

Two changes fixed it:

- `standby_drives_ready: false` -- the operator's Standby parks the fire
  channels and locks the software without touching the ready contact.
- `arm()` sends the open command **directly** rather than calling `standby()`.
  Routing it through standby meant that when standby stopped driving ready,
  arming silently lost its edge and every fire was refused for want of an armed
  machine. Passing through that level and stopping there are different things.

## Solved: the machine only half-armed

`ready_edge_open_s` was too short. The contact has to be held in the state the
machine reads for **3.0 s** before the closing edge -- effectively how long a
person would hold the ready button. Below that it half-arms. Determined on
hardware by sweeping the value with `--ready-open`.

## Current configuration

| setting | value | where |
|---|---|---|
| relay pins | 8 ready, 9/10 fire | `shootingyesready_inoF-1.ino` |
| `READY_ON_NC` | `true` | same |
| fire polarity | both normal | `config/system.yaml` |
| `standby_drives_ready` | `false` | same |
| `ready_edge_open_s` | `3.0` | same |

Arming, firing, standby and motion all working.

---

## The three real bugs fixed (all host-side, all mine)

**1. Motion was eating the laser's serial replies.**
`MotionVelocityDriver.state` drains the link, and `set_velocity()` returns
`self.state` — so it drained 60×/second. On a single board the laser shares that
port, so its command replies were swallowed before it could read them. The laser
then timed out and acted on a stale view of the hardware. This is what made
everything erratic and unreproducible. `Link.drain()` now takes a filter and
motion consumes only `TEL` lines.

**2. `arm()` was driving the fire relays.**
It sent `x` first for a deterministic start, but `x` resets the fire channels
too — and that transition was enough to make the machine fire. So Enable/Ready
fired the laser. `arm()` now sends only `e` then `r`.

**3. Phantom button presses.**
After a blocking laser command the loop re-polled the pad and overwrote the edge
tracker. One unlucky read made the next frame look like a fresh press —
manufacturing commands the operator never gave. Removed; a per-action cooldown
handles staleness without inventing input.

Also fixed along the way: rate-control integrating idle time instead of
wall-clock (linear was ~6× too slow), the firmware watchdog stalling a held
stick, servo speed not matched to commanded rate (flexion stutter), and fire
retrying every frame when it failed.

---

## What to do next

**Meter the Steute connector.** This is the thing that ends it.

1. Relay supply off. For each relay module, check continuity COM→NO and COM→NC,
   coil released and coil energised.
2. Press the real footswitch. Note which terminal pairs close and which open.

Those two maps answer polarity, terminals and channel order at once.

**Why this matters:** every laser change in this session fixed one button and
broke another — ready works/fire doesn't, fire works/standby arms, fix
standby/fire dies. That is not the signature of a code bug. The three code bugs
above each stayed fixed and made things strictly better. The revolving symptom
is what you get when relay contacts are on circuits other than the ones the
software assumes, and no setting can compensate for that.

**Also worth doing:** `READY_ON_NC = true` means pin 8 is on the relay's NC
terminal, so a reset, replug, upload or power loss ARMS the laser. Moving that
wire to NO and setting `READY_ON_NC = false` makes power loss fail safe.

---

## Motion recording

Every session writes `python/logs/motion-<timestamp>.csv` -- one row per sample
at 20 Hz, plus a forced row on every operator action.

| column | |
|---|---|
| `t_s`, `wall_clock` | seconds since start, and an ISO timestamp |
| `linear_mm` | carriage position |
| `rotation_deg` | ureteroscope rotation, after the 2.5:1 reduction |
| `flexion_deg` | tip flexion angle |
| `linear_mm_s`, `rotation_deg_s` | velocities |
| `laser_enabled/ready/firing`, `estopped` | state |
| `event` | blank except on arm / fire / home / e-stop |

Events force a row and flush immediately, so a fire is never dropped to keep
the sample cadence and never lost to a crash.

Off with `--no-record`, or `logging.record_motion: false`. Rate is
`logging.record_hz`.

## Commands

```bash
cd "/Users/aryan/Documents/Code/KatzLab/python"

./.venv/bin/python -m katzlab bringup --skip-flash --yes    # normal start
./.venv/bin/python -m katzlab bringup --skip-flash --yes --ready-open 3.0   # tune arming
./.venv/bin/python -m katzlab bringup --yes                 # reflash first
./.venv/bin/python -m katzlab bringup --skip-flash --yes --no-laser   # motion only

./.venv/bin/python -m katzlab monitor          # raw serial
./.venv/bin/python -m katzlab diagnose-laser   # relay bench test, never fires
./.venv/bin/python -m katzlab find-polarity    # arm in each polarity, never fires
./.venv/bin/python -m katzlab sweep-fire       # walk channel/order/gap, DOES fire
./.venv/bin/python -m katzlab bench            # measure real linear mm/s
./.venv/bin/python scripts/gamepad_monitor.py  # live pad readout
```

Emergency: `pkill -f katzlab`, then cut the relay supply. Cutting power is the
only real stop — a motion command in flight cannot be interrupted mid-move in
chunked mode, and the laser relays have not behaved as the software expects.

Firmware is generated, never hand-edited:
```bash
./.venv/bin/python scripts/build_velocity_firmware.py   # motion core + laser verbatim
./.venv/bin/python scripts/build_merged_firmware.py     # the older chunked build
```

80 tests: `./.venv/bin/python -m pytest tests/ -q`
