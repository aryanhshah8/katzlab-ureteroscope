# Command reference

Every command in this repo, what it actually does, and what it touches. All
Python commands are run from `python/`, using the project's own virtualenv:

```bash
cd python
./.venv/bin/python -m katzlab <subcommand> [flags]
./.venv/bin/python scripts/<script>.py
```

The examples below drop the `./.venv/bin/` prefix for readability -- always
use the venv's interpreter, not the system `python3`.

---

## `python -m katzlab <subcommand>`

One process, many subcommands. `--config PATH` and `--input-source
{mac_gamepad,teensy_host,keyboard}` are **global** flags -- they go *before*
the subcommand name, e.g. `katzlab --input-source keyboard run`, not after.

### `ports`

```
katzlab ports
```

Lists every serial port on the machine and flags which ones look like a
Teensy (matched by USB vendor ID). Touches nothing on the board. The first
thing to run when a `bringup`/`run` can't find the Teensy.

### `selftest`

```
katzlab selftest
```

Checks config, controller mapping, serial link, and both firmware handshakes
(motion + laser) -- **without commanding any motion or touching the laser
relays.** Read-only diagnostic. Run this before `bringup` if something feels
off and you want to isolate config/wiring problems from control-loop
behaviour.

### `monitor`

```
katzlab monitor [--port PORT]
```

Opens the serial link and prints every line the firmware sends, live. This is
the raw view underneath everything else -- `TEL` telemetry lines, `Laser
enabled/ready/firing:` status blocks, boot banners. Useful for typing firmware
commands directly (`e`, `r`, `f`, `?`, `w`, `v<us>`, `D<deg>`, `A<sec>` --
see `docs/LASER.md`) without the Python control loop's shaping/dwell logic in
the way. Ctrl-C to stop.

### `bringup` -- the one command for normal use

```
katzlab bringup [--yes] [--skip-flash] [--no-laser] [--no-record]
                 [--no-status] [--ready-open SECONDS] [--debug-buttons]
```

The guided flow: flash the firmware (unless `--skip-flash`), verify the
controller/link/firmware, ask the two physical safety questions a computer
cannot answer for itself (relay supply off before flashing; carriage at
mechanical home), then hand off into the same control loop `run` uses.

| flag | effect |
|---|---|
| `--yes` | assume yes to both safety prompts -- use once you trust the setup |
| `--skip-flash` | firmware is already flashed; skip the upload step (most runs, once flashed once) |
| `--no-laser` | motion only; the laser link is opened but never armed/used |
| `--no-record` | do not write a session CSV to `python/logs/` |
| `--no-status` | hide the live one-line position readout |
| `--ready-open SECONDS` | override `laser.ready_edge_open_s` for this run only, without editing `system.yaml` -- for tuning how long the ready contact must be held before the arming edge |
| `--debug-buttons` | log every button press/release edge and which action it dispatched -- the first thing to turn on when a button "does the wrong thing" |

**Normal daily use:**
```
katzlab bringup --skip-flash --yes
```

**After changing firmware:**
```
katzlab bringup --yes
```
(omit `--skip-flash` so it reflashes; still asks the two safety questions
unless `--yes` is also given)

### `run` -- connect and go, no prompts

```
katzlab run [--no-record] [--no-status] [--debug-buttons] [--dry-run] [--no-laser]
```

The bare control loop: connects to the controller and the Teensy and starts
immediately -- no flash step, no safety confirmations (there is nothing to
confirm, since it never touches the flash). Use this once you've already run
`bringup` at least once this session and just want back in without the
ceremony. `--dry-run` connects the controller but no serial link at all --
logs what it *would* send, moves nothing -- for checking the control mapping
with the rig powered down.

### `bench`

```
katzlab bench [--distance MM] [--timeout SECONDS] [--yes]
```

**Moves the carriage.** Drives the linear axis a configurable distance
(default 10 mm) and back, and reports the *actual* measured mm/s against the
configured ceiling and duty cycle. This is the tool that answers "is it
actually as fast as the config says" instead of trusting the numbers on paper.

### `diagnose-laser`

```
katzlab diagnose-laser [--yes]
```

**Never arms or fires.** Cycles each fire relay coil on and off five times
with the ready contact forced open the entire time, so the machine cannot
fire no matter what the fire relays do. Used to answer one question
independent of arming logic: do the relays physically click when commanded.
Stand at the rig and listen.

### `find-polarity`

```
katzlab find-polarity [--yes]
```

**Arms in each of the four fire-channel polarity combinations, asks what the
Dornier did after each, and never sends a fire command.** Used to find a
polarity where arming does not also trigger a shot, without ever actually
firing while searching. Prints which combination (if any) came back clean at
the end.

### `sweep-fire`

```
katzlab sweep-fire [--pulse SECONDS] [--all]
```

**Arms and fires the laser.** Walks every combination of channel
enable/disable, lead order, and inter-channel gap, firing a short pulse
(default 0.3s) at each and asking whether it fired. This is the tool of last
resort for the fire path -- point the fiber somewhere safe and stay at the
machine before running it. `--all` keeps going through every combination
instead of stopping at the first one that works.

---

## Standalone scripts (`python scripts/*.py`)

Not `katzlab` subcommands -- run directly with the venv's Python.

### `keyboard_control.py`

```
python scripts/keyboard_control.py [extra bringup flags]
```

One command for keyboard-driven control -- equivalent to
`katzlab --input-source keyboard bringup --skip-flash --yes`, with any extra
arguments forwarded straight through to `bringup`. See
`docs/KEYBOARD-CONTROL.md` for the key layout.

### `calibrate_controller.py`

```
python scripts/calibrate_controller.py
```

Interactive wizard. Walks through each stick and button once, live, and
writes the physical axis/button indices to `config/controller.yaml`. Run this
once per physical controller (and again if the controller starts enumerating
under a different name/backend -- the software refuses to run rather than use
a stale mapping in that case).

### `gamepad_monitor.py`

```
python scripts/gamepad_monitor.py
```

Live readout of every raw axis and button on the connected pad, redrawn in
place as you move things. The Python replacement for the old
`Standalone_Controller_Diagnostics.ino` sketch. Use it to read off indices by
hand when the calibration wizard is struggling, or just to confirm the pad is
being seen at all.

### `check_sticks.py`

```
python scripts/check_sticks.py
```

Shows the RAW pad reading next to the SHAPED value the control loop actually
receives, side by side, for each axis. Answers "is this dead because of the
mapping, or dead somewhere in the deadzone/curve/pairing math" -- puts the
fault on one side of the shaping stage or the other instead of guessing.

### `watch_pins.py`

```
python scripts/watch_pins.py
```

**Read-only -- never sends a laser command.** Watches the three laser relay
pins over serial and prints every level transition with a millisecond
timestamp, plus the measured gap between fire channel 1 and 2 when both move.
Run it in a second terminal alongside `bringup`/`run` to see, in real time,
whether the pins are doing what the software thinks they're doing.

### `build_velocity_firmware.py` / `build_merged_firmware.py`

```
python scripts/build_velocity_firmware.py    # regenerates Motion_Velocity_Server.ino
python scripts/build_merged_firmware.py      # regenerates Combined_Motion_Laser.ino
```

**Not something you run day to day.** Both `.ino` files under `firmware/` are
*generated* -- the laser half is spliced in verbatim from the original sketch
under `firmware/original/`, and the motion half comes from
`firmware/src/velocity_motion_core.inc`. If either source changes, run the
matching build script to regenerate the `.ino`, then reflash. Editing the
generated `.ino` files directly is a trap -- the next build overwrites it.

---

## Keyboard control

See `docs/KEYBOARD-CONTROL.md` for the full keyboard layout and how to enable
it -- it's an alternative to the gamepad, selected via `input.source` in
`config/system.yaml` or the `--input-source keyboard` flag on any command
above.

---

## Emergency stop, outside of any of this

`X` on the controller (or the `estop` key on keyboard) stops motion and drops
the laser to a locked, disarmed state through software. If the software
cannot be reached -- a crash, a hung process, a lost link -- **cut power to
the relay supply.** That is the actual emergency stop; everything above is
software asking politely. See `docs/LASER.md` for the full safety writeup,
including the live fail-safe hazard in the current wiring (`READY_ON_NC`).
