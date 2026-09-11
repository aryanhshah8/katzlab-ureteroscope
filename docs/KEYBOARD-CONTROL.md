# Keyboard control

An alternative to the gamepad. Same control loop, same safety logic, same
firmware protocol underneath -- only where the input comes from changes.

## Enabling it

Either edit `config/system.yaml`:

```yaml
input:
  source: keyboard
```

or, without touching the config file, pass it on the command line (this flag
goes *before* the subcommand):

```bash
katzlab --input-source keyboard bringup --skip-flash --yes
katzlab --input-source keyboard run
```

The terminal you run it in becomes the input device -- it must be an
interactive terminal (not piped, not a background job). No gamepad needs to
be connected, but `config/controller.yaml` must still exist on disk (even
though its contents are ignored in keyboard mode) because the current startup
code loads it unconditionally before choosing which input backend to use.
Run `scripts/calibrate_controller.py` once beforehand if it has never been
generated on this machine, even if you only ever plan to use the keyboard.

## The layout

| Key | Action |
|---|---|
| **Up / Down** | Rotation |
| **Left / Right** | Flexion |
| **A / D** | Linear travel |
| **R** | Enable + Ready (arm) |
| **F** | Fire -- hold down to fire |
| **S** | Standby |
| **P** | Pause |
| **H** | Home -- hold ~0.6s, same as the controller's H button |
| **X** | **EMERGENCY STOP** |

Letters work in either case -- `f` and `F` do the same thing. This is
deliberate: the layout is meant to be driven by feel without looking down at
the keyboard, and requiring Shift for some keys but not others would only
invite mistakes under pressure.

Arrow keys are read as standard terminal escape sequences, which every common
terminal (macOS Terminal, iTerm, and Linux terminal emulators alike) sends the
same way, so this works identically on the Mac and on the Mint machine.

### One key that wasn't on the original request list

**R (Enable + Ready)** was added. It isn't part of what was asked for, but
without a way to arm the laser, **F (Fire) can never do anything** -- the
laser refuses to fire while disarmed, same as on the gamepad. R was chosen
because it isn't used anywhere else in the layout and reads naturally as
"Ready." If a different key is wanted instead, it's a one-line change in
`katzlab/input/keyboard.py` (`SINGLE_KEYS`).

## How "holding a key" actually works, and why it matters

A gamepad tells the software the instant a button is released. A terminal
cannot -- it only reports that a key went *down*; while it's held, the
operating system's own key-repeat resends the same character at whatever rate
your terminal is configured for, and when you let go, nothing else arrives at
all. There is no key-up signal to read.

So "held" is approximated: each key remembers when it was last seen, and
counts as held for 0.35 seconds after that. While a key is genuinely pressed,
OS repeat keeps refreshing that timestamp faster than it can expire, so it
reads as one continuous hold -- which is what lets **F** behave as "fire
while held" rather than firing once per keystroke.

Two consequences worth knowing, not discovering the hard way:

- **If your terminal's "delay until repeat" setting is unusually slow**, a
  genuine hold can read as press-release-press instead of one smooth hold
  (you'd see Fire pause briefly mid-shot, then resume). If that happens,
  raise `HOLD_TIMEOUT_S` at the top of `katzlab/input/keyboard.py` -- 0.35s
  clears most terminals' defaults, but not necessarily all of them.
- **Losing terminal focus, or a terminal that doesn't repeat keys at all,
  makes every key read as released** within that same 0.35s window. This
  fails toward safe -- it looks exactly like letting go of everything, the
  same as unplugging a gamepad.

There is no analog control from a keyboard -- every axis is either exactly
`-1.0`, `0.0`, or `+1.0` (full rate in one direction, or nothing), never a
partial deflection. The deadzone and response-curve settings in
`config/system.yaml` (`input.deadzone`, `axis_expo`, `paired_axes`, etc.) are
gamepad-only concepts and have no effect in keyboard mode -- there's no
"magnitude" for them to shape.

## Restoring your terminal if something goes wrong

Keyboard mode puts the terminal into `cbreak` mode (no line echo, no line
buffering) for the duration of the run, and restores it automatically on exit
-- including on Ctrl-C. If the process is ever killed hard enough that this
restoration doesn't run (a `kill -9`, a crash), the terminal can be left
looking "broken" -- keys not echoing, no visible input. Nothing is actually
wrong with the terminal; run:

```bash
reset
```

and it will return to normal.

## What still needs verifying on real hardware

This has been tested as pure logic (16 unit tests feeding synthetic keystrokes
into the parser -- see `python/tests/test_keyboard_input.py`) but **not yet
driven against the physical rig.** Before relying on it:

1. Confirm arrow keys register as rotation/flexion on the actual terminal
   you'll use (`katzlab --input-source keyboard run --dry-run` -- logs what it
   would send without moving anything).
2. Confirm holding **F** produces one continuous fire rather than a stutter,
   given your terminal's actual key-repeat rate.
3. Confirm **X** stops things as fast as you need it to.
