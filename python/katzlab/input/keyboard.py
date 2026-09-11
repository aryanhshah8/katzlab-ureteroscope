"""Controller replacement typed at a terminal keyboard.

Reads raw keystrokes from the terminal the process is running in -- no window,
no display server, no pygame. That is the right fit here: this program is
already terminal-driven end to end (bringup, monitor, the live status line all
print to the same terminal), and a rig that might be SSH'd into or run on a
bare TTY on the Mint machine cannot assume a GUI window exists to receive
keyboard focus, which pygame's keyboard API would need.

THE ONE THING A TERMINAL CANNOT GIVE US: key-release events
-------------------------------------------------------------
A gamepad tells you the instant a button comes up. A terminal only tells you
that a key went down -- if it is held, the operating system's own key-repeat
re-sends the same byte at some OS/terminal-dependent rate, and when you let go,
nothing arrives at all. There is no "key up" byte.

So "held" is emulated: every logical key has a last-seen timestamp, and it
counts as held for HOLD_TIMEOUT_S after the most recent byte for it arrived.
While a key is genuinely down, the OS's repeat keeps refreshing that timestamp
faster than the timeout expires, so it reads as one continuous hold -- exactly
what EdgeTracker downstream needs to turn Fire into a single press-and-hold
rather than sixty separate presses a second. Two consequences of this that are
worth knowing rather than being surprised by:

  - HOLD_TIMEOUT_S has to comfortably outlast the terminal's own "delay until
    repeat" setting, or a genuine hold reads as press-release-press instead of
    one continuous hold (harmless for Fire -- it just means an unwanted pause
    partway through -- but worth knowing about). 350 ms clears most terminals'
    defaults; if a specific machine's repeat delay is unusually slow, raise it.
  - Losing terminal focus, or the terminal not repeating at all, means every
    key reads as released within HOLD_TIMEOUT_S. That fails toward safe: it
    looks exactly like letting go of everything, not like holding it.

KEY LAYOUT
----------
    Up / Down       rotation            Left / Right    flexion
    A / D           linear travel
    R               Enable + Ready      S               Standby
    F               Fire (hold)         P               Pause
    H               Home (hold ~0.6s)   X               EMERGENCY STOP

Letters are case-insensitive -- this is meant to be driven by feel without
looking at the keyboard, and requiring Shift for some but not others would
only invite mistakes. Arrow keys are read as their standard ANSI escape
sequences (``ESC [ A/B/C/D``), which is what every common terminal sends.

R (arm) is not on the list the operator asked for. Fire refuses to do anything
until the laser is armed, so without a way to arm it, F would be permanently
dead -- R was added to make the layout actually usable, chosen because it is
not used by anything else here and reads naturally as "Ready".
"""

from __future__ import annotations

import atexit
import logging
import os
import select
import sys
import time

from .base import ControllerFrame, InputSource, InputUnavailable

log = logging.getLogger(__name__)

# How long a logical key stays "held" after its last byte arrived. Must clear
# the terminal's own key-repeat delay -- see the module docstring.
HOLD_TIMEOUT_S = 0.35

# Single-byte keys, case-folded to lowercase before lookup so Shift is never
# required. Named by the logical control they drive, not by the raw letter,
# so the mapping is legible without cross-referencing the docstring.
SINGLE_KEYS: dict[bytes, str] = {
    b"a": "linear_neg",
    b"d": "linear_pos",
    b"s": "standby",
    b"h": "home",
    b"x": "estop",
    b"p": "pause",
    b"f": "fire",
    b"r": "enable_ready",
}

# Standard ANSI arrow-key escape sequences (xterm and everything compatible
# with it, which covers macOS Terminal/iTerm and every Linux terminal emulator
# this rig is likely to run in).
ARROW_KEYS: dict[bytes, str] = {
    b"\x1b[A": "rot_pos",   # Up
    b"\x1b[B": "rot_neg",   # Down
    b"\x1b[C": "flex_pos",  # Right
    b"\x1b[D": "flex_neg",  # Left
}

AXIS_KEYS = {
    "linear": ("linear_pos", "linear_neg"),
    "rotation": ("rot_pos", "rot_neg"),
    "flexion": ("flex_pos", "flex_neg"),
}

BUTTON_KEYS = ("standby", "home", "estop", "pause", "fire", "enable_ready")


class KeyboardInputSource(InputSource):
    """Reads the process's own terminal as if it were a gamepad.

    Constructed with the same signature `create_input_source` already uses for
    every other backend (``controller`` plus the gamepad shaping kwargs) so it
    drops into the existing call sites unchanged. The gamepad-only arguments
    (deadzone, expo, per-axis overrides, stick pairing) are accepted and
    ignored: a key is either down or it is not, so there is nothing for a
    deadzone or a response curve to shape -- every axis this source reports is
    already exactly -1.0, 0.0 or +1.0.
    """

    def __init__(self, controller=None, **_gamepad_only_kwargs) -> None:
        self._controller = controller
        self._fd: int | None = None
        self._old_termios = None
        self._buffer = bytearray()
        self._last_seen: dict[str, float] = {}
        self._atexit_registered = False

    @property
    def name(self) -> str:
        return "Keyboard (terminal)"

    # -- lifecycle -----------------------------------------------------

    def open(self) -> None:
        if not sys.stdin.isatty():
            raise InputUnavailable(
                "keyboard input needs an interactive terminal -- stdin is not "
                "a tty (piped input, a background job, or a non-interactive "
                "shell all land here)"
            )

        import termios
        import tty

        self._fd = sys.stdin.fileno()
        self._old_termios = termios.tcgetattr(self._fd)

        # cbreak, not raw: it turns off line buffering and local echo (so
        # keystrokes reach us immediately, one at a time, without the user
        # seeing them typed into their own terminal) while leaving ISIG alone,
        # so Ctrl-C still raises KeyboardInterrupt exactly as it does with any
        # other input source. Raw mode would swallow that.
        tty.setcbreak(self._fd)

        # A crash between here and close() would otherwise strand the
        # operator's shell in cbreak mode -- no echo, no line editing, looking
        # broken until they know to run `reset`. This is cheap insurance.
        if not self._atexit_registered:
            atexit.register(self._restore_terminal)
            self._atexit_registered = True

        log.info(
            "keyboard input ready -- Up/Down rotation, Left/Right flexion, "
            "A/D linear, R ready, F fire, S standby, P pause, H home, X e-stop"
        )

    def close(self) -> None:
        self._restore_terminal()

    def _restore_terminal(self) -> None:
        if self._fd is None or self._old_termios is None:
            return
        import termios

        try:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_termios)
        except Exception:  # noqa: BLE001 - terminal teardown must not raise
            pass
        self._fd = None

    # -- reading ---------------------------------------------------------

    def poll(self) -> ControllerFrame:
        now = time.monotonic()
        self._drain(now)

        def held(name: str) -> bool:
            return (now - self._last_seen.get(name, float("-inf"))) <= HOLD_TIMEOUT_S

        axes = {}
        for axis_name, (pos_key, neg_key) in AXIS_KEYS.items():
            axes[axis_name] = (1.0 if held(pos_key) else 0.0) - (
                1.0 if held(neg_key) else 0.0
            )

        buttons = {name: held(name) for name in BUTTON_KEYS}

        return ControllerFrame(axes=axes, buttons=buttons, connected=True)

    def _drain(self, now: float) -> None:
        """Read every byte currently waiting, then parse what arrived.

        Split out from `poll()` so tests can feed it synthetic byte chunks
        directly instead of needing a real terminal and real keystrokes.
        """
        if self._fd is not None:
            while True:
                readable, _, _ = select.select([self._fd], [], [], 0)
                if not readable:
                    break
                chunk = os.read(self._fd, 64)
                if not chunk:
                    break
                self._buffer.extend(chunk)

        self._parse_buffer(now)

    def _parse_buffer(self, now: float) -> None:
        buf = self._buffer
        i = 0
        while i < len(buf):
            if buf[i] == 0x1B:  # ESC -- either an arrow key or a lone Escape
                remaining = len(buf) - i
                if remaining < 3:
                    # Could be the front of an arrow sequence whose later
                    # bytes have not arrived yet. Leave it for next time
                    # rather than misreading it as a bare Escape.
                    break
                seq = bytes(buf[i : i + 3])
                name = ARROW_KEYS.get(seq)
                if name is not None:
                    self._last_seen[name] = now
                i += 3
                continue

            key = bytes(buf[i : i + 1]).lower()
            name = SINGLE_KEYS.get(key)
            if name is not None:
                self._last_seen[name] = now
            i += 1

        del buf[:i]
