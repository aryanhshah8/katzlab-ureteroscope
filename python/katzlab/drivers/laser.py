"""Driver for the laser sketch (``shootingyesready_inoF-1.ino``).

Protocol: one character per line. The sketch echoes the command, acts, then
always prints a two-line status block::

    Laser enabled/ready/firing: 1/1/0
    Pin levels 8/9/10: 1/0/0

The second line is the synchronisation point, and the first is authoritative
state. Python mirrors the interlock rather than owning it -- the firmware still
enforces its own dwell timers and command ordering, and this class refuses to
send commands that the firmware would reject anyway.
"""

from __future__ import annotations

import logging
import re
import time
from dataclasses import dataclass

from ..config import LaserConfig
from ..transport import Link, LinkTimeout

log = logging.getLogger(__name__)

STATUS_TAIL = r"Pin levels 8/9/10:"

RE_STATE = re.compile(r"Laser enabled/ready/firing:\s*([01])/([01])/([01])")
RE_PINS = re.compile(r"Pin levels 8/9/10:\s*([01])/([01])/([01])")

# Single-character commands the sketch accepts.
CMD_ENABLE = "e"
CMD_READY = "r"
CMD_FIRE = "f"
CMD_PAUSE = "p"
CMD_STANDBY = "s"
CMD_STOP = "x"
CMD_STATUS = "?"


class LaserError(RuntimeError):
    """A laser command was refused, locally or by the firmware."""


@dataclass(frozen=True)
class LaserState:
    enabled: bool = False
    ready: bool = False
    firing: bool = False
    pin_ready: int = 8
    pin_fire1: int = 9
    pin_fire2: int = 10

    @property
    def armed(self) -> bool:
        return self.enabled and self.ready

    def describe(self) -> str:
        return (
            f"enabled={int(self.enabled)} ready={int(self.ready)} "
            f"firing={int(self.firing)} pins={self.pin_ready}/{self.pin_fire1}/{self.pin_fire2}"
        )

class LaserDriver:
    """Speaks the laser sketch's single-character protocol."""

    def __init__(self, link: Link | None, cfg: LaserConfig, *, dry_run: bool = False) -> None:
        self._link = link
        self._cfg = cfg
        self._dry_run = dry_run
        self._state = LaserState()
        self._synced = False

    @property
    def state(self) -> LaserState:
        return self._state

    # -- synchronisation ---------------------------------------------------

    def sync(self, timeout_s: float = 15.0) -> LaserState:
        """Read the boot banner and settle on a known state.

        Optional: it waits passively for the startup banner, and falls back to
        asking outright if that banner has already gone past. Commands do not
        depend on it -- each one waits for its own status block.
        """
        if self._dry_run:
            self._synced = True
            log.info("laser DRY RUN -- no serial link, no relay will move")
            return self._state

        try:
            line, gathered = self._link.wait_for([STATUS_TAIL], timeout_s=timeout_s)
            self._absorb(gathered + [line])
        except LinkTimeout:
            # No banner seen -- ask directly rather than assuming anything.
            log.debug("no laser boot banner; querying status")
            self._synced = True
            return self.status()
        self._synced = True
        log.info("laser firmware ready | %s", self._state.describe())
        return self._state

    # -- commands ----------------------------------------------------------

    def status(self) -> LaserState:
        return self._send(CMD_STATUS)

    def apply_fire_polarity(self) -> None:
        """Push the configured fire-channel polarity to the firmware.

        Sent as an absolute mask rather than the 3/4 toggles, which flip
        relative to whatever state the board is already in -- reconnecting
        without a reboot would invert the wrong way.
        """
        mask = (1 if self._cfg.fire_ch1_invert else 0) | (
            2 if self._cfg.fire_ch2_invert else 0
        )
        self._link.write_line(f"P{mask}")
        if mask:
            log.warning(
                "fire channels inverted (mask %d): an inverted channel is "
                "ENERGISED at rest and the machine may complain continuously",
                mask,
            )

    def enable(self) -> LaserState:
        return self._send(CMD_ENABLE)

    def ready(self) -> LaserState:
        return self._send(CMD_READY)

    def arm(self, *, attempts: int = 2) -> LaserState:
        """Run a deterministic Enable -> Ready sequence, and verify it took.

        Firing "inconsistently" comes from arming from an unknown starting
        state. The firmware's ``r`` is ignored unless ``e`` has been accepted,
        and ``r`` arms on the open->closed EDGE of the ready contact, not on its
        level -- so sending ``r`` while the contact is already closed does
        nothing at all. Both make a blind ``e`` then ``r`` a coin flip.

        So: relock first for a known state, then enable, then ready, checking
        after each step and retrying the edge once if it did not take.
        """
        for attempt in range(1, attempts + 1):
            # Guarantee the ready contact is OPEN, and hold it open long enough
            # for the machine to notice, before asking for the closing edge.
            #
            # This sends [s] DIRECTLY rather than calling standby(). The two are
            # not the same thing: standby() is the operator's button and honours
            # standby_drives_ready, which on this rig is false because holding
            # the contact open ARMS the machine. Arming still needs that open
            # half -- it just needs to pass through it rather than stop there.
            # Routing this through standby() meant that when standby stopped
            # driving ready, arming silently lost its edge and stopped latching,
            # which then made every fire fail for want of an armed machine.
            #
            # [s] parks the fire channels at the level they already sit at, so
            # unlike [x] it produces no fire-channel transition.
            self._send(CMD_STANDBY)
            time.sleep(self._cfg.ready_edge_open_s)

            # Deliberately NOT relocking first.
            #
            # [x] was here to guarantee a known starting state, but it drives
            # the FIRE channels as well as the ready contact -- and that
            # transition is enough to make the machine fire. Arming must not
            # touch the fire channels at all. The firmware's [r] already forces
            # its own open->close edge, so the ready edge is unambiguous
            # without disturbing anything else.
            state = self.enable()
            if not state.enabled:
                log.warning("arm attempt %d: enable not acknowledged", attempt)
                continue

            state = self.ready()
            if state.ready:
                log.info("laser armed on attempt %d | %s", attempt, state.describe())
                return state

            log.warning("arm attempt %d: ready did not latch", attempt)

        raise LaserError(
            f"could not arm the laser after {attempts} attempts "
            f"({self._state.describe()}).\n"
            "The firmware reports its own command state, not the Dornier's -- "
            "check the machine's panel. If the panel shows READY but this does "
            "not, or vice versa, the pin 8 sense is wrong: send [w] via "
            "'katzlab monitor' to flip it at runtime, then re-test."
        )

    def fire(self) -> LaserState:
        if self._cfg.require_ready_before_fire and not self._state.armed:
            raise LaserError(
                "refusing to fire: laser is not armed "
                f"({self._state.describe()}). Press Enable/Ready first."
            )

        state = self._send(CMD_FIRE)
        if not state.firing:
            log.warning("fire was not acknowledged | %s", state.describe())
        return state

    def pause(self) -> LaserState:
        """Return the fire channels to idle, keeping the arm state."""
        return self._send(CMD_PAUSE)

    def standby(self) -> LaserState:
        """Stop firing, and drop the software out of the armed state.

        Whether this also drives the READY contact depends on
        ``standby_drives_ready``. On a machine that arms on the contact's "open"
        level, driving it here ARMS rather than disarms -- so by default this
        only parks the fire channels. The software state still clears, so a
        subsequent fire is refused.
        """
        if self._cfg.standby_drives_ready:
            return self._send(CMD_STANDBY)

        state = self._send(CMD_PAUSE)
        self._state = LaserState(
            enabled=state.enabled,
            ready=False,
            firing=False,
            pin_ready=state.pin_ready,
            pin_fire1=state.pin_fire1,
            pin_fire2=state.pin_fire2,
        )
        return self._state

    def stop(self, *, timeout_s: float = 35.0, force_ready: bool = False) -> LaserState:
        """Full stop and relock. This is the laser half of an e-stop.

        Deliberately patient: the firmware does not read serial while a motion
        move is in flight, so a stop issued mid-move waits for that move. The
        default 10 s was short enough to time out during a long move and report
        a relock failure for a laser that was about to relock fine.
        """
        if self._cfg.standby_drives_ready or force_ready:
            return self._send(CMD_STOP, timeout_s=timeout_s)

        # Park the fire channels and lock the software without touching the
        # ready contact -- see standby(). The fire path is dead either way.
        state = self._send(CMD_PAUSE, timeout_s=timeout_s)
        self._state = LaserState(
            enabled=False,
            ready=False,
            firing=False,
            pin_ready=state.pin_ready,
            pin_fire1=state.pin_fire1,
            pin_fire2=state.pin_fire2,
        )
        return self._state

    # -- internals ---------------------------------------------------------

    def _send(self, command: str, *, timeout_s: float = 10.0) -> LaserState:
        if self._dry_run:
            return self._simulate(command)

        # Same reasoning as MotionDriver._exchange: never let a previous
        # command's status block satisfy this command's wait.
        self._link.drain()
        self._link.write_line(command)
        try:
            line, gathered = self._link.wait_for([STATUS_TAIL], timeout_s=timeout_s)
        except LinkTimeout as exc:
            self._synced = False
            raise LaserError(
                f"no status block after {timeout_s:.0f}s for laser command "
                f"{command!r}. Laser state is now UNKNOWN -- treat the machine "
                "as potentially armed and check the Dornier directly."
            ) from exc

        self._absorb(gathered + [line])
        self._synced = True
        log.debug("laser %r -> %s", command, self._state.describe())
        return self._state

    def _simulate(self, command: str) -> LaserState:
        """Run the interlock locally instead of driving relays. --dry-run only."""
        state = self._state
        if command == CMD_ENABLE:
            state = LaserState(True, state.ready)
        elif command == CMD_READY:
            state = LaserState(state.enabled, state.enabled)
        elif command == CMD_FIRE:
            state = LaserState(state.enabled, state.ready, state.firing)
        elif command == CMD_PAUSE:
            state = LaserState(state.enabled, state.ready, False)
        elif command == CMD_STANDBY:
            state = LaserState(state.enabled, False, False)
        elif command == CMD_STOP:
            state = LaserState(False, False, False)
        self._state = state
        log.info("DRY RUN laser %r -> %s", command, state.describe())
        return state

    def _absorb(self, lines: list[str]) -> None:
        state = self._state
        for text in lines:
            if match := RE_STATE.search(text):
                state = LaserState(
                    enabled=match.group(1) == "1",
                    ready=match.group(2) == "1",
                    firing=match.group(3) == "1",
                    pin_ready=state.pin_ready,
                    pin_fire1=state.pin_fire1,
                    pin_fire2=state.pin_fire2,
                )
            if match := RE_PINS.search(text):
                state = LaserState(
                    enabled=state.enabled,
                    ready=state.ready,
                    firing=state.firing,
                    pin_ready=int(match.group(1)),
                    pin_fire1=int(match.group(2)),
                    pin_fire2=int(match.group(3)),
                )
        self._state = state
