"""Serial transport to the Teensy.

The firmware talks plain ASCII lines, so this layer stays deliberately dumb: it
opens the port, writes lines, and hands back lines. All protocol knowledge lives
in the driver modules.

The one piece of cleverness is :meth:`Link.wait_for`. Both firmware sketches are
request/response with a recognisable "I am ready for the next command" banner,
and the 3-DOF sketch blocks for the entire duration of a move. Synchronising on
that banner is what keeps Python from running ahead of a board that is busy
stepping a motor.
"""

from __future__ import annotations

import logging
import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Iterable

import serial
from serial.tools import list_ports

log = logging.getLogger(__name__)

# Teensy boards enumerate under PJRC's vendor id.
TEENSY_VID = 0x16C0


class LinkError(RuntimeError):
    """Serial link could not be opened, or the board stopped answering."""


class LinkTimeout(LinkError):
    """Expected response did not arrive inside the timeout."""


@dataclass(frozen=True)
class PortInfo:
    device: str
    description: str
    vid: int | None
    pid: int | None
    serial_number: str | None

    @property
    def is_teensy(self) -> bool:
        return self.vid == TEENSY_VID


def list_candidate_ports() -> list[PortInfo]:
    """Every serial port that could plausibly be a Teensy, best guess first."""
    found: list[PortInfo] = []
    for p in list_ports.comports():
        # Bluetooth and the internal debug console are never the rig.
        if "Bluetooth" in p.device or "debug-console" in p.device:
            continue
        found.append(
            PortInfo(
                device=p.device,
                description=p.description or "",
                vid=p.vid,
                pid=p.pid,
                serial_number=p.serial_number,
            )
        )
    found.sort(key=lambda i: (not i.is_teensy, i.device))
    return found


def resolve_port(requested: str) -> str:
    """Turn ``"auto"`` into a concrete device path."""
    if requested and requested != "auto":
        return requested

    candidates = list_candidate_ports()
    if not candidates:
        raise LinkError(
            "No serial ports found. Check that the Teensy is plugged in and that "
            "no other program (Arduino Serial Monitor, screen) is holding the port."
        )

    chosen = candidates[0]
    if not chosen.is_teensy:
        log.warning(
            "No Teensy VID (0x%04X) among %d port(s); falling back to %s (%s). "
            "Set link.port explicitly in system.yaml if this is wrong.",
            TEENSY_VID,
            len(candidates),
            chosen.device,
            chosen.description,
        )
    return chosen.device


class Link:
    """A line-oriented serial connection with a background reader.

    The reader thread exists so a blocking firmware move cannot fill the OS
    buffer and stall us, and so :meth:`recent_lines` can always show what the
    board said even when nobody was explicitly waiting for it.
    """

    def __init__(
        self,
        port: str,
        baud: int = 115200,
        *,
        read_timeout_s: float = 6.0,
        history: int = 500,
    ) -> None:
        self.port = port
        self.baud = baud
        self.read_timeout_s = read_timeout_s

        self._serial: serial.Serial | None = None
        self._lines: deque[str] = deque(maxlen=history)
        self._pending: deque[str] = deque()
        self._lock = threading.RLock()
        self._new_line = threading.Condition(self._lock)
        self._reader: threading.Thread | None = None
        self._closing = threading.Event()

    # -- lifecycle ---------------------------------------------------------

    def open(self, connect_timeout_s: float = 8.0) -> None:
        if self._serial is not None:
            return
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=0.1)
        except serial.SerialException as exc:
            raise LinkError(f"could not open {self.port}: {exc}") from exc

        self._closing.clear()
        self._reader = threading.Thread(
            target=self._read_loop, name=f"link-reader-{self.port}", daemon=True
        )
        self._reader.start()

        # Teensy reboots when the USB CDC port is opened; the sketches then print
        # a banner. Give that a moment so the first real command is not eaten.
        deadline = time.monotonic() + connect_timeout_s
        while time.monotonic() < deadline:
            if self._lines:
                break
            time.sleep(0.05)
        log.info("link open on %s at %d baud", self.port, self.baud)

    def close(self) -> None:
        self._closing.set()
        if self._reader is not None:
            self._reader.join(timeout=2.0)
            self._reader = None
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:  # noqa: BLE001 - closing must never raise
                pass
            self._serial = None
        log.info("link closed on %s", self.port)

    def __enter__(self) -> "Link":
        self.open()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    # -- reader thread -----------------------------------------------------

    def _read_loop(self) -> None:
        buffer = bytearray()
        while not self._closing.is_set():
            ser = self._serial
            if ser is None:
                break
            try:
                chunk = ser.read(256)
            except (serial.SerialException, OSError) as exc:
                log.error("serial read failed on %s: %s", self.port, exc)
                break

            if not chunk:
                continue

            buffer.extend(chunk)
            while b"\n" in buffer:
                raw, _, rest = buffer.partition(b"\n")
                buffer = bytearray(rest)
                line = raw.decode("utf-8", errors="replace").rstrip("\r")
                with self._new_line:
                    self._lines.append(line)
                    self._pending.append(line)
                    self._new_line.notify_all()

    # -- reading -----------------------------------------------------------

    def drain(self, predicate=None) -> list[str]:
        """Take received lines that are not yet consumed.

        With a ``predicate``, ONLY matching lines are removed; everything else
        stays queued for another reader. That matters whenever two drivers share
        one port: an unfiltered drain by either of them silently eats the
        other's replies, and the victim then times out holding a stale view of
        the hardware.
        """
        with self._lock:
            if predicate is None:
                out = list(self._pending)
                self._pending.clear()
                return out

            out, keep = [], []
            for line in self._pending:
                (out if predicate(line) else keep).append(line)
            self._pending.clear()
            self._pending.extend(keep)
            return out

    def recent_lines(self, count: int = 20) -> list[str]:
        with self._lock:
            return list(self._lines)[-count:]

    def wait_for(
        self,
        patterns: Iterable[str],
        timeout_s: float | None = None,
        *,
        collect: bool = True,
    ) -> tuple[str, list[str]]:
        """Block until a line contains one of ``patterns``.

        Returns the matching line and, when ``collect`` is set, everything read
        on the way there -- which is how the drivers harvest telemetry that the
        firmware prints during a move.
        """
        timeout_s = self.read_timeout_s if timeout_s is None else timeout_s
        compiled = [re.compile(p) for p in patterns]
        gathered: list[str] = []
        deadline = time.monotonic() + timeout_s

        with self._new_line:
            while True:
                while self._pending:
                    line = self._pending.popleft()
                    if collect:
                        gathered.append(line)
                    for rx in compiled:
                        if rx.search(line):
                            return line, gathered

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    # Read _lines directly: the condition's lock is already
                    # held here, so going through recent_lines() would nest.
                    tail = gathered[-5:] or list(self._lines)[-5:]
                    raise LinkTimeout(
                        f"timed out after {timeout_s:.1f}s waiting for "
                        f"{list(patterns)} on {self.port}. Last lines: {tail}"
                    )
                self._new_line.wait(timeout=min(remaining, 0.25))

    # -- writing -----------------------------------------------------------

    def write_line(self, text: str) -> None:
        ser = self._serial
        if ser is None:
            raise LinkError(f"link on {self.port} is not open")
        payload = (text.rstrip("\r\n") + "\n").encode("ascii", errors="replace")
        try:
            ser.write(payload)
            ser.flush()
        except (serial.SerialException, OSError) as exc:
            raise LinkError(f"serial write failed on {self.port}: {exc}") from exc
        log.debug(">> %s", text.strip())
