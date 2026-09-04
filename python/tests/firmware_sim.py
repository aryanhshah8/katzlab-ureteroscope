"""A stand-in for the merged firmware, speaking its exact serial dialect.

Lets the whole Python stack be exercised without the rig powered up -- and,
more usefully, pins the wire format down so a change to either side that breaks
the other shows up here rather than on the bench.

The responses are copied from the sketches' own Serial.print calls, including
the parts that matter for parsing: the prompt banner, the final-summary lines
the motion driver reads position out of, and the two-line laser status block.
"""

from __future__ import annotations

import os
import pty
import select
import threading
import time
import tty


class FirmwareSimulator:
    """Serves one end of a pty pair as if it were the Teensy."""

    def __init__(self, *, terse: bool = False, move_delay_s: float = 0.0) -> None:
        self._master_fd, self._slave_fd = pty.openpty()
        self.port = os.ttyname(self._slave_fd)

        # A pty echoes by default and does line discipline on top. Both are
        # wrong here: the echo feeds our own output back as input (an endless
        # write/parse loop), and the line discipline rewrites the bytes. Raw
        # mode makes the pair behave like the serial link it stands in for.
        tty.setraw(self._master_fd)
        tty.setraw(self._slave_fd)

        self.terse = terse
        self.move_delay_s = move_delay_s

        # Mirrors of the firmware's own state.
        self.linear_mm = 0.0
        self.rotation_deg = 0.0
        self.flexion_tip_deg = 0.0
        self.enabled = False
        self.ready = False
        self.firing = False

        self.received: list[str] = []
        self.reject_next: str | None = None

        self._running = False
        self._thread: threading.Thread | None = None

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> "FirmwareSimulator":
        self._running = True
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        return self

    def boot(self) -> None:
        """Emit the startup banner.

        Deliberately separate from :meth:`start`. pyserial flushes the input
        buffer when it opens a port, so anything sent before the host connects
        is lost -- and that matches the real board, which reboots when the USB
        CDC port is opened and only then prints. Call this after opening the
        link.
        """
        self._boot_banner()

    def stop(self) -> None:
        # The reader polls with a short select timeout rather than blocking in
        # os.read, so clearing the flag is enough to retire it. Closing a pty
        # master out from under a blocked reader hangs on macOS, so the order
        # here matters: retire the thread first, then close.
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        for fd in (self._master_fd, self._slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass

    def __enter__(self) -> "FirmwareSimulator":
        return self.start()

    def __exit__(self, *exc_info: object) -> None:
        self.stop()

    # -- output ------------------------------------------------------------

    def _emit(self, *lines: str) -> None:
        payload = "".join(f"{line}\r\n" for line in lines).encode()
        os.write(self._master_fd, payload)

    def _boot_banner(self) -> None:
        self._emit(
            "=== COMBINED MOTION + LASER BUILD ===",
            "Motion: three numbers, or home/h",
            "Laser : e r f p s x 1 2 3 4 o g t y w ?",
        )
        self._laser_status()
        self._prompt()

    def _prompt(self) -> None:
        self._emit(
            "",
            "POSITIONING: Linear = RELATIVE | Rotation = RELATIVE | Flexion = ABSOLUTE",
        )
        if not self.terse:
            self._emit(
                "Enter Linear Additional(mm) Rotation Additional(deg) Flexion Target(deg):",
                f"Current absolute linear position: {self.linear_mm:.3f} mm",
                f"Current mapped rotation position: {self.rotation_deg:.2f} deg",
                "Linear range allowed: 0.0 mm to 177.0 mm",
                "Return to home state with: home OR h",
            )

    def _movement_summary(self) -> None:
        self._emit(
            "",
            "3-DOF Movement Complete",
            f"Final Linear Absolute Position: {self.linear_mm:.3f} mm",
            "Final Linear Relative Motion, Motor Deg, Count: 0.000, 0.00, 0",
            "Final Rotation Motor Deg, Ureteroscope Deg, Count: 0.00, 0.00, 0",
            f"Final Rotation Map Position: {self.rotation_deg:.2f} deg",
            f"Final Flexion Step, Motor Deg, Tip Deg: 2048, 0.00, {self.flexion_tip_deg:.2f}",
        )

    def _laser_status(self) -> None:
        # Pin 8 is on the relay's NC terminal (READY_ON_NC = true in the
        # firmware), so its level is inverted relative to the ready state:
        # coil released / pin LOW closes the contact and ARMS the machine,
        # coil energised / pin HIGH opens it and is standby.
        pin8 = 0 if self.ready else 1
        self._emit(
            f"Laser enabled/ready/firing: {int(self.enabled)}/{int(self.ready)}/{int(self.firing)}",
            f"Pin levels 8/9/10: {pin8}/{int(self.firing)}/{int(self.firing)}",
        )

    # -- input -------------------------------------------------------------

    def _serve(self) -> None:
        buffer = b""
        while self._running:
            try:
                readable, _, _ = select.select([self._master_fd], [], [], 0.05)
            except (OSError, ValueError):
                break
            if not readable:
                continue
            try:
                chunk = os.read(self._master_fd, 256)
            except OSError:
                break
            if not chunk:
                continue
            buffer += chunk
            while b"\n" in buffer:
                raw, _, buffer = buffer.partition(b"\n")
                line = raw.decode(errors="replace").strip()
                if line:
                    self._handle(line)

    LASER_LETTERS = set("erfpsx1234ogtyw?")

    def _handle(self, line: str) -> None:
        self.received.append(line)

        if line == "q":
            self.terse = not self.terse
            self._emit(f"Terse output: {'ON' if self.terse else 'OFF'}")
            self._prompt()
            return

        if len(line) == 1 and line in self.LASER_LETTERS:
            self._handle_laser(line)
            self._laser_status()
            self._prompt()
            return

        self._handle_motion(line)

    def _handle_laser(self, command: str) -> None:
        if command == "e":
            self.enabled = True
        elif command == "r":
            if self.enabled:
                self.ready = True
        elif command == "f":
            if self.enabled and self.ready:
                self.firing = True
        elif command == "p":
            self.firing = False
        elif command == "s":
            self.ready = False
            self.firing = False
        elif command == "x":
            self.enabled = self.ready = self.firing = False

    def _handle_motion(self, line: str) -> None:
        if self.reject_next:
            self._emit("", self.reject_next)
            self.reject_next = None
            self._prompt()
            return

        if line.lower() in ("home", "h"):
            self.linear_mm = 0.0
            self.rotation_deg = 0.0
            self.flexion_tip_deg = 0.0
            self._movement_summary()
            self._prompt()
            return

        parts = line.split()
        if len(parts) != 3:
            self._emit("Wrong Format")
            self._prompt()
            return

        try:
            linear, rotation, flexion = (float(p) for p in parts)
        except ValueError:
            self._emit("Wrong Format")
            self._prompt()
            return

        if not 0.0 <= self.linear_mm + linear <= 177.0:
            self._emit("", "Linear Range Error")
            self._prompt()
            return
        if not -270.0 <= flexion <= 270.0:
            self._emit("", "Flexion Range Error")
            self._prompt()
            return

        if self.move_delay_s:
            time.sleep(self.move_delay_s)

        self.linear_mm += linear
        self.rotation_deg += rotation
        self.flexion_tip_deg = flexion
        self._movement_summary()
        self._prompt()
