"""Stand-in for Motion_Velocity_Server.ino.

Models the parts the driver depends on: commands are accepted without blocking,
velocity ramps toward its target, position integrates from that velocity, and
telemetry is streamed on a period.
"""

from __future__ import annotations

import os
import pty
import select
import threading
import time
import tty


class VelocitySimulator:
    LASER_LETTERS = set("erfpsx1234ow")

    def __init__(self, *, accel_mm_s2: float = 40.0) -> None:
        self._master_fd, self._slave_fd = pty.openpty()
        self.port = os.ttyname(self._slave_fd)
        tty.setraw(self._master_fd)
        tty.setraw(self._slave_fd)

        self.accel = accel_mm_s2
        self.target_linear = 0.0
        self.current_linear = 0.0
        self.linear_mm = 0.0
        self.target_rotation = 0.0
        self.current_rotation = 0.0
        self.rotation_deg = 0.0
        self.flexion_deg = 0.0
        self.homing = False
        self.enabled = self.ready = self.firing = False

        self.telemetry_ms = 100
        self.flexion_deadband = 0.25
        self.fire_polarity = 0
        self.received: list[str] = []
        self._running = False
        self._thread: threading.Thread | None = None

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> "VelocitySimulator":
        self._running = True
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        return self

    def boot(self) -> None:
        self._emit(
            "",
            "=== MOTION VELOCITY SERVER  BUILD VEL-1 ===",
            "Flexion servo: connected",
        )

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        for fd in (self._master_fd, self._slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass

    def __enter__(self) -> "VelocitySimulator":
        return self.start()

    def __exit__(self, *exc: object) -> None:
        self.stop()

    # -- output ------------------------------------------------------------

    def _emit(self, *lines: str) -> None:
        os.write(self._master_fd, "".join(f"{l}\r\n" for l in lines).encode())

    def _telemetry(self) -> None:
        self._emit(
            f"TEL L{self.linear_mm:.3f} R{self.rotation_deg:.2f} "
            f"F{self.flexion_deg:.2f} VL{self.current_linear:.3f} "
            f"VR{self.current_rotation:.2f} "
            f"E{int(self.enabled)} Y{int(self.ready)} G{int(self.firing)} "
            f"H{int(self.homing)}"
        )

    def _laser_status(self) -> None:
        self._emit(
            f"Laser enabled/ready/firing: {int(self.enabled)}/{int(self.ready)}/{int(self.firing)}",
            f"Pin levels 8/9/10: {0 if self.ready else 1}/{int(self.firing)}/{int(self.firing)}",
        )

    # -- physics -----------------------------------------------------------

    def _integrate(self, dt: float) -> None:
        if self.homing:
            self.target_linear = max(-8.0, min(8.0, -self.linear_mm * 1.2))
            self.target_rotation = max(-90.0, min(90.0, -self.rotation_deg * 12.0))
            if abs(self.linear_mm) <= 0.05 and abs(self.rotation_deg) <= 0.5:
                self.homing = False
                self.target_linear = self.target_rotation = 0.0
                self.current_linear = self.current_rotation = 0.0
                self._emit("HOME complete")

        for name in ("linear", "rotation"):
            cur = getattr(self, f"current_{name}")
            tgt = getattr(self, f"target_{name}")
            rate = (self.accel if name == "linear" else self.accel * 10) * dt
            if cur < tgt:
                cur = min(tgt, cur + rate)
            elif cur > tgt:
                cur = max(tgt, cur - rate)
            setattr(self, f"current_{name}", cur)

        self.linear_mm = max(0.0, min(177.0, self.linear_mm + self.current_linear * dt))
        self.rotation_deg += self.current_rotation * dt

    # -- serve -------------------------------------------------------------

    def _serve(self) -> None:
        buf = b""
        last = time.monotonic()
        last_tel = 0.0
        while self._running:
            try:
                readable, _, _ = select.select([self._master_fd], [], [], 0.01)
            except (OSError, ValueError):
                break

            now = time.monotonic()
            self._integrate(min(0.25, now - last))
            last = now

            if readable:
                try:
                    chunk = os.read(self._master_fd, 256)
                except OSError:
                    break
                buf += chunk
                while b"\n" in buf:
                    raw, _, buf = buf.partition(b"\n")
                    line = raw.decode(errors="replace").strip()
                    if line:
                        self._handle(line)

            if self.telemetry_ms and (now - last_tel) * 1000 >= self.telemetry_ms:
                last_tel = now
                self._telemetry()

    def _handle(self, line: str) -> None:
        self.received.append(line)
        head, rest = line[0], line[1:]

        if len(line) == 1 and line in self.LASER_LETTERS:
            if line == "e":
                self.enabled = True
            elif line == "r":
                self.ready = self.enabled
            elif line == "f":
                self.firing = self.enabled and self.ready
            elif line == "p":
                self.firing = False
            elif line == "s":
                self.ready = self.firing = False
            elif line == "x":
                self.enabled = self.ready = self.firing = False
            self._laser_status()
            return

        if head in "Ll":
            self.homing = False
            self.target_linear = max(-12.0, min(12.0, float(rest)))
        elif head == "R":
            self.homing = False
            self.target_rotation = max(-120.0, min(120.0, float(rest)))
        elif head == "F":
            self.flexion_deg = max(-270.0, min(270.0, float(rest)))
        elif head == "S":
            self.homing = False
            self.target_linear = self.target_rotation = 0.0
            self.current_linear = self.current_rotation = 0.0
            self._emit("STOPPED")
        elif head == "Z":
            self.linear_mm = self.rotation_deg = 0.0
            self._emit("ZEROED")
        elif head == "H":
            self.homing = True
            self._emit("HOMING")
        elif head == "P":
            self.fire_polarity = int(rest)
        elif head == "D":
            self.flexion_deadband = float(rest)
        elif head == "T":
            self.telemetry_ms = int(rest or 0)
        elif head == "?":
            self._telemetry()
            self._laser_status()
