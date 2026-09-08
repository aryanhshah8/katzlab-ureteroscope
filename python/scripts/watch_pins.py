#!/usr/bin/env python3
"""Watch the laser relay pins and print every transition, with timing.

Answers the one question nothing has actually measured: when you press Fire,
do pins 9 and 10 physically change? Everything so far has confirmed the
software SENDS the command and that the relays CAN click. Neither shows what
the pins do during a real fire, while armed, in the live control loop.

  - transitions appear the instant they happen, with a millisecond timestamp
  - the gap between channel 1 and channel 2 is measured, so a configured lead
    can be confirmed rather than assumed
  - read-only: this never sends a laser command

Run it in a SECOND terminal while 'katzlab bringup' runs in the first.
Both can share the port -- this only polls status.

    python scripts/watch_pins.py
"""

from __future__ import annotations

import re
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from katzlab.transport import Link, resolve_port

RE_PINS = re.compile(r"Pin levels 8/9/10:\s*([01])/([01])/([01])")
RE_STATE = re.compile(r"Laser enabled/ready/firing:\s*([01])/([01])/([01])")

BOLD, DIM, GREEN, YELLOW, RED, OFF = (
    "\033[1m", "\033[2m", "\033[32m", "\033[33m", "\033[31m", "\033[0m",
)


def describe(pins: tuple[int, int, int]) -> str:
    ready, f1, f2 = pins
    bits = [
        f"pin8 {'LOW  (ready CLOSED = armed)' if ready == 0 else 'HIGH (ready open = standby)'}",
        f"pin9 {f1}",
        f"pin10 {f2}",
    ]
    return "  ".join(bits)


def main() -> int:
    port = resolve_port("auto")
    link = Link(port, 115200, read_timeout_s=5.0)
    link.open(connect_timeout_s=8.0)
    time.sleep(2.5)
    link.drain()

    print(f"\n{BOLD}WATCHING LASER PINS{OFF}  on {port}")
    print(f"{DIM}Read-only -- never sends a laser command.{OFF}")
    print(f"{DIM}Press Fire on the controller and watch pins 9 and 10.{OFF}\n")

    last_pins = None
    last_state = None
    fire_edge_at: dict[int, float] = {}
    started = time.monotonic()

    try:
        while True:
            link.write_line("?")
            time.sleep(0.06)

            for line in link.drain():
                if match := RE_STATE.search(line):
                    state = tuple(int(g) for g in match.groups())
                    if state != last_state:
                        e, r, f = state
                        print(f"  {time.monotonic()-started:7.3f}s  "
                              f"{YELLOW}state{OFF}  enabled={e} ready={r} firing={f}")
                        last_state = state

                if match := RE_PINS.search(line):
                    pins = tuple(int(g) for g in match.groups())
                    if pins == last_pins:
                        continue
                    now = time.monotonic()

                    changed = []
                    if last_pins is not None:
                        for idx, name in ((0, "pin8"), (1, "pin9"), (2, "pin10")):
                            if pins[idx] != last_pins[idx]:
                                changed.append(f"{name} {last_pins[idx]}->{pins[idx]}")
                                if idx in (1, 2):
                                    fire_edge_at[idx] = now

                    colour = GREEN if pins[1] or pins[2] else DIM
                    print(f"  {now-started:7.3f}s  {colour}pins {pins[0]}/{pins[1]}/{pins[2]}{OFF}"
                          f"   {DIM}{describe(pins)}{OFF}"
                          + (f"   {BOLD}[{', '.join(changed)}]{OFF}" if changed else ""))

                    # If both fire channels moved, report the actual gap.
                    if 1 in fire_edge_at and 2 in fire_edge_at:
                        gap_ms = abs(fire_edge_at[1] - fire_edge_at[2]) * 1000
                        if gap_ms > 0:
                            print(f"           {DIM}measured ch1<->ch2 gap: "
                                  f"{gap_ms:.0f} ms{OFF}")
                        fire_edge_at.clear()

                    last_pins = pins

            time.sleep(0.04)

    except KeyboardInterrupt:
        print(f"\n{DIM}stopped{OFF}")
    finally:
        link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
