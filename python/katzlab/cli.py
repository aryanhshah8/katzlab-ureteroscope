"""Command line entry point.

    python -m katzlab ports        list serial ports and flag which look like Teensys
    python -m katzlab selftest     check config, link and controller without moving anything
    python -m katzlab monitor      passive serial monitor
    python -m katzlab run          the joystick control loop
"""

from __future__ import annotations

import argparse
import logging
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

from .config import (
    ConfigError,
    SystemConfig,
    load_controller_config,
    load_system_config,
)
from .control import ControlLoop
from .drivers import LaserDriver, MotionDriver, VelocityMotionDriver
from .recorder import SessionRecorder
from .input import InputUnavailable, create_input_source
from .transport import Link, LinkError, list_candidate_ports, resolve_port

log = logging.getLogger("katzlab")


def setup_logging(level: str, log_dir: str | None = None) -> None:
    handlers: list[logging.Handler] = [logging.StreamHandler(sys.stderr)]

    if log_dir:
        directory = Path(__file__).resolve().parent.parent / log_dir
        directory.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        handlers.append(logging.FileHandler(directory / f"session-{stamp}.log"))

    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
        handlers=handlers,
    )


# --------------------------------------------------------------------------
# link construction
# --------------------------------------------------------------------------


def open_links(cfg: SystemConfig) -> tuple[Link, Link | None]:
    """Return ``(motion_link, laser_link)``.

    In ``single`` mode both drivers share one link, and ``laser_link`` is the
    same object -- which is exactly why a laser command has to wait behind an
    in-flight motion command on that setup.
    """
    if cfg.link.mode == "single":
        port = resolve_port(cfg.link.port)
        link = Link(port, cfg.link.baud, read_timeout_s=cfg.link.read_timeout_s)
        link.open(cfg.link.connect_timeout_s)
        return link, link

    motion_port = resolve_port(cfg.link.motion_port)
    laser_port = resolve_port(cfg.link.laser_port)
    if motion_port == laser_port:
        raise LinkError(
            f"split mode needs two distinct ports but both resolved to "
            f"{motion_port}. Set link.motion_port and link.laser_port "
            "explicitly in system.yaml."
        )

    motion_link = Link(motion_port, cfg.link.baud, read_timeout_s=cfg.link.read_timeout_s)
    laser_link = Link(laser_port, cfg.link.baud, read_timeout_s=cfg.link.read_timeout_s)
    motion_link.open(cfg.link.connect_timeout_s)
    laser_link.open(cfg.link.connect_timeout_s)
    return motion_link, laser_link


def configure_firmware(link: Link, cfg: SystemConfig) -> None:
    """Put the firmware into host mode and set the step rate.

    [q] switches to terse output and makes flexion fire-and-forget; [v<us>] sets
    the STEP pulse half-period. Both are runtime-only and reset when the board
    reboots, so they have to be re-sent on every connection.
    """
    link.drain()
    link.write_line("q")
    time.sleep(0.3)

    link.write_line(f"v{cfg.motion.step_delay_us}")
    time.sleep(0.3)

    replies = link.drain()
    for line in replies:
        if "Terse output" in line or "Step delay" in line:
            log.info("firmware: %s", line.strip())

    ceiling = (1_000_000.0 / (2.0 * cfg.motion.step_delay_us)) * 4.0 / 1600.0
    if cfg.motion.linear.max_rate_mm_s > ceiling:
        log.warning(
            "motion.linear.max_rate_mm_s is %.2f mm/s but step_delay_us=%d only "
            "delivers %.2f mm/s -- the extra will never materialise",
            cfg.motion.linear.max_rate_mm_s,
            cfg.motion.step_delay_us,
            ceiling,
        )


# --------------------------------------------------------------------------
# commands
# --------------------------------------------------------------------------


def cmd_ports(_args, _cfg: SystemConfig) -> int:
    ports = list_candidate_ports()
    if not ports:
        print("No serial ports found.")
        print("Plug in the Teensy, and close any Arduino Serial Monitor holding it.")
        return 1

    print(f"{'DEVICE':<28} {'VID:PID':<12} {'TEENSY':<7} DESCRIPTION")
    print("-" * 78)
    for p in ports:
        vid_pid = f"{p.vid:04X}:{p.pid:04X}" if p.vid and p.pid else "-"
        print(f"{p.device:<28} {vid_pid:<12} {'yes' if p.is_teensy else 'no':<7} {p.description}")
    return 0


def cmd_monitor(args, cfg: SystemConfig) -> int:
    port = resolve_port(args.port or cfg.link.port)
    print(f"Monitoring {port} at {cfg.link.baud} baud. Ctrl-C to stop.\n")

    with Link(port, cfg.link.baud) as link:
        try:
            while True:
                for line in link.drain():
                    print(line)
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\nStopped.")
    return 0


def cmd_selftest(_args, cfg: SystemConfig) -> int:
    """Check every piece without commanding any motion."""
    ok = True

    print("\n1. Controller configuration")
    try:
        controller = load_controller_config()
        print(f"   OK   calibrated for {controller.name!r}")
    except ConfigError as exc:
        print(f"   FAIL {exc}")
        return 1

    print("\n2. Controller device")
    try:
        source = create_input_source(
            cfg.input.source,
            controller,
            deadzone=cfg.input.deadzone,
            expo=cfg.input.expo,
            axis_expo=cfg.input.axis_expo,
            axis_deadzone=cfg.input.axis_deadzone,
            paired_axes=cfg.input.paired_axes,
        )
        source.open()
        frame = source.poll()
        print(f"   OK   {source.name}, connected={frame.connected}")
        source.close()
    except (InputUnavailable, TypeError) as exc:
        print(f"   FAIL {exc}")
        ok = False

    print("\n3. Serial link")
    motion_link = laser_link = None
    try:
        motion_link, laser_link = open_links(cfg)
        print(f"   OK   motion on {motion_link.port}")
        if laser_link is not None and laser_link is not motion_link:
            print(f"   OK   laser on {laser_link.port}")
        elif laser_link is motion_link:
            print("   OK   laser shares the motion link (single-board mode)")

        print("\n4. Firmware handshake")
        motion = MotionDriver(motion_link, cfg.motion)
        try:
            motion.sync(timeout_s=20.0)
            print(f"   OK   motion firmware | {motion.state.describe()}")
        except Exception as exc:  # noqa: BLE001 - report, do not crash the test
            print(f"   FAIL motion firmware did not present its prompt: {exc}")
            print("        Is the merged Combined_Motion_Laser sketch flashed?")
            ok = False

        if laser_link is not None:
            laser = LaserDriver(laser_link, cfg.laser)
            try:
                state = laser.status()
                print(f"   OK   laser firmware | {state.describe()}")
                if state.armed:
                    print("   WARN laser reports ARMED at startup -- check the Dornier")
            except Exception as exc:  # noqa: BLE001
                print(f"   FAIL laser firmware did not answer: {exc}")
                ok = False
    except LinkError as exc:
        print(f"   FAIL {exc}")
        ok = False
    finally:
        if motion_link is not None:
            motion_link.close()
        if laser_link is not None and laser_link is not motion_link:
            laser_link.close()

    print("\n" + ("All checks passed." if ok else "One or more checks FAILED."))
    return 0 if ok else 1


def cmd_run(args, cfg: SystemConfig) -> int:
    controller = load_controller_config()

    source = create_input_source(
        cfg.input.source,
        controller,
        deadzone=cfg.input.deadzone,
        expo=cfg.input.expo,
    )
    source.open()

    if args.dry_run:
        motion_link = laser_link = None
        motion = MotionDriver(None, cfg.motion, dry_run=True)
        motion.sync()
        laser = None if args.no_laser else LaserDriver(None, cfg.laser, dry_run=True)
        if laser is not None:
            laser.sync()
    else:
        motion_link, laser_link = open_links(cfg)

        motion = make_motion_driver(motion_link, cfg)
        motion.sync(timeout_s=20.0)
        if cfg.motion.protocol == "chunked":
            configure_firmware(motion_link, cfg)

        laser = None
        if laser_link is not None and not args.no_laser:
            laser = LaserDriver(laser_link, cfg.laser)
            laser.sync()
            laser.apply_fire_polarity()
            if cfg.safety.start_disarmed:
                laser.stop()
                log.info("laser forced to disarmed state at startup")

    print_banner(cfg, source.name, laser is not None, dry_run=args.dry_run)

    loop = ControlLoop(cfg, source, motion, laser)
    loop.debug_buttons = getattr(args, "debug_buttons", False)
    loop.recorder = start_recorder(
        cfg, cfg.logging.record_motion and not getattr(args, "no_record", False)
    )
    status = None if getattr(args, "no_status", False) else make_status_line(
        cfg, laser is not None
    )
    try:
        loop.run(status_callback=status)
    finally:
        print()
        if loop.recorder is not None:
            loop.recorder.close()
        source.close()
        if motion_link is not None:
            motion_link.close()
        if laser_link is not None and laser_link is not motion_link:
            laser_link.close()

    stats = loop.stats
    print(
        f"\nSession: {stats.uptime_s:.0f}s | "
        f"{stats.motion_commands} motion, {stats.laser_commands} laser commands | "
        f"{stats.estops} e-stop(s) | {stats.errors} error(s)"
    )
    return 0


# --------------------------------------------------------------------------
# bringup: the one command
# --------------------------------------------------------------------------

FQBN = "teensy:avr:teensy41"
FIRMWARE_DIR = Path(__file__).resolve().parent.parent.parent / "firmware"
SKETCHES = {
    "velocity": FIRMWARE_DIR / "Motion_Velocity_Server",
    "chunked": FIRMWARE_DIR / "Combined_Motion_Laser",
}


def make_motion_driver(link, cfg: SystemConfig):
    """Return the driver matching motion.protocol."""
    if cfg.motion.protocol == "velocity":
        return VelocityMotionDriver(link, cfg.motion)
    return MotionDriver(link, cfg.motion)


def _ask(question: str, *, assume_yes: bool) -> bool:
    if assume_yes:
        print(f"  {question} [assumed yes]")
        return True
    try:
        return input(f"  {question} [y/N] ").strip().lower() in ("y", "yes")
    except (EOFError, KeyboardInterrupt):
        print()
        return False


def _wait_for_teensy(timeout_s: float = 30.0) -> str | None:
    """Wait for a Teensy port to appear. It re-enumerates after a flash."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for port in list_candidate_ports():
            if port.is_teensy:
                return port.device
        time.sleep(0.5)
    return None


def cmd_bringup(args, cfg: SystemConfig) -> int:
    """Flash, verify, and hand over to the control loop, in one go."""
    print("\n" + "=" * 68)
    print("  KATZLAB BRINGUP")
    print("=" * 68)

    # -- 1. the two things software cannot check for itself ----------------
    print("\n1. Safety checks (these are physical -- I cannot verify them)")
    if not _ask(
        "Is the laser RELAY SUPPLY switched OFF? "
        "(pin 8 is on NC; flashing arms the laser)",
        assume_yes=args.yes,
    ):
        print("\n  Stopping. Switch the relay supply off, then run this again.")
        return 1

    if not _ask(
        "Is the carriage at its MECHANICAL HOME and the scope flat at rotation "
        "neutral? (boot position becomes absolute 0.00 mm)",
        assume_yes=args.yes,
    ):
        print("\n  Stopping. Set the mechanical zero, then run this again.")
        return 1

    # -- 2. flash ----------------------------------------------------------
    if args.skip_flash:
        print("\n2. Flash  [skipped]")
    else:
        print("\n2. Flash the merged firmware")
        if shutil.which("arduino-cli") is None:
            print("   FAIL arduino-cli not found. Install it with: brew install arduino-cli")
            return 1

        port = _wait_for_teensy(timeout_s=10.0)
        if port is None:
            print("   FAIL no Teensy found. Plug its micro-USB into this Mac.")
            return 1
        print(f"   found Teensy on {port}")

        sketch = SKETCHES[cfg.motion.protocol]
        print(f"   sketch: {sketch.name}  (protocol: {cfg.motion.protocol})")
        result = subprocess.run(
            ["arduino-cli", "upload", "--fqbn", FQBN, "-p", port, str(sketch)],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            print("   FAIL upload failed:")
            print("   " + (result.stderr or result.stdout).strip().replace("\n", "\n   "))
            return 1
        print("   uploaded")

        print("   waiting for the board to come back...")
        if _wait_for_teensy(timeout_s=30.0) is None:
            print("   FAIL board did not re-enumerate after flashing")
            return 1
        time.sleep(2.0)      # let setup() finish its diagnostics

    # -- 3. verify ---------------------------------------------------------
    print("\n3. Verify")
    controller = load_controller_config()
    print(f"   OK   controller mapping: {controller.name}")

    source = create_input_source(
        cfg.input.source,
        controller,
        deadzone=cfg.input.deadzone,
        expo=cfg.input.expo,
        axis_expo=cfg.input.axis_expo,
            axis_deadzone=cfg.input.axis_deadzone,
            paired_axes=cfg.input.paired_axes
    )
    source.open()
    print(f"   OK   controller connected: {source.name}")

    motion_link, laser_link = open_links(cfg)
    print(f"   OK   serial link on {motion_link.port}")

    motion = make_motion_driver(motion_link, cfg)
    try:
        motion.sync(timeout_s=25.0)
    except Exception as exc:  # noqa: BLE001
        print(f"   FAIL motion firmware did not answer: {exc}")
        source.close()
        motion_link.close()
        return 1
    print(f"   OK   motion firmware ({cfg.motion.protocol}) | {motion.state.describe()}")

    if cfg.motion.protocol == "chunked":
        configure_firmware(motion_link, cfg)

    laser = None
    if not args.no_laser:
        laser = LaserDriver(laser_link, cfg.laser)
        state = laser.status()
        print(f"   OK   laser firmware | {state.describe()}")
        laser.apply_fire_polarity()
        laser.stop()
        print("   OK   laser forced to disarmed state")

    # -- 4. power up -------------------------------------------------------
    print("\n4. Power up the actuators")
    if not _ask("Motor and servo supplies ON, all grounds common?", assume_yes=args.yes):
        print("\n  Stopping before anything can move.")
        source.close()
        motion_link.close()
        return 1

    if laser is not None and not _ask(
        "Relay supply ON now? (the Teensy is running and holding pin 8 at standby)",
        assume_yes=args.yes,
    ):
        print("   continuing with the laser DISABLED for this session")
        laser = None

    # -- 5. go -------------------------------------------------------------
    print_banner(cfg, source.name, laser is not None)

    loop = ControlLoop(cfg, source, motion, laser)
    loop.debug_buttons = getattr(args, "debug_buttons", False)
    loop.recorder = start_recorder(
        cfg, cfg.logging.record_motion and not getattr(args, "no_record", False)
    )
    status = None if getattr(args, "no_status", False) else make_status_line(
        cfg, laser is not None
    )
    try:
        loop.run(status_callback=status)
    finally:
        print()
        if loop.recorder is not None:
            loop.recorder.close()
        source.close()
        motion_link.close()
        if laser_link is not None and laser_link is not motion_link:
            laser_link.close()

    stats = loop.stats
    print(
        f"\nSession: {stats.uptime_s:.0f}s | "
        f"{stats.motion_commands} motion, {stats.laser_commands} laser commands | "
        f"{stats.estops} e-stop(s) | {stats.errors} error(s)"
    )
    return 0


def cmd_bench(args, cfg: SystemConfig) -> int:
    """Measure the linear axis's real throughput.

    Everything else here is arithmetic on paper. This is the only thing that
    reports what the carriage actually does, which is what matters when the
    complaint is "it barely moves".
    """
    print("\n" + "=" * 68)
    print("  LINEAR THROUGHPUT BENCHMARK")
    print("=" * 68)
    print(f"  This MOVES the carriage up to {args.distance:.1f} mm and back.")
    print("  Make sure the travel is clear.\n")

    if not args.yes and input("  Proceed? [y/N] ").strip().lower() not in ("y", "yes"):
        return 1

    motion_link, laser_link = open_links(cfg)
    motion = MotionDriver(motion_link, cfg.motion)
    try:
        motion.sync(timeout_s=25.0)
        configure_firmware(motion_link, cfg)

        start_mm = motion.state.linear_mm
        chunk = cfg.motion.linear.max_chunk_mm
        rate = cfg.motion.linear.max_rate_mm_s
        ceiling = (1_000_000.0 / (2.0 * cfg.motion.step_delay_us)) * 4.0 / 1600.0

        print(f"\n  start position   {start_mm:.3f} mm")
        print(f"  configured rate  {rate:.2f} mm/s")
        print(f"  firmware ceiling {ceiling:.2f} mm/s  (step_delay {cfg.motion.step_delay_us} us)")
        print(f"  chunk size       {chunk:.2f} mm\n")

        direction = 1.0 if start_mm < (cfg.motion.linear.max_mm / 2) else -1.0
        commands = 0
        began = time.monotonic()

        while abs(motion.state.linear_mm - start_mm) < args.distance:
            if time.monotonic() - began > args.timeout:
                print("  (timeout reached)")
                break
            motion.move(linear_mm=direction * chunk)
            commands += 1

        duration = time.monotonic() - began
        travelled = abs(motion.state.linear_mm - start_mm)
        achieved = travelled / duration if duration else 0.0

        print(f"  travelled        {travelled:.3f} mm")
        print(f"  elapsed          {duration:.2f} s")
        print(f"  commands         {commands}  ({duration / max(commands, 1) * 1000:.0f} ms each)")
        print(f"\n  ACHIEVED         {achieved:.2f} mm/s")
        print(f"  duty cycle       {achieved / ceiling * 100:.0f}% of the firmware ceiling")

        if achieved < ceiling * 0.6:
            print("\n  Duty cycle is low. Per-command overhead dominates: raise")
            print("  motion.linear.max_chunk_mm so fewer round trips cover the")
            print("  same distance (costs e-stop latency).")
        if ceiling < 8.0:
            print(f"\n  The ceiling itself is low. Lower motion.step_delay_us")
            print(f"  ({cfg.motion.step_delay_us} -> {cfg.motion.step_delay_us // 2} doubles it) until the motor stalls,")
            print("  then back off. There is no acceleration ramp, so this is the")
            print("  rate it must start at from standstill.")

        print("\n  returning to start...")
        while abs(motion.state.linear_mm - start_mm) > 0.05:
            step = motion.state.linear_mm - start_mm
            motion.move(linear_mm=-max(-chunk, min(chunk, step)))
        print(f"  back at {motion.state.linear_mm:.3f} mm\n")

    finally:
        motion_link.close()
        if laser_link is not None and laser_link is not motion_link:
            laser_link.close()
    return 0


def cmd_diagnose_laser(args, cfg: SystemConfig) -> int:
    """Exercise the fire relays and report, WITHOUT ever arming or firing.

    Every step here is one the firmware forces into standby first: the bench
    test drives the ready contact open for its whole duration, so the machine
    cannot fire whatever these contacts do. Nothing sends e, r or f.

    This answers the hardware question -- is each fire channel actually
    switching, and does its switching reach anything -- which is the one the
    software has been guessing at.
    """
    print("\n" + "=" * 68)
    print("  LASER RELAY DIAGNOSTIC")
    print("=" * 68)
    print("  Does NOT arm and does NOT fire. The ready contact is forced open")
    print("  for the whole test, so the machine stays in standby.")
    print()
    print("  You need to be AT the rig for this: the result is what you hear")
    print("  and what a meter reads, not what the terminal prints.")
    print("=" * 68)

    if not args.yes and input("\n  Relay supply ON and you are watching? [y/N] ").strip().lower() not in ("y", "yes"):
        return 1

    motion_link, laser_link = open_links(cfg)
    laser = LaserDriver(laser_link, cfg.laser)
    try:
        print("\n1. Relock to a known safe state")
        state = laser.stop()
        print(f"   {state.describe()}")

        print("\n2. Applying configured fire polarity")
        laser.apply_fire_polarity()
        time.sleep(0.3)
        print(f"   ch1_invert={cfg.laser.fire_ch1_invert}  ch2_invert={cfg.laser.fire_ch2_invert}")

        for command, channel in (("t", 1), ("y", 2)):
            print(f"\n3.{channel} Bench-testing fire channel {channel} "
                  f"-- LISTEN for 5 clicks, METER the output terminals")
            laser_link.drain()
            laser_link.write_line(command)

            # The test is 5 cycles of 1 s on, 1 s off, driven inside the firmware.
            deadline = time.monotonic() + 16.0
            while time.monotonic() < deadline:
                for line in laser_link.drain():
                    if line.strip():
                        print(f"     {line}")
                time.sleep(0.2)

        print("\n4. Final state")
        print(f"   {laser.status().describe()}")

        print("\n" + "=" * 68)
        print("  HOW TO READ IT")
        print("=" * 68)
        print("  Both channels clicked        -> relays are fine. The problem is")
        print("                                  which Steute terminals they land")
        print("                                  on, or the channel timing. Try")
        print("                                  'g' (gap) and 'o' (lead order).")
        print("  One channel silent           -> dead board channel, missing")
        print("                                  jumper, or no coil power on that")
        print("                                  channel. Hardware. Fix that first.")
        print("  Clicks, Dornier ignores them -> wrong terminals on the Steute")
        print("                                  connector.")
        print()
        print("  Then the measurement that settles it: meter each channel's")
        print("  output pair with the coil released and again energised, and")
        print("  do the same on the pedal's own contacts pressed vs released.")
        print("  Matching those two contact maps tells us exactly what to drive.")
        print("=" * 68 + "\n")
    finally:
        try:
            laser.stop()
        except Exception:  # noqa: BLE001
            pass
        motion_link.close()
        if laser_link is not None and laser_link is not motion_link:
            laser_link.close()
    return 0


RE_CH1 = re.compile(r"ch1 \(pin 9\)\s*:\s*(ENABLED|disabled)")
RE_CH2 = re.compile(r"ch2 \(pin 10\)\s*:\s*(ENABLED|disabled)")
RE_LEAD = re.compile(r"leads\s*:\s*channel\s*(\d)")
RE_GAP = re.compile(r"gap\s*:\s*(\d+)\s*ms")

GAPS = [0, 25, 50, 100, 250, 500]


def _read_fire_config(link: Link) -> dict:
    """Parse the firmware's own fire-channel report."""
    link.drain()
    link.write_line("?")
    time.sleep(0.6)
    text = "\n".join(link.drain())

    cfg = {}
    if m := RE_CH1.search(text):
        cfg["ch1"] = m.group(1) == "ENABLED"
    if m := RE_CH2.search(text):
        cfg["ch2"] = m.group(1) == "ENABLED"
    if m := RE_LEAD.search(text):
        cfg["lead"] = int(m.group(1))
    if m := RE_GAP.search(text):
        cfg["gap"] = int(m.group(1))
    return cfg


def _apply_fire_config(link: Link, want: dict, have: dict) -> None:
    """Drive the firmware's toggles until its config matches `want`.

    The firmware exposes 1/2/o/g as toggles and a stepper, not absolute
    setters -- so the current state has to be read back and the difference
    walked, rather than assumed.
    """
    if want["ch1"] != have.get("ch1"):
        link.write_line("1")
        time.sleep(0.15)
    if want["ch2"] != have.get("ch2"):
        link.write_line("2")
        time.sleep(0.15)
    if want["lead"] != have.get("lead"):
        link.write_line("o")
        time.sleep(0.15)

    steps = 0
    current_gap = have.get("gap", 0)
    while current_gap != want["gap"] and steps < len(GAPS):
        link.write_line("g")
        time.sleep(0.15)
        current_gap = GAPS[(GAPS.index(current_gap) + 1) % len(GAPS)]
        steps += 1
    link.drain()


def cmd_sweep_fire(args, cfg: SystemConfig) -> int:
    """Walk the fire-channel configurations, firing a short pulse at each.

    This is the search the laser sketch's own notes describe but leaves to be
    done by hand: which channels, in which order, with what gap between them,
    does this machine actually accept as a fire.

    It DOES fire the laser, once per configuration, for a fraction of a second.
    There is no way to test a fire without firing.
    """
    print("\n" + "=" * 68)
    print("  FIRE CONFIGURATION SWEEP")
    print("=" * 68)
    print("  This ARMS AND FIRES the laser -- one short pulse per configuration,")
    print(f"  {args.pulse:.1f}s each, up to 14 configurations.")
    print()
    print("  Point it somewhere safe. Wear eye protection. Stay at the machine.")
    print("  Ctrl-C stops and relocks at any point.")
    print("=" * 68)

    if input("\n  Type FIRE to begin: ").strip() != "FIRE":
        print("  Cancelled.")
        return 1

    combos = [
        {"ch1": True, "ch2": False, "lead": 1, "gap": 0},
        {"ch1": False, "ch2": True, "lead": 2, "gap": 0},
    ]
    for lead in (1, 2):
        for gap in GAPS:
            combos.append({"ch1": True, "ch2": True, "lead": lead, "gap": gap})

    motion_link, laser_link = open_links(cfg)
    laser = LaserDriver(laser_link, cfg.laser)
    working = []

    try:
        for index, want in enumerate(combos, 1):
            have = _read_fire_config(laser_link)
            _apply_fire_config(laser_link, want, have)

            channels = ("ch1+ch2" if want["ch1"] and want["ch2"]
                        else "ch1 only" if want["ch1"] else "ch2 only")
            detail = f"lead {want['lead']}, gap {want['gap']} ms" if want["ch1"] and want["ch2"] else ""
            print(f"\n[{index}/{len(combos)}] {channels}  {detail}")

            try:
                laser.arm()
            except LaserError as exc:
                print(f"    could not arm: {exc}")
                continue

            laser.fire()
            time.sleep(args.pulse)
            laser.pause()
            laser.standby()

            answer = input("    Did it fire? [y/N/q] ").strip().lower()
            if answer == "q":
                break
            if answer in ("y", "yes"):
                working.append((channels, detail))
                print("    ^ recorded")
                if not args.all:
                    break
    except KeyboardInterrupt:
        print("\n  Interrupted.")
    finally:
        try:
            laser.stop()
            print("\n  Laser relocked.")
        except Exception as exc:  # noqa: BLE001
            print(f"\n  WARNING: could not relock: {exc} -- cut the relay supply")
        motion_link.close()
        if laser_link is not None and laser_link is not motion_link:
            laser_link.close()

    print("\n" + "=" * 68)
    if working:
        print("  CONFIGURATIONS THAT FIRED")
        for channels, detail in working:
            print(f"    {channels}  {detail}")
        print()
        print("  Tell me which one and I will make it the permanent default in")
        print("  the firmware, so it is not a runtime setting you have to redo.")
    else:
        print("  NOTHING FIRED in any configuration.")
        print()
        print("  That rules out channel selection, order and gap as the cause,")
        print("  which is worth knowing. It points at the contacts themselves:")
        print("    - run 'katzlab diagnose-laser' to hear whether both relays click")
        print("    - arm the machine and press the PHYSICAL pedal. If it fires on")
        print("      foot but not on relays, the relays are on the wrong Steute")
        print("      terminals, and metering them pressed vs released says which.")
    print("=" * 68 + "\n")
    return 0


def cmd_find_polarity(args, cfg: SystemConfig) -> int:
    """Find a fire-channel polarity that ARMS without firing.

    Never sends a fire command. It only relocks, sets a polarity, arms, and asks
    what the machine did -- which is exactly the question "why does Enable/Ready
    fire the laser" needs answered, and it can be answered without firing.
    """
    print("\n" + "=" * 68)
    print("  FIRE POLARITY FINDER")
    print("=" * 68)
    print("  Arms the laser in each of the four polarity combinations and asks")
    print("  what the Dornier did. NO fire command is ever sent.")
    print("  Between steps the laser is relocked.")
    print("=" * 68)

    if not args.yes and input("\n  Ready? [y/N] ").strip().lower() not in ("y", "yes"):
        return 1

    combos = [
        (False, False, "both normal   (open at rest)"),
        (True, False, "ch1 inverted  (ch1 closed at rest)"),
        (False, True, "ch2 inverted  (ch2 closed at rest)"),
        (True, True, "both inverted (both closed at rest)"),
    ]

    motion_link, laser_link = open_links(cfg)
    laser = LaserDriver(laser_link, cfg.laser)
    results = []

    try:
        for index, (ch1, ch2, label) in enumerate(combos, 1):
            laser.stop()
            time.sleep(0.4)
            laser_link.write_line(f"P{(1 if ch1 else 0) | (2 if ch2 else 0)}")
            time.sleep(0.5)
            laser_link.drain()

            print(f"\n[{index}/4] {label}")
            try:
                state = laser.arm()
                print(f"    armed: {state.describe()}")
            except LaserError as exc:
                print(f"    could not arm: {exc}")
                results.append((label, "would not arm"))
                continue

            print("    Look at the Dornier.")
            answer = input("      [r] READY, clean   [f] it FIRED   [c] complains   [n] nothing : ").strip().lower()
            results.append((label, {"r": "READY, clean", "f": "FIRED on arm",
                                    "c": "complains", "n": "no response"}.get(answer, answer)))
            laser.stop()
    except KeyboardInterrupt:
        print("\n  Interrupted.")
    finally:
        try:
            laser.stop()
            print("\n  Laser relocked.")
        except Exception as exc:  # noqa: BLE001
            print(f"\n  WARNING: could not relock: {exc}")
        motion_link.close()
        if laser_link is not None and laser_link is not motion_link:
            laser_link.close()

    print("\n" + "=" * 68)
    for label, outcome in results:
        print(f"  {label:<32} {outcome}")
    print("=" * 68)

    clean = [l for l, o in results if o == "READY, clean"]
    if clean:
        print("\n  Use this one -- it arms without firing:")
        for l in clean:
            print(f"    {l}")
        print("\n  Set the matching fire_ch1_invert / fire_ch2_invert in")
        print("  config/system.yaml and it will be applied at every startup.")
    else:
        print("\n  No polarity arms cleanly. Every combination either fires on")
        print("  arm or is rejected -- which means the relay contacts are not on")
        print("  the terminals the machine watches for a pedal press. That is a")
        print("  wiring fact, and no polarity setting can compensate for it.")
    print()
    return 0


def start_recorder(cfg: SystemConfig, enabled: bool) -> SessionRecorder | None:
    """Open a CSV recorder for this session, if recording is on."""
    if not enabled:
        return None
    directory = Path(__file__).resolve().parent.parent / cfg.logging.session_log_dir
    recorder = SessionRecorder(directory, rate_hz=cfg.logging.record_hz)
    path = recorder.open()
    print(f"  Recording motion to {path}")
    return recorder


def make_status_line(cfg: SystemConfig, has_laser: bool):
    """A single line, redrawn in place, showing where the scope actually is.

    The CSV answers "what happened"; this answers "what is happening". Printed
    on \r so it stays on one line, and rate-limited so it never competes with
    the log for the serial budget or the terminal.
    """
    state = {"last": 0.0}

    def render(status: dict) -> None:
        now = time.monotonic()
        if now - state["last"] < 0.15:
            return
        state["last"] = now

        motion = status["motion"]
        laser = status["laser"]

        parts = [
            f"lin {getattr(motion, 'linear_mm', 0.0):8.3f} mm",
            f"rot {getattr(motion, 'rotation_map_deg', 0.0):8.2f}\u00b0",
            f"flex {getattr(motion, 'flexion_tip_deg', 0.0):7.2f}\u00b0",
            f"v {getattr(motion, 'linear_velocity_mm_s', 0.0):+6.2f} mm/s",
        ]
        if has_laser and laser is not None:
            if laser.firing:
                parts.append("LASER FIRING")
            elif laser.armed:
                parts.append("armed")
            else:
                parts.append("safe ")
        if status["estopped"]:
            parts.append("E-STOP")

        sys.stdout.write("\r\033[K  " + " | ".join(parts))
        sys.stdout.flush()

    return render


def print_banner(
    cfg: SystemConfig, controller_name: str, has_laser: bool, *, dry_run: bool = False
) -> None:
    print("\n" + "=" * 68)
    print("  KATZLAB URETEROSCOPE CONTROL" + ("   [DRY RUN - NOTHING MOVES]" if dry_run else ""))
    print("=" * 68)
    print(f"  Controller : {controller_name}  (source: {cfg.input.source})")
    print(f"  Link       : {cfg.link.mode}")
    print(f"  Laser      : {'enabled' if has_laser else 'DISABLED for this session'}")
    print("-" * 68)
    print("  Left stick        linear travel")
    print("  Right stick X     rotation")
    print("  Right stick Y     flexion")
    print("  Left secondary    Enable + Ready       Left primary   Standby")
    print("  Right secondary   Fire                 Right primary  Pause")
    print("  X                 EMERGENCY STOP       H              Home")
    print("-" * 68)
    print("  After an e-stop, press Standby to acknowledge and re-enable control.")
    print("  Ctrl-C stops the loop and relocks the laser.")
    print("=" * 68 + "\n")


# --------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="katzlab", description="Python control for the KatzLab ureteroscope rig"
    )
    parser.add_argument("--config", help="path to system.yaml")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("ports", help="list serial ports")
    sub.add_parser("selftest", help="check config, link and controller; moves nothing")

    monitor = sub.add_parser("monitor", help="passive serial monitor")
    monitor.add_argument("--port", help="override the configured port")

    bring = sub.add_parser(
        "bringup", help="flash, verify and start controlling -- the one command"
    )
    bring.add_argument("--yes", action="store_true", help="assume yes to the safety prompts")
    bring.add_argument("--skip-flash", action="store_true", help="firmware is already flashed")
    bring.add_argument("--no-laser", action="store_true", help="motion only")
    bring.add_argument(
        "--no-record", action="store_true", help="do not write a motion CSV"
    )
    bring.add_argument(
        "--no-status", action="store_true", help="hide the live position line"
    )
    bring.add_argument(
        "--ready-open",
        type=float,
        metavar="SECONDS",
        help="override laser.ready_edge_open_s for this run (tune arming)",
    )
    bring.add_argument(
        "--debug-buttons",
        action="store_true",
        help="log every button edge and which action it dispatched",
    )

    bench = sub.add_parser("bench", help="measure real linear mm/s (moves the carriage)")
    bench.add_argument("--distance", type=float, default=10.0, help="mm to travel (default 10)")
    bench.add_argument("--timeout", type=float, default=60.0, help="give up after N seconds")
    bench.add_argument("--yes", action="store_true", help="skip the confirmation")

    diag = sub.add_parser(
        "diagnose-laser",
        help="bench-test the fire relays; never arms or fires",
    )
    diag.add_argument("--yes", action="store_true", help="skip the confirmation")

    sweep = sub.add_parser(
        "sweep-fire", help="walk fire-channel configurations; ARMS AND FIRES the laser"
    )
    sweep.add_argument("--pulse", type=float, default=0.3, help="fire pulse length (s)")
    sweep.add_argument("--all", action="store_true", help="keep going after the first hit")

    findpol = sub.add_parser(
        "find-polarity", help="find a polarity that arms without firing; never fires"
    )
    findpol.add_argument("--yes", action="store_true", help="skip the confirmation")

    run = sub.add_parser("run", help="run the joystick control loop")
    run.add_argument(
        "--no-record", action="store_true", help="do not write a motion CSV"
    )
    run.add_argument(
        "--no-status", action="store_true", help="hide the live position line"
    )
    run.add_argument(
        "--debug-buttons",
        action="store_true",
        help="log every button edge and which action it dispatched",
    )
    run.add_argument(
        "--dry-run",
        action="store_true",
        help="verify the control mapping with no serial link; nothing moves",
    )
    run.add_argument(
        "--no-laser",
        action="store_true",
        help="motion only; the laser link is left untouched",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg = load_system_config(args.config)
    except ConfigError as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2

    if getattr(args, "ready_open", None) is not None:
        import dataclasses

        cfg = dataclasses.replace(
            cfg,
            laser=dataclasses.replace(cfg.laser, ready_edge_open_s=args.ready_open),
        )
        print(f"ready_edge_open_s overridden to {args.ready_open:.2f}s for this run")

    setup_logging(cfg.logging.level, cfg.logging.session_log_dir)

    handlers = {
        "ports": cmd_ports,
        "monitor": cmd_monitor,
        "selftest": cmd_selftest,
        "run": cmd_run,
        "bringup": cmd_bringup,
        "bench": cmd_bench,
        "diagnose-laser": cmd_diagnose_laser,
        "sweep-fire": cmd_sweep_fire,
        "find-polarity": cmd_find_polarity,
    }

    try:
        return handlers[args.command](args, cfg)
    except (LinkError, ConfigError, InputUnavailable) as exc:
        log.error("%s", exc)
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
