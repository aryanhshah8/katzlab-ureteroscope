#!/usr/bin/env python3
"""Generate Combined_Motion_Laser.ino from the two original sketches.

Both sketches are needed at once on a single Teensy, but each defines its own
``setup()`` and ``loop()``. Those two names are the *only* symbols that collide
-- every global, constant and helper in the two files is distinct, and the
command alphabets do not overlap either ('h'/'home' is motion, the laser's
letters are e r f p s x 1 2 3 4 o g t y w ?).

So the merge is mechanical, and doing it as a build step rather than by hand
means the originals stay the source of truth. Edit the .ino files, re-run this,
reflash.

    python scripts/build_merged_firmware.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
MOTION_SRC = ROOT / "3DOF_MovementF" / "3DOF_MovementF.ino"
LASER_SRC = ROOT / "shootingyesready_inoF-1" / "shootingyesready_inoF-1.ino"
OUT_DIR = ROOT / "firmware" / "Combined_Motion_Laser"
OUT_FILE = OUT_DIR / "Combined_Motion_Laser.ino"


def strip_function(source: str, signature: str) -> str:
    """Remove one brace-balanced top-level function definition."""
    start = source.find(signature)
    if start == -1:
        raise SystemExit(f"could not find {signature!r} to remove")

    brace = source.find("{", start)
    if brace == -1:
        raise SystemExit(f"no opening brace after {signature!r}")

    depth = 0
    for i in range(brace, len(source)):
        if source[i] == "{":
            depth += 1
        elif source[i] == "}":
            depth -= 1
            if depth == 0:
                return source[:start] + source[i + 1 :]
    raise SystemExit(f"unbalanced braces in {signature!r}")


def require(source: str, needle: str, label: str) -> None:
    if needle not in source:
        raise SystemExit(
            f"expected text not found in {label}: {needle!r}\n"
            "The source sketch changed shape; update this build script."
        )


def transform_motion(source: str) -> str:
    require(source, "void setup() {", "3DOF sketch")
    require(source, "bool readMovementCommand() {", "3DOF sketch")

    source = source.replace("void setup() {", "void motionSetup() {", 1)
    source = strip_function(source, "void loop() {")

    # readMovementCommand() reads the serial line itself. The merged dispatcher
    # has to read the line first to decide who it belongs to, so the reading
    # half is removed and the parsing half takes the line as an argument.
    source = source.replace(
        "bool readMovementCommand();",
        "bool parseMovementLine(const char inputLine[]);",
        1,
    )

    old_head = (
        "bool readMovementCommand() {\n"
        "  // Read one complete line from the Serial Monitor\n"
        "  // The character array keeps the input small and avoids dynamic memory use\n"
        "  char inputLine[64];\n"
        "\n"
        "  // Reset this flag before reading each new command\n"
        "  homeCommandRequested = false;\n"
        "\n"
        "  // readBytesUntil stops at newline or when Serial timeout is reached\n"
        "  size_t inputLength = Serial.readBytesUntil('\\n', inputLine, sizeof(inputLine) - 1);\n"
        "\n"
        "  // Add a null terminator so sscanf knows where the text ends\n"
        "  inputLine[inputLength] = '\\0';\n"
    )
    new_head = (
        "bool parseMovementLine(const char inputLine[]) {\n"
        "  // The dispatcher has already read the line and decided it is not a\n"
        "  // laser command, so this only has to parse.\n"
        "  homeCommandRequested = false;\n"
    )
    if old_head not in source:
        raise SystemExit(
            "readMovementCommand() no longer matches the expected text.\n"
            "Update transform_motion() in this build script."
        )
    source = source.replace(old_head, new_head, 1)

    # ---- serial budget --------------------------------------------------
    # At 20 commands/s the verbose prints are ~900 bytes per command, which is
    # ~80 ms of pure transmission at 115200 -- more than the motion itself.
    # Terse mode drops everything except the lines the host driver parses.
    source = source.replace(
        "void printCommandPrompt() {",
        "void printCommandPrompt() {\n"
        "  // Terse mode keeps only the line the host driver synchronises on.\n"
        "  if (terseOutput) {\n"
        "    Serial.println(\"\\nPOSITIONING: Linear = RELATIVE | Rotation = RELATIVE | Flexion = ABSOLUTE\");\n"
        "    return;\n"
        "  }",
        1,
    )
    source = source.replace(
        "void printMovementStatusIfReady() {",
        "void printMovementStatusIfReady() {\n"
        "  if (terseOutput) {\n"
        "    return;\n"
        "  }",
        1,
    )
    source = source.replace(
        "void printRequestedTargets() {",
        "void printRequestedTargets() {\n"
        "  // ~500 bytes of pre-move report. The host already knows what it asked for.\n"
        "  if (terseOutput) {\n"
        "    return;\n"
        "  }",
        1,
    )
    source = source.replace(
        "void printFinalSummary() {",
        "void printFinalSummary() {\n"
        "  // Terse: only the three lines the host parses position out of.\n"
        "  if (terseOutput) {\n"
        "    Serial.println(\"3-DOF Movement Complete\");\n"
        "    Serial.print(\"Final Linear Absolute Position: \");\n"
        "    Serial.print(currentLinearPositionMM, 3);\n"
        "    Serial.println(\" mm\");\n"
        "    Serial.print(\"Final Rotation Map Position: \");\n"
        "    Serial.print(currentRotationMapDeg, 2);\n"
        "    Serial.println(\" deg\");\n"
        "    Serial.print(\"Final Flexion Step, Motor Deg, Tip Deg: \");\n"
        "    Serial.print(currentFlexionStep);\n"
        "    Serial.print(\", \");\n"
        "    Serial.print(measuredFlexionMotorDeg, 2);\n"
        "    Serial.print(\", \");\n"
        "    Serial.println(measuredTipDeg, 2);\n"
        "    return;\n"
        "  }",
        1,
    )
    source = source.replace(
        '  Serial.println("\\nStarting 3-DOF Movement");',
        '  if (!terseOutput) {\n    Serial.println("\\nStarting 3-DOF Movement");\n  }',
        1,
    )

    # ---- stop waiting on the servo for every jog ------------------------
    # runSimultaneousMovementLoop() runs while ANY axis is unfinished. Flexion
    # was never "finished" when the servo was connected, so a pure linear jog
    # still waited 40 ms (feedback poll) to 500 ms (no-change timeout) on a
    # servo that was not being asked to move at all.
    require(source, "  flexionDone = !servoConnected;", "3DOF sketch")
    source = source.replace(
        "  flexionDone = !servoConnected;",
        "  // Nothing to wait for if the servo is absent, or if this command asks\n"
        "  // for the flexion angle the servo was already sent to.\n"
        "  flexionDone = !servoConnected || (targetFlexionSteps == lastCommandedFlexionSteps);",
        1,
    )

    require(source, "void startFlexionMovement() {", "3DOF sketch")
    source = source.replace(
        "void startFlexionMovement() {",
        "void startFlexionMovement() {\n"
        "  // Already satisfied -- do not re-issue the same position command.\n"
        "  if (flexionDone) {\n"
        "    return;\n"
        "  }",
        1,
    )
    source = source.replace(
        "  // Start the no-change timer after the command is accepted\n"
        "  lastFlexionChangeTime = millis();",
        "  // Start the no-change timer after the command is accepted\n"
        "  lastFlexionChangeTime = millis();\n"
        "  lastCommandedFlexionSteps = targetFlexionSteps;\n"
        "\n"
        "  // Host mode is fire-and-forget. Re-commanding an absolute target 20x a\n"
        "  // second restarts the servo's acceleration ramp before it ever reaches\n"
        "  // speed, which is what makes flexion feel like it is clicking rather\n"
        "  // than moving. Issue the command and let the servo get on with it; the\n"
        "  // host sends a fresh target on the next tick anyway.\n"
        "  if (terseOutput) {\n"
        "    flexionDone = true;\n"
        "  }",
        1,
    )

    # A 1-step margin is ~0.088 motor degrees. A geared servo under load will
    # not land that precisely, so it burned the 500 ms no-change timeout instead.
    source = source.replace(
        "const int flexionStopMarginSteps = 1;",
        "const int flexionStopMarginSteps = 8;   // ~0.7 motor deg; 1 was unreachable under load",
        1,
    )

    # ---- position tracking must follow the COMMANDED direction ----------
    # readLinearFeedback() stops on fabsf(measured), so the encoder's sign is
    # irrelevant to stopping. But updateAbsoluteLinearPosition() then added the
    # SIGNED measurement to the tracked position. If the encoder's sign
    # disagrees with the commanded direction -- easy to get wrong, and
    # linearEncoderSign/linear_dirForward have both been flipped by hand in this
    # sketch -- the tracked position walks the wrong way. The host clamps
    # against that position, so one direction gets throttled to nothing while
    # the other runs unclamped. Take the magnitude and apply the direction we
    # actually commanded, which is known exactly.
    require(source, "  currentLinearPositionMM = currentLinearPositionMM + measuredLinearUreteroMM;", "3DOF sketch")
    source = source.replace(
        "  currentLinearPositionMM = currentLinearPositionMM + measuredLinearUreteroMM;",
        "  currentLinearPositionMM =\n"
        "    currentLinearPositionMM + (linearMovementSign * fabsf(measuredLinearUreteroMM));",
        1,
    )

    require(
        source,
        "  currentRotationMapDeg = normalizeRotationMapDeg(currentRotationMapDeg + measuredUreteroRotationDeg);",
        "3DOF sketch",
    )
    source = source.replace(
        "  currentRotationMapDeg = normalizeRotationMapDeg(currentRotationMapDeg + measuredUreteroRotationDeg);",
        "  currentRotationMapDeg = normalizeRotationMapDeg(\n"
        "    currentRotationMapDeg + (rotationMovementSign * fabsf(measuredUreteroRotationDeg)));",
        1,
    )

    # A dead or unplugged encoder reads zero forever, so the fabsf() stop
    # condition never trips and every move runs to the backup limit -- 300
    # pulses (0.75 mm) past target, every single command. Tighten that to
    # something that cannot be mistaken for normal motion.
    source = source.replace(
        "  long linearBackupPulseLimit = targetLinearPulses + 300;",
        "  long linearBackupPulseLimit = targetLinearPulses + 8;   // was 300 = 0.75 mm of overshoot per command",
        1,
    )
    source = source.replace(
        "  long rotationBackupPulseLimit = targetRotationPulses + 300;",
        "  long rotationBackupPulseLimit = targetRotationPulses + 8;   // was 300",
        1,
    )

    # ---- let the host raise the step rate --------------------------------
    # 800 us high + 800 us low = 625 pulses/s = 1.5625 mm/s, and that is the
    # ceiling regardless of duty cycle. There is no acceleration ramp, so this
    # has to be raised by feel -- too fast from a standstill and the motor stalls.
    source = source.replace(
        "const unsigned int stepDelay_us = 800;",
        "unsigned int stepDelay_us = 800;   // runtime-settable with [v<us>]",
        1,
    )

    # Drop the file-level block comment; the merged file gets its own.
    source = re.sub(r"\A///[^\n]*\n/\*.*?\*/\n", "", source, count=1, flags=re.DOTALL)
    return source


def transform_laser(source: str) -> str:
    require(source, "void setup() {", "laser sketch")
    source = source.replace("void setup() {", "void laserSetup() {", 1)
    source = strip_function(source, "void loop() {")
    source = re.sub(r"\A///[^\n]*\n/\*.*?\*/\n", "", source, count=1, flags=re.DOTALL)
    return source


HEADER = '''/// Combined 3-DOF Motion + Laser  -  GENERATED FILE, DO NOT EDIT BY HAND
/*
  Generated by python/scripts/build_merged_firmware.py from:
    3DOF_MovementF/3DOF_MovementF.ino
    shootingyesready_inoF-1/shootingyesready_inoF-1.ino

  Edit those two files and re-run the build script. Editing this file directly
  means the next build silently discards your change.

  WHY THIS EXISTS
    Both sketches are needed at once on a single Teensy 4.1. Their pins do not
    collide (motion uses 0-7, 16, 17, 20, 21; the laser relays use 8, 9, 10) and
    the only clashing symbols are setup() and loop(). This file renames those
    two per sketch and adds a dispatcher; everything else is carried over
    unchanged.

  COMMAND DISPATCH
    Three numbers        linear_mm_relative rotation_deg_relative flexion_deg_absolute
    home | h             return all three axes to the home state
    e r f p s x          laser: enable, ready, fire, pause, standby, stop
    1 2 3 4 o g t y w ?  laser: channel, polarity, order, gap, bench test, status
    q                    toggle terse output / host mode
    v<us>                set the STEP pulse half-period, 40-5000 us

    The two alphabets do not overlap, so routing is unambiguous.

  TERSE MODE IS HOST MODE
    Send [q] once after connecting. It does two things, both of which matter
    only when a program is driving:

      1. Drops the verbose prints. At ~20 commands/s the pre-move report and
         full summary are ~900 bytes per command -- about 80 ms of transmission
         at 115200, more than the motion itself takes.
      2. Makes flexion fire-and-forget. Re-commanding an absolute servo target
         20x a second restarts its acceleration ramp before it reaches speed,
         which is what makes flexion click rather than move.

    The line the host synchronises on is still printed. Terse resets on boot.

  WHY A JOG USED TO BARELY MOVE
    runSimultaneousMovementLoop() runs while ANY axis is unfinished, and flexion
    counted as unfinished whenever the servo was connected -- even when the
    command was not asking it to move. So every linear-only jog waited on the
    servo: 40 ms for a feedback poll, or the full 500 ms no-change timeout if
    the servo sat a couple of steps off target. Combined with the print cost,
    the linear axis was stepping about a quarter of the time. Both are fixed
    here: an unchanged flexion target now completes immediately.

  !!! LIVE FAIL-SAFE HAZARD - READ BEFORE POWERING OR FLASHING !!!
    READY_ON_NC is true, so pin 8 is on the relay's NC terminal. That makes the
    ready contact CLOSED - the laser ARMED - whenever the coil is de-energised:

      pin 8 LOW  / coil released   -> contact CLOSED -> ARMED
      pin 8 HIGH / coil energised  -> contact OPEN   -> standby

    Standby therefore depends on the coil staying energised. A Teensy reset, a
    USB replug, a firmware upload, a loose jumper, or loss of the relay supply
    all release the coil and ARM THE LASER with nobody commanding it. Flashing
    this file does exactly that for the duration of the upload.

    Before flashing, de-energise the relay supply. Do not leave the rig
    energised unattended.

    The real fix is hardware: move the pin 8 wire from the relay's NC terminal
    to its NO terminal, then set READY_ON_NC = false in the source sketch and
    regenerate. That makes de-energised mean standby, and power loss fail safe.

    (Comments in the source sketch claim pin 8 was moved to NO. They are stale.
    The value of READY_ON_NC is what this build follows.)

  WHAT THE SOFTWARE ALREADY DOES ABOUT IT
    setupLaserOutputs() writes the safe level BEFORE pinMode(), so pin 8 drives
    standby the instant it becomes an output rather than glitching through
    armed first. setup() calls laserSetup() before motionSetup() so that
    happens as early as possible. This closes the reset-to-setup window; it
    cannot close the window while the Teensy is held in reset or being
    programmed, because the pin is not an output then. That window is hardware.

  A LIMITATION WORTH KNOWING
    A motion command blocks until the move finishes, so a laser command sent
    during a move waits for it. Keep per-command moves short (the host caps them
    in system.yaml) to bound that wait.
*/

// Terse mode is really "host mode": it suppresses the chatty prints AND stops
// the movement loop waiting for the servo to arrive. Both are right when a
// program is issuing ~20 commands a second and wrong when a human is typing.
// Toggled at runtime with [q]; off at boot so the Serial Monitor stays usable.
bool terseOutput = false;

// Last flexion target actually sent to the ST3020. A jog that only moves the
// linear or rotation axis re-sends the same flexion target every command; without
// this the movement loop would wait for the servo on every one of them.
int lastCommandedFlexionSteps = 2048;   // flexionNeutralStep, commanded in setup()

'''

DISPATCHER = '''

// ===========================================================================
// Merged Command Dispatcher
// ===========================================================================

// Single letters owned by the laser sketch. 'h' is deliberately absent: it is
// the motion sketch's home shortcut, and the two alphabets must stay disjoint.
static bool isLaserCommand(char command) {
  switch (command) {
    case 'e': case 'r': case 'f': case 'p': case 's': case 'x':
    case '1': case '2': case '3': case '4':
    case 'o': case 'g': case 't': case 'y': case 'w': case '?':
      return true;
    default:
      return false;
  }
}

static bool readDispatchLine(char inputLine[], size_t size) {
  size_t length = Serial.readBytesUntil('\\n', inputLine, size - 1);

  if (length > 0 && inputLine[length - 1] == '\\r') {
    length--;
  }
  inputLine[length] = '\\0';

  for (size_t i = 0; i < length; i++) {
    if (inputLine[i] != ' ' && inputLine[i] != '\\t') {
      return true;
    }
  }
  return false;  // blank line
}

// The original motion loop printed its prompt once because it then blocked on
// "while (Serial.available() == 0) {}". The merged loop must not block -- a
// laser command has to be servicable at any time -- so the prompt is instead
// gated on a flag and printed exactly once per handled command.
static bool promptPending = true;

void setup() {
  // Laser first and deliberately: setupLaserOutputs() parks the relays in their
  // safe state, and that must happen before anything else can take time.
  laserSetup();
  motionSetup();

  Serial.println();
  Serial.println("=== COMBINED MOTION + LASER BUILD ===");
  Serial.println("Motion: three numbers, or home/h");
  Serial.println("Laser : e r f p s x 1 2 3 4 o g t y w ?");
  Serial.println("Send q for terse output when driving from a host program.");
  promptPending = true;
}

void loop() {
  if (promptPending) {
    printCommandPrompt();
    promptPending = false;
  }

  if (Serial.available() <= 0) {
    return;
  }

  char inputLine[64];
  if (!readDispatchLine(inputLine, sizeof(inputLine))) {
    return;  // blank line, nothing consumed, no new prompt owed
  }

  char single = getSingleLetterCommand(inputLine);

  if (single == 'q') {
    terseOutput = !terseOutput;
    Serial.print("Terse output: ");
    Serial.println(terseOutput ? "ON" : "OFF");
    promptPending = true;
    return;
  }

  // A bare digit is ambiguous: it is a laser channel/polarity command, and it
  // is also not a valid motion command (motion always needs three numbers), so
  // it is routed to the laser. Hosts send all three numbers and never hit this.
  // v<microseconds>: set the STEP pulse half-period at runtime.
  // The sketch has no acceleration ramp, so this is the rate the motor must
  // start at from a standstill -- lower it too far and the stepper stalls
  // instead of turning. Tune by feel: halve it, test, repeat until it slips,
  // then back off.
  if (inputLine[0] == 'v' || inputLine[0] == 'V') {
    int requested = atoi(inputLine + 1);

    if (requested >= 40 && requested <= 5000) {
      stepDelay_us = (unsigned int)requested;
      Serial.print("Step delay: ");
      Serial.print(stepDelay_us);
      Serial.print(" us  ->  ");
      Serial.print(1000000.0f / (2.0f * stepDelay_us), 0);
      Serial.print(" pulses/s  ->  ");
      Serial.print((1000000.0f / (2.0f * stepDelay_us)) * linearMMPerRev / linearPulsesPerRev, 2);
      Serial.println(" mm/s max");
    } else {
      Serial.println("Step delay must be 40 to 5000 us");
    }

    promptPending = true;
    return;
  }

  if (single != '\\0' && isLaserCommand(single)) {
    if (!terseOutput) {
      printReceivedInput(inputLine);
    }
    handleLaserCommand(single);
    printLaserStatus();
    promptPending = true;
    return;
  }

  // Anything else belongs to the motion sketch.
  if (!parseMovementLine(inputLine)) {
    Serial.println("Wrong Format");
    promptPending = true;
    return;
  }

  if (!calculateAllTargets()) {
    promptPending = true;
    return;
  }

  moveAllAxesTogether();
  promptPending = true;
}
'''


def main() -> int:
    for path in (MOTION_SRC, LASER_SRC):
        if not path.exists():
            raise SystemExit(f"source sketch not found: {path}")

    motion = transform_motion(MOTION_SRC.read_text())
    laser = transform_laser(LASER_SRC.read_text())

    merged = (
        HEADER
        + "\n// " + "=" * 73 + "\n"
        + "// Motion sketch, from 3DOF_MovementF.ino\n"
        + "// " + "=" * 73 + "\n\n"
        + motion
        + "\n// " + "=" * 73 + "\n"
        + "// Laser sketch, from shootingyesready_inoF-1.ino\n"
        + "// " + "=" * 73 + "\n\n"
        + laser
        + DISPATCHER
    )

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    OUT_FILE.write_text(merged)

    # A merged file that still contains a bare setup()/loop() from a source
    # sketch would not compile; catch that here rather than in the IDE.
    for forbidden in ("\nvoid setup() {\n  setupLaserOutputs", "\nvoid loop() {\n  if (Serial.available() <= 0)"):
        if forbidden in merged:
            raise SystemExit("merge failed: a source setup()/loop() survived")

    print(f"wrote {OUT_FILE}")
    print(f"  {len(merged.splitlines())} lines")
    print(f"  motion: {len(motion.splitlines())} lines from {MOTION_SRC.name}")
    print(f"  laser : {len(laser.splitlines())} lines from {LASER_SRC.name}")
    print("\nFlash with Arduino IDE or Teensy Loader, then send [q] for terse mode.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
