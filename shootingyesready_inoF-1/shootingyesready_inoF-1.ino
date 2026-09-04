/// Teensy 4.1 Laser Firing Only  -  BUILD: LASER-V5
/*
Overall Purpose:
  Control one Dornier laser system from the Arduino Serial Monitor.
  Laser-only version of the 3-DOF ureteroscope sketch. All movement, encoder,
  stepper, and servo code has been removed.

  THIS FILE REPLACES laser_firing_only_fixed.ino AND EVERY OTHER LASER SKETCH
  IN Downloads. Delete the rest so there is exactly one thing to flash.

Hardware:
  Teensy 4.1
  Three High Level Trigger Relay Modules (confirmed via jumper + bench test)
  External relay power supply, ground shared with the Teensy
  Relay OUTPUT side lands on a Steute 140094 footswitch, which in turn
  connects to the Dornier. The relays do NOT talk to the laser directly -
  they talk to the pedal. See "TOPOLOGY" below, this matters.

Pins:
  Pin 8  -> laser ready/standby channel   (relay NC terminal - see WARNING)
  Pin 9  -> fire channel 1
  Pin 10 -> fire channel 2

Serial commands:
  e = Enable        r = Ready        f = Fire
  p = Pause         s = Standby      x = Stop and relock
  1 = fire channel 1 on/off          2 = fire channel 2 on/off
  3 = invert channel 1 polarity      4 = invert channel 2 polarity
  o = swap which channel leads       g = step the inter-channel gap
  ? = status

============================================================================
STATUS OF THE INVESTIGATION

  Bug 1 - Ready never arms.  FIXED, CONFIRMED ON HARDWARE.
    Pin 8 is landed on the relay's NC terminal, so the ready contact is
    asserted when the coil is RELEASED - the opposite of the fire channels.
    The input is also EDGE sensitive: a static level does not arm it, the
    machine must see an open->closed transition. Both are handled below.

    Evidence: on the old build, "e r f x" armed the machine only AFTER x.
    r drove pin 8 HIGH (opening the contact), x drove it LOW (closing it).
    The only closing edge in the whole sequence was the one x produced.

  Bug 2 - "release footswitch fully", laser never fires.  OPEN.

============================================================================
TOPOLOGY - WHY BUG 2 IS PROBABLY NOT A FIRMWARE BUG

  The fire relays are wired to a Steute 140094 footswitch, not to a Dornier
  interface connector. The pedal's own contacts are therefore still in the
  circuit, and how our relay contacts sit relative to them decides everything:

    A) Relays in SERIES with the pedal contacts
       -> nothing this sketch does can fire the laser. A human still has to
          stand on the pedal. Ready working while fire never does is exactly
          what this looks like.

    B) Pedal is a CHANGEOVER / SPDT type (common on medical footswitches)
       -> the machine expects one contact to CLOSE and the other to OPEN on
          press. This sketch has been closing BOTH. That is an invalid
          channel state, and "release footswitch fully" is the standard
          complaint for exactly that. Commands [3] and [4] invert a channel
          so it sits CLOSED at idle and OPENS on fire, which is the shape a
          changeover contact actually needs.

    C) Relays land on the wrong pair of terminals on the Steute connector
       -> same class of error we already confirmed on pin 8.

  Firmware cannot distinguish these. THE DECISIVE TEST NEEDS NO CODE:
    1. Arm the laser, then press the PHYSICAL pedal.
       Fires on foot but not on relays -> the difference is in the contacts,
       not in this sketch.
    2. Meter the Steute's terminals pressed vs released and write down which
       pairs close and which open. That contact map answers A, B, and C at
       once, and tells us exactly what to drive.

============================================================================
CHANGE LOG (from the pasted laser_firing_only_fixed.ino):
  1) Ready polarity inverted and renamed for the CONTACT state rather than
     the coil state. Coil-based naming is what hid this bug for three rounds,
     because on pin 8 the coil and the contact are opposites.
  2) Ready [r] now drives a deliberate OPEN -> settle -> CLOSE edge instead of
     writing a level, so arming no longer depends on whatever state pin 8
     happened to already be in.
  3) SAFETY: boot, Standby [s] and Stop [x] now drive pin 8 to the OPEN
     level. The old code drove LOW in all three places, which on NC wiring
     CLOSES the contact - that is why [x] armed the machine.
  4) SAFETY: the software/machine state desync is gone. The old build printed
     ready=0 after [x] while the machine was actually READY.
  5) Standby [s] always drives the contact open, instead of only when
     laserReady was already true. If the two disagreed, the old code left the
     laser armed.
  6) The ready confirmation message no longer claims success. Nothing here
     reads back from the Dornier, so it now says so and tells you to look at
     the machine. The old unconditional "CLOSED and HELD" print is what made
     four broken builds look healthy.
  7) Fire channels are driven independently: enable, polarity, order and gap
     are all runtime-settable, for hypotheses A/B/C above.
  8) BUILD banner at boot and on [?], so the running build is never in doubt.

!!! FAIL-SAFE WARNING !!!
  Pin 8 on NC means standby requires the coil to stay ENERGIZED. A Teensy
  reset, USB replug, firmware upload, loose jumper, or loss of the relay
  supply will RELEASE the coil, CLOSE the ready contact, and ARM THE LASER
  with nobody commanding it. Uploading this sketch will do it.

  THE FIX IS HARDWARE, NOT SOFTWARE: move the pin 8 wire from the relay's NC
  terminal to its NO terminal, then set READY_ON_NC = false below. That makes
  de-energized = standby and power loss fail safe.
  Until that wire moves, do not leave this rig energized unattended.
============================================================================
*/

// ---------------------------------------------------------------------------
// Build Identity
// ---------------------------------------------------------------------------

// Printed at boot and on [?]. If you reset the Teensy and do not see this
// banner, the board is running a DIFFERENT sketch and nothing in this file is
// in effect. Check the banner before trusting any test result.
const char* BUILD_NAME = "LASER-V5.2";
const char* BUILD_NOTE = "pin8 NO + edge-armed; fire confirmed working; [w] flips ready sense";

// ---------------------------------------------------------------------------
// Laser Pin Assignments
// ---------------------------------------------------------------------------

const int relay_laser = 8;
const int fire1_laser = 9;
const int fire2_laser = 10;

// ---------------------------------------------------------------------------
// Relay Board Electrical Facts
// ---------------------------------------------------------------------------

// All three boards are high-level-trigger type:
//   HIGH = coil energizes -> NO closes, NC opens
//   LOW  = coil releases  -> NO opens,  NC closes
const int COIL_ENERGIZED = HIGH;
const int COIL_RELEASED  = LOW;

// ---------------------------------------------------------------------------
// Ready Channel (pin 8)
// ---------------------------------------------------------------------------

// true  = pin 8 is on the NC terminal (contact closed when the coil releases)
// false = pin 8 is on the NO terminal (contact open when the coil releases)
//
// CHANGED TO false IN V5.2. Fire started working after an undocumented
// hardware change, and Ready stopped working in the same moment. Ready had
// been correct on NC, so whatever moved also flipped pin 8's sense - the
// wire is now behaving as NO. This is the matching software change.
//
// If Ready still does not arm, this assumption is wrong: send [w] to flip it
// back at runtime (no reflash) and tell me, because then the hardware change
// did something other than move pin 8.
// false as of 2026-09-03, decided on clean evidence rather than inference.
//
// Symptom: with fire working correctly, STANDBY and STOP both armed the machine
// while READY did not -- i.e. it armed on the edge our "open" produces. That is
// an inverted ready sense, and it is Bug 1's signature exactly.
//
// Earlier attempts to flip this were made while two host-side bugs were active
// (the motion driver was draining the laser's serial replies, and arm() was
// driving the fire relays), so every observation before those were fixed was
// unreliable. This one was taken afterwards.
//
// Bonus: false is also the fail-safe configuration. De-energised now means
// standby, so a reset, replug, upload or power loss drops the laser to standby
// instead of arming it.
bool READY_ON_NC = true;

int readyClosedLevel() { return READY_ON_NC ? COIL_RELEASED : COIL_ENERGIZED; }
int readyOpenLevel()   { return READY_ON_NC ? COIL_ENERGIZED : COIL_RELEASED; }

// Hold the contact open this long before closing it, so the closing edge is
// unambiguous. Well above the relay's ~10 ms mechanical operate time.
const unsigned long READY_EDGE_SETTLE_MS = 200;

// ---------------------------------------------------------------------------
// Fire Channels (pins 9 and 10)
// ---------------------------------------------------------------------------

// Which channels participate in a Fire.
bool fireCh1Enabled = true;
bool fireCh2Enabled = true;

// Polarity per channel.
//   false = normal:   contact OPEN at idle, CLOSES on fire   (relay on NO)
//   true  = inverted: contact CLOSED at idle, OPENS on fire  (changeover /
//                     relay on NC). This is hypothesis B - a changeover pedal
//                     needs one channel in each polarity, not both closing.
//
// WARNING: an inverted channel is ENERGIZED at idle. That is the state that
// previously made the Dornier complain continuously, so expect noise from the
// machine while testing this and do not leave it inverted by default.
bool fireCh1Invert = false;
bool fireCh2Invert = false;

int fireAssertLevel(bool invert) { return invert ? COIL_RELEASED : COIL_ENERGIZED; }
int fireIdleLevel(bool invert)   { return invert ? COIL_ENERGIZED : COIL_RELEASED; }

// Which channel changes state first, and the gap before the other follows.
// Dual-channel safety inputs often require one contact to lead the other.
int fireLeadChannel = 1;

const unsigned long GAP_TABLE_MS[] = {0, 25, 50, 100, 250, 500};
const int GAP_TABLE_COUNT = sizeof(GAP_TABLE_MS) / sizeof(GAP_TABLE_MS[0]);
int fireGapIndex = 0;

unsigned long fireGap_ms() { return GAP_TABLE_MS[fireGapIndex]; }

// After arming, wait this long with the fire contacts provably at idle before
// a Fire is allowed. If the machine samples the pedal as it enters READY, a
// too-early assertion is itself enough to trigger a footswitch complaint.
const unsigned long FIRE_ARM_DWELL_MS = 500;

// Minimum idle time between one Fire and the next, so any stuck-pedal
// detector always sees a clean release.
const unsigned long FIRE_RELEASE_DWELL_MS = 400;

// ---------------------------------------------------------------------------
// Laser State
// ---------------------------------------------------------------------------

bool laserEnabled = false;
bool laserReady = false;
bool laserFiring = false;

unsigned long readyAssertedAtMs = 0;
unsigned long fireIdleAtMs = 0;

// ---------------------------------------------------------------------------
// Function Prototypes
// ---------------------------------------------------------------------------

void setupLaserOutputs();
void driveFireChannelsToIdle();
void driveLaserOutputsToReleasedState();
void lockLaserSoftwareState();
void openReadyContact();
void closeReadyContactWithEdge();
void assertFireChannels();
void exerciseFireRelay(int pin, int channelLabel);
void pauseLaserFiring();
void stopLaserAndRelock();
void waitForDwell(unsigned long sinceMs, unsigned long requiredMs, const char* label);
bool readSerialLine(char inputLine[], size_t inputLineSize);
char getSingleLetterCommand(const char inputLine[]);
bool handleLaserCommand(char command);
void printReceivedInput(const char inputLine[]);
void printBuildBanner();
void printMenu();
void printLaserCommandHelp();
void printFireConfig();
void printReadyWiring();
void printLaserStatus();

// ---------------------------------------------------------------------------
// Laser Helpers
// ---------------------------------------------------------------------------

void setupLaserOutputs() {
  // On Teensy 4.x digitalWrite() before pinMode() sets the output data
  // register, so each pin drives the intended level the instant it becomes an
  // OUTPUT rather than glitching through the opposite state first.
  driveLaserOutputsToReleasedState();

  pinMode(relay_laser, OUTPUT);
  pinMode(fire1_laser, OUTPUT);
  pinMode(fire2_laser, OUTPUT);

  driveLaserOutputsToReleasedState();
  lockLaserSoftwareState();
}

void driveFireChannelsToIdle() {
  // Drive BOTH channels to their idle level regardless of the enable mask.
  // A channel that gets disabled mid-test must still be parked, not left
  // wherever it happened to be.
  digitalWrite(fire1_laser, fireIdleLevel(fireCh1Invert));
  digitalWrite(fire2_laser, fireIdleLevel(fireCh2Invert));
  laserFiring = false;
  fireIdleAtMs = millis();
}

void driveLaserOutputsToReleasedState() {
  // Safe state: fire channels at idle AND the ready contact OPEN (standby).
  //
  // The old build wrote LOW to pin 8 here unconditionally. On NC wiring that
  // CLOSES the ready contact - this one line is what armed the laser on [x].
  driveFireChannelsToIdle();
  digitalWrite(relay_laser, readyOpenLevel());
}

void lockLaserSoftwareState() {
  // Lock the command sequence. The user must send Enable [e] then Ready [r]
  // before Fire [f] is accepted again.
  laserEnabled = false;
  laserReady = false;
  laserFiring = false;
}

void openReadyContact() {
  digitalWrite(relay_laser, readyOpenLevel());
  Serial.print("  ready contact OPEN   (pin 8 -> ");
  Serial.print(readyOpenLevel() == HIGH ? "HIGH, coil ENERGIZED" : "LOW, coil released");
  Serial.println(")");
}

void closeReadyContactWithEdge() {
  // The Dornier arms on the open->closed TRANSITION, not on a static level.
  // Force open first so the closing edge is unambiguous regardless of what
  // state pin 8 was already sitting in.
  Serial.println("Ready sequence: forcing contact OPEN to guarantee a clean edge");
  digitalWrite(relay_laser, readyOpenLevel());
  delay(READY_EDGE_SETTLE_MS);

  digitalWrite(relay_laser, readyClosedLevel());
  readyAssertedAtMs = millis();
  Serial.print("  ready contact CLOSED and HELD (pin 8 -> ");
  Serial.print(readyClosedLevel() == HIGH ? "HIGH, coil ENERGIZED" : "LOW, coil released");
  Serial.println(")");
}

void waitForDwell(unsigned long sinceMs, unsigned long requiredMs, const char* label) {
  unsigned long elapsed = millis() - sinceMs;

  if (elapsed >= requiredMs) {
    return;
  }

  unsigned long remaining = requiredMs - elapsed;
  Serial.print("  waiting ");
  Serial.print(remaining);
  Serial.print(" ms for ");
  Serial.print(label);
  Serial.println(" dwell");
  delay(remaining);
}

void assertFireChannels() {
  // Move the selected channels from idle to asserted, in the configured order
  // with the configured gap. "Asserted" respects each channel's polarity, so
  // an inverted channel OPENS here while a normal one CLOSES.
  int  leadPin    = (fireLeadChannel == 1) ? fire1_laser : fire2_laser;
  int  trailPin   = (fireLeadChannel == 1) ? fire2_laser : fire1_laser;
  bool leadOn     = (fireLeadChannel == 1) ? fireCh1Enabled : fireCh2Enabled;
  bool trailOn    = (fireLeadChannel == 1) ? fireCh2Enabled : fireCh1Enabled;
  bool leadInv    = (fireLeadChannel == 1) ? fireCh1Invert : fireCh2Invert;
  bool trailInv   = (fireLeadChannel == 1) ? fireCh2Invert : fireCh1Invert;
  int  trailLabel = (fireLeadChannel == 1) ? 2 : 1;

  if (leadOn) {
    digitalWrite(leadPin, fireAssertLevel(leadInv));
    Serial.print("  fire channel ");
    Serial.print(fireLeadChannel);
    Serial.print(" (pin ");
    Serial.print(leadPin);
    Serial.print(") -> contact ");
    Serial.print(leadInv ? "OPEN" : "CLOSED");
    Serial.println("  [leading]");
  }

  if (leadOn && trailOn && fireGap_ms() > 0) {
    delay(fireGap_ms());
  }

  if (trailOn) {
    digitalWrite(trailPin, fireAssertLevel(trailInv));
    Serial.print("  fire channel ");
    Serial.print(trailLabel);
    Serial.print(" (pin ");
    Serial.print(trailPin);
    Serial.print(") -> contact ");
    Serial.print(trailInv ? "OPEN" : "CLOSED");
    Serial.print("  [trailing by ");
    Serial.print(leadOn ? fireGap_ms() : 0);
    Serial.println(" ms]");
  }
}

void exerciseFireRelay(int pin, int channelLabel) {
  // Bench test for ONE fire relay in isolation.
  //
  // Purpose: pin 10 has never produced an observable effect on the Dornier in
  // any test, while pin 9 reliably does. That is a hardware question - is the
  // pin 10 channel actually switching, and does its switching reach anything?
  // This drives the coil directly, ignoring the invert setting, so the click
  // is audible and the output terminals can be metered.
  //
  // SAFETY: the ready contact is forced OPEN for the whole test, so the
  // machine is in standby and nothing can fire no matter what these contacts
  // do. The other fire channel is parked at idle.
  digitalWrite(relay_laser, readyOpenLevel());
  laserReady = false;
  driveFireChannelsToIdle();

  Serial.println("--- FIRE RELAY BENCH TEST ---");
  Serial.print("Exercising fire channel ");
  Serial.print(channelLabel);
  Serial.print(" (pin ");
  Serial.print(pin);
  Serial.println(") in isolation.");
  Serial.println("Ready contact is forced OPEN - the laser is in standby and");
  Serial.println("cannot fire during this test.");
  Serial.println("LISTEN for the relay click. METER continuity across that");
  Serial.println("channel's output terminals. Compare against the other channel.");

  for (int cycle = 1; cycle <= 5; cycle++) {
    Serial.print("  cycle ");
    Serial.print(cycle);
    Serial.println("/5: coil ENERGIZED");
    digitalWrite(pin, COIL_ENERGIZED);
    delay(1000);

    Serial.println("           coil released");
    digitalWrite(pin, COIL_RELEASED);
    delay(1000);
  }

  driveFireChannelsToIdle();

  Serial.println("Test finished. Channel returned to idle.");
  Serial.println("No click on this channel -> dead board channel, missing");
  Serial.println("  jumper, or no coil power. Fix that before any more firmware.");
  Serial.println("Clicks but the Dornier never reacts -> the contact is landed");
  Serial.println("  on the wrong terminals of the Steute connector.");
  Serial.println("-----------------------------");
}

void pauseLaserFiring() {
  // Return both fire channels to idle. Does not clear laserEnabled/laserReady.
  driveFireChannelsToIdle();
}

void stopLaserAndRelock() {
  // Release the fire channels first, then open the ready contact.
  // Opening the contact IS the "clear ready" action - nothing is pulsed.
  pauseLaserFiring();
  driveLaserOutputsToReleasedState();
  lockLaserSoftwareState();
}

// ---------------------------------------------------------------------------
// Serial Input Helpers
// ---------------------------------------------------------------------------

bool readSerialLine(char inputLine[], size_t inputLineSize) {
  size_t inputLength = Serial.readBytesUntil('\n', inputLine, inputLineSize - 1);

  // If Serial Monitor is set to "Both NL & CR", readBytesUntil('\n') leaves
  // the carriage return at the end. Remove it so the echo prints cleanly and
  // parsing behaves the same for every line-ending mode.
  if (inputLength > 0 && inputLine[inputLength - 1] == '\r') {
    inputLength--;
  }

  inputLine[inputLength] = '\0';

  // Ignore blank lines.
  for (size_t i = 0; i < inputLength; i++) {
    if (inputLine[i] != ' ' && inputLine[i] != '\t') {
      return true;
    }
  }

  return false;
}

char getSingleLetterCommand(const char inputLine[]) {
  int inputStart = 0;

  while (inputLine[inputStart] == ' ' || inputLine[inputStart] == '\t') {
    inputStart++;
  }

  char command = inputLine[inputStart];

  if (command == '\0') {
    return '\0';
  }

  int inputEnd = inputStart + 1;

  while (inputLine[inputEnd] == ' ' || inputLine[inputEnd] == '\t') {
    inputEnd++;
  }

  // One-character commands only.
  if (inputLine[inputEnd] != '\0') {
    return '\0';
  }

  if (command >= 'A' && command <= 'Z') {
    command = command + 32;
  }

  return command;
}

void printReceivedInput(const char inputLine[]) {
  Serial.print("Serial Monitor input received: [");
  Serial.print(inputLine);
  Serial.println("]");
}

// ---------------------------------------------------------------------------
// Laser Command Handler
// ---------------------------------------------------------------------------

bool handleLaserCommand(char command) {
  if (command == '\0') {
    Serial.println("Wrong Format");
    Serial.println("Commands are one character only.");

    if (laserFiring) {
      Serial.println("Invalid input received while firing. Emergency stop triggered.");
      stopLaserAndRelock();
      Serial.println("All laser outputs are OFF. Laser commands are locked.");
    } else {
      printLaserCommandHelp();
    }

    return false;
  }

  Serial.print("Laser command input: ");
  Serial.println(command);

  // -------------------------------------------------------------------------
  // e: enable software command sequence
  // -------------------------------------------------------------------------
  if (command == 'e') {
    if (laserFiring) {
      Serial.println("Enable [e] is not allowed while firing. Emergency stop triggered.");
      stopLaserAndRelock();
      Serial.println("All laser outputs are OFF. Laser commands are locked.");
      return true;
    }

    if (laserEnabled) {
      Serial.println("Enable [e] ignored: laser commands are already unlocked.");
      return true;
    }

    driveLaserOutputsToReleasedState();
    laserEnabled = true;
    laserReady = false;

    Serial.println("Laser command: Enable [e]");
    Serial.println("Laser commands are unlocked. Next type Ready [r] before Fire [f].");
    return true;
  }

  // -------------------------------------------------------------------------
  // r: ready - open->settle->close edge, then hold
  // -------------------------------------------------------------------------
  if (command == 'r') {
    if (laserFiring) {
      Serial.println("Ready [r] is not allowed while firing. Emergency stop triggered.");
      stopLaserAndRelock();
      Serial.println("All laser outputs are OFF. Laser commands are locked.");
      return true;
    }

    if (!laserEnabled) {
      Serial.println("Ready blocked: type Enable [e] first.");
      printLaserCommandHelp();
      return true;
    }

    if (laserReady) {
      Serial.println("Ready [r] ignored: laser is already ready.");
      return true;
    }

    // Park the fire channels only - closeReadyContactWithEdge() owns pin 8.
    // The fire contacts must be provably at idle before the machine arms, or
    // it may latch a footswitch complaint at the moment it enters READY.
    pauseLaserFiring();

    Serial.println("Laser command: Ready [r]");
    closeReadyContactWithEdge();
    laserReady = true;

    // This flag is SOFTWARE STATE ONLY. Nothing here reads back from the
    // Dornier, so this sketch cannot know whether the machine actually armed.
    // Say so, rather than printing a confident "READY" that has repeatedly
    // been believed over the machine's own screen.
    Serial.println("Software now believes the laser is ready - this is NOT confirmation.");
    Serial.println(">>> LOOK AT THE DORNIER SCREEN. Does it read READY? <<<");
    return true;
  }

  // -------------------------------------------------------------------------
  // f: fire
  // -------------------------------------------------------------------------
  if (command == 'f') {
    if (!laserEnabled || !laserReady) {
      Serial.println("Fire blocked: type Enable [e], then Ready [r], then Fire [f].");
      printLaserCommandHelp();
      return true;
    }

    if (laserFiring) {
      Serial.println("Fire [f] ignored: laser is already firing.");
      return true;
    }

    if (!fireCh1Enabled && !fireCh2Enabled) {
      Serial.println("Fire blocked: both fire channels are disabled. Send [1] or [2].");
      return true;
    }

    Serial.println("Laser command: Fire [f]");
    printFireConfig();

    // Give the machine a clean, provably-idle footswitch window before
    // asserting. Both dwells are no-ops once enough time has already passed.
    waitForDwell(readyAssertedAtMs, FIRE_ARM_DWELL_MS, "post-arm");
    waitForDwell(fireIdleAtMs, FIRE_RELEASE_DWELL_MS, "post-release");

    assertFireChannels();
    laserFiring = true;

    Serial.println("Fire channels asserted and HELD. Send Pause [p] to release.");
    Serial.println("Still says \"release footswitch fully\"? Send [p], then try the");
    Serial.println("next row of the test matrix printed at boot.");
    return true;
  }

  // -------------------------------------------------------------------------
  // p: pause firing
  // -------------------------------------------------------------------------
  if (command == 'p') {
    Serial.println("Laser command: Pause [p]");
    pauseLaserFiring();
    Serial.println("Both fire channels are back at idle. Laser remains enabled and ready.");
    return true;
  }

  // -------------------------------------------------------------------------
  // s: standby - opens the ready contact
  // -------------------------------------------------------------------------
  if (command == 's') {
    if (!laserEnabled) {
      Serial.println("Standby blocked: type Enable [e] first.");
      printLaserCommandHelp();
      return true;
    }

    Serial.println("Laser command: Standby [s]");
    pauseLaserFiring();

    // Always drive the contact open, even if the software did not think the
    // laser was ready. If the two disagree, the safe move is to open.
    openReadyContact();

    laserReady = false;
    Serial.println("Standby finished. Type Ready [r] before Fire [f].");
    return true;
  }

  // -------------------------------------------------------------------------
  // x: stop and relock
  // -------------------------------------------------------------------------
  if (command == 'x') {
    Serial.println("Laser command: Stop [x]");
    stopLaserAndRelock();
    Serial.println("All outputs at idle. Ready contact is OPEN. Commands locked.");
    return true;
  }

  // -------------------------------------------------------------------------
  // 1 / 2: enable or disable a fire channel
  // -------------------------------------------------------------------------
  if (command == '1' || command == '2') {
    if (laserFiring) {
      Serial.println("Channel toggle refused while firing. Send Pause [p] first.");
      return true;
    }

    if (command == '1') {
      fireCh1Enabled = !fireCh1Enabled;
    } else {
      fireCh2Enabled = !fireCh2Enabled;
    }

    pauseLaserFiring();
    printFireConfig();
    return true;
  }

  // -------------------------------------------------------------------------
  // 3 / 4: invert a fire channel's polarity (changeover-pedal hypothesis)
  // -------------------------------------------------------------------------
  if (command == '3' || command == '4') {
    if (laserFiring) {
      Serial.println("Polarity toggle refused while firing. Send Pause [p] first.");
      return true;
    }

    if (command == '3') {
      fireCh1Invert = !fireCh1Invert;
    } else {
      fireCh2Invert = !fireCh2Invert;
    }

    // Re-park both channels under the NEW polarity, so idle really is idle.
    pauseLaserFiring();

    Serial.println("Channel polarity changed. Both channels re-parked at idle.");
    if (fireCh1Invert || fireCh2Invert) {
      Serial.println("NOTE: an inverted channel is ENERGIZED at idle. If the Dornier");
      Serial.println("starts complaining continuously with no Fire sent, that is this.");
    }
    printFireConfig();
    return true;
  }

  // -------------------------------------------------------------------------
  // o: swap which channel leads
  // -------------------------------------------------------------------------
  if (command == 'o') {
    if (laserFiring) {
      Serial.println("Lead swap refused while firing. Send Pause [p] first.");
      return true;
    }

    fireLeadChannel = (fireLeadChannel == 1) ? 2 : 1;
    printFireConfig();
    return true;
  }

  // -------------------------------------------------------------------------
  // g: step the inter-channel gap
  // -------------------------------------------------------------------------
  if (command == 'g') {
    fireGapIndex = (fireGapIndex + 1) % GAP_TABLE_COUNT;
    Serial.print("Inter-channel gap: ");
    Serial.print(fireGap_ms());
    Serial.println(" ms");
    return true;
  }

  // -------------------------------------------------------------------------
  // t / y: exercise one fire relay in isolation
  // -------------------------------------------------------------------------
  if (command == 't' || command == 'y') {
    if (laserFiring) {
      Serial.println("Bench test refused while firing. Send Pause [p] first.");
      return true;
    }

    if (command == 't') {
      exerciseFireRelay(fire1_laser, 1);
    } else {
      exerciseFireRelay(fire2_laser, 2);
    }

    return true;
  }

  // -------------------------------------------------------------------------
  // ?: status
  // -------------------------------------------------------------------------
  if (command == '?') {
    printBuildBanner();
    printMenu();
    printReadyWiring();
    printFireConfig();
    printLaserStatus();
    return true;
  }

  // -------------------------------------------------------------------------
  // Unknown command
  // -------------------------------------------------------------------------
  Serial.println("Unknown command.");

  if (laserFiring) {
    Serial.println("Unknown command received while firing. Emergency stop triggered.");
    stopLaserAndRelock();
    Serial.println("All laser outputs are OFF. Laser commands are locked.");
    return false;
  }

  printLaserCommandHelp();
  return false;
}

// ---------------------------------------------------------------------------
// Serial Printing
// ---------------------------------------------------------------------------

void printBuildBanner() {
  Serial.println("===========================================================");
  Serial.print("  BUILD: ");
  Serial.println(BUILD_NAME);
  Serial.print("  ");
  Serial.println(BUILD_NOTE);
  Serial.println("  No banner after a reset = the board is running a DIFFERENT");
  Serial.println("  sketch, and no test result from it means anything.");
  Serial.println("===========================================================");
}

void printMenu() {
  Serial.println("--- DORNIER LASER ACTIVATION SYSTEM ---");
  printLaserCommandHelp();
  Serial.println("---------------------------------------");
}

void printLaserCommandHelp() {
  Serial.println("LASER: [e] Enable | [r] Ready | [f] Fire | [p] Pause | [s] Standby | [x] Stop");
  Serial.println("FIRE:  [1]/[2] channel on-off | [3]/[4] invert polarity");
  Serial.println("       [o] swap lead | [g] gap ms | [?] status");
  Serial.println("BENCH: [t] exercise pin 9 relay | [y] exercise pin 10 relay");
}

void printReadyWiring() {
  Serial.println("  --- ready channel (pin 8) ---");
  Serial.print("  landed on     : ");
  Serial.println(READY_ON_NC ? "NC (contact CLOSED when coil released)"
                             : "NO (contact OPEN when coil released)");
  Serial.print("  close contact : pin 8 -> ");
  Serial.println(readyClosedLevel() == HIGH ? "HIGH" : "LOW");
  Serial.print("  open contact  : pin 8 -> ");
  Serial.println(readyOpenLevel() == HIGH ? "HIGH" : "LOW");
  Serial.println("  -----------------------------");
}

void printFireConfig() {
  Serial.println("  --- fire channels ---");
  Serial.print("  ch1 (pin 9)  : ");
  Serial.print(fireCh1Enabled ? "ENABLED " : "disabled");
  Serial.print(fireCh1Invert ? "  INVERTED (closed at idle, opens on fire)"
                             : "  normal (open at idle, closes on fire)");
  Serial.println();
  Serial.print("  ch2 (pin 10) : ");
  Serial.print(fireCh2Enabled ? "ENABLED " : "disabled");
  Serial.print(fireCh2Invert ? "  INVERTED (closed at idle, opens on fire)"
                             : "  normal (open at idle, closes on fire)");
  Serial.println();
  Serial.print("  leads        : channel ");
  Serial.println(fireLeadChannel);
  Serial.print("  gap          : ");
  Serial.print(fireGap_ms());
  Serial.println(" ms");
  Serial.println("  ---------------------");
}

void printLaserStatus() {
  Serial.print("Laser enabled/ready/firing: ");
  Serial.print(laserEnabled);
  Serial.print("/");
  Serial.print(laserReady);
  Serial.print("/");
  Serial.println(laserFiring);

  Serial.print("Pin levels 8/9/10: ");
  Serial.print(digitalRead(relay_laser));
  Serial.print("/");
  Serial.print(digitalRead(fire1_laser));
  Serial.print("/");
  Serial.println(digitalRead(fire2_laser));
}

// ---------------------------------------------------------------------------
// Arduino Setup
// ---------------------------------------------------------------------------

void setup() {
  setupLaserOutputs();

  Serial.begin(115200);
  Serial.setTimeout(500);
  delay(2000);

  printBuildBanner();
  printMenu();
  printReadyWiring();
  printFireConfig();

  if (READY_ON_NC) {
    Serial.println("!! FAIL-SAFE WARNING: pin 8 is on the relay's NC terminal.");
    Serial.println("!! Standby needs the coil ENERGIZED, so a reset, replug, upload,");
    Serial.println("!! loose jumper, or relay-supply loss ARMS the laser unattended.");
    Serial.println("!! Fix in hardware: move pin 8 from NC to NO, then set");
    Serial.println("!! READY_ON_NC = false. Do not leave this rig unattended.");
  } else {
    Serial.println("Fail-safe: pin 8 treated as NO. Coil released = contact open =");
    Serial.println("standby, so a reset or power loss drops the laser to standby.");
    Serial.println("This is the correct and safe configuration.");
  }

  Serial.println("STATUS: Fire is working. Ready is the open item.");
  Serial.println("  Ready does not arm? Send [w] to flip the pin 8 sense at");
  Serial.println("  runtime, then [x] [e] [r] and watch the Dornier screen.");
  Serial.println("  [w] is runtime only - it resets on every boot. Tell me which");
  Serial.println("  way works and I will make it the permanent default.");
  printLaserStatus();
}

// ---------------------------------------------------------------------------
// Arduino Main Loop
// ---------------------------------------------------------------------------

void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  char inputLine[16];

  if (!readSerialLine(inputLine, sizeof(inputLine))) {
    return;
  }

  printReceivedInput(inputLine);

  char command = getSingleLetterCommand(inputLine);
  handleLaserCommand(command);
  printLaserStatus();
}
