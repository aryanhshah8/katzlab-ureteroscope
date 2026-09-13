
#include <SCServo.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

SMS_STS servo;

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------

const int linear_enaPin = 2;
const int linear_dirPin = 3;
const int linear_stepPin = 4;

const int rotation_enaPin = 5;
const int rotation_dirPin = 6;
const int rotation_stepPin = 7;


const int enaPin_Enable = LOW;
const int enaPin_Disable = HIGH;

// Direction levels, carried over from 3DOF_MovementF.ino.
const int linear_dirForward = LOW;
const int linear_dirBackward = HIGH;
const int rotation_dirClockwise = HIGH;
const int rotation_dirCounter = LOW;

// ---------------------------------------------------------------------------
// Axis calibration
// ---------------------------------------------------------------------------

const float linearMMPerRev = 4.0f;          // TR8x4 lead screw
const float linearPulsesPerRev = 1600.0f;   // match the TB6600 microstep DIPs
const float linearStepsPerMM = linearPulsesPerRev / linearMMPerRev;

const float linearMinPositionMM = 0.0f;
const float linearMaxPositionMM = 177.0f;
const float linearLimitToleranceMM = 0.01f;

const float rotationPulsesPerRev = 1600.0f;
const float rotationGearRatio = 2.5f;       // motor deg per ureteroscope deg
const float rotationStepsPerScopeDeg =
  (rotationPulsesPerRev / 360.0f) * rotationGearRatio;

// Ceilings. The host clamps too, but these are the backstop.
const float linearMaxSpeedMMPerSec = 12.0f;
const float rotationMaxSpeedDegPerSec = 120.0f;

// Ramp rates. These are what make motion smooth: the motor is never commanded
// to jump to speed, so it does not stall or cog on starting.
const float linearAccelMMPerSec2 = 40.0f;
const float linearDecelMMPerSec2 = 80.0f;
const float rotationAccelDegPerSec2 = 400.0f;
const float rotationDecelDegPerSec2 = 800.0f;

const float stoppedLinearTolerance = 0.002f;
const float stoppedRotationTolerance = 0.02f;

const unsigned int stepPulseHighTime_us = 5;
const unsigned int directionSetupTime_us = 20;

// Ramp to a stop if the host goes quiet.
const unsigned long commandTimeout_ms = 400;

// ---------------------------------------------------------------------------
// Flexion (ST3020)
// ---------------------------------------------------------------------------

const long servo_baud = 1000000;
const int servo_ID = 1;
// Servo speed is NOT a constant any more, and that is the whole reason flexion
// used to stutter.
//
// WritePosEx takes a speed in servo steps/s. A fixed high value means the servo
// sprints to whatever target it was last given, arrives, and stops dead until
// the next one -- so jogging came out as a string of little lunges no matter
// how the deadband was tuned. At 8 tip deg/s the tip actually needs about
//   (8 / 1.714) / 360 * 4095  ~=  53 steps/s
// against the 900 it was being told. Seventeen times too fast.
//
// applyFlexionTarget() now measures how fast the host's target is moving and
// sets the servo speed to match, so the servo travels continuously instead of
// arriving early and waiting.
const int servo_acceleration = 60;
// Pinned to a flat 53 steps/s for now (the speed needed at the usual 8
// tip-deg/s jog rate) instead of matching the live target rate.
const int flexionMinServoSpeed = 53;
const int flexionMaxServoSpeed = 53;

// Command the servo a little AHEAD of the target so it always has somewhere to
// travel to. Without this it converges on the current target and stops between
// updates, which is the same stutter by another route.
float flexionLeadSeconds = 0.35f;

// Low-pass on the measured target velocity. The host's updates are quantised by
// its own deadband, so the raw difference is lumpy.
const float flexionVelocityFilter = 0.35f;

float flexionTargetVelocityDegPerSec = 0.0f;
float flexionPrevTargetDeg = 0.0f;
unsigned long flexionLastApply_ms = 0;
int flexionLastSentStep = -1;
const unsigned long flexionApplyInterval_ms = 20;
const int flexionNeutralStep = 2048;
const int servoMaximumStep = 4095;

const float flexionGearRatio = 3.5f;
const float flexionTwistFactor0To90 = 6.0f;
const float flexionTwistFactor90To180 = 6.5f;
const float flexionTwistFactor180To270 = 7.2f;
const float flexionMaxTipDeg = 270.0f;
const float flexionMinTipDeg = -270.0f;

// Only re-command the servo once the target has moved this far.
//
// This has to be sized against the flexion RATE, not picked once. At 30 deg/s a
// 3 deg deadband means a new target every 100 ms, which reads as continuous. At
// 8 deg/s the same deadband is one 3 deg hop every 375 ms -- visibly steppy.
// Small enough that commands stay frequent at the slowest usable rate, but not
// so small that the servo is re-commanded faster than it can act on.
// Runtime-settable with [D<deg>] so it can be tuned against the feel of the rig
// rather than guessed at.
float flexionCommandDeadbandDeg = 0.25f;

bool servoConnected = false;
float flexionTargetTipDeg = 0.0f;
float flexionLastSentTipDeg = 0.0f;

// ---------------------------------------------------------------------------
// Motion state
// ---------------------------------------------------------------------------

float targetLinearSpeed = 0.0f;
float currentLinearSpeed = 0.0f;
float linearPositionMM = 0.0f;
long linearStepEstimate = 0;

float targetRotationSpeed = 0.0f;
float currentRotationSpeed = 0.0f;
float rotationPositionDeg = 0.0f;
long rotationStepEstimate = 0;

bool linearPulseHigh = false;
bool rotationPulseHigh = false;
int linearActiveDir = 0;
int rotationActiveDir = 0;
unsigned long linearPulseHighStart_us = 0;
unsigned long rotationPulseHighStart_us = 0;
unsigned long linearLastRise_us = 0;
unsigned long rotationLastRise_us = 0;
unsigned long linearDirChanged_us = 0;
unsigned long rotationDirChanged_us = 0;
unsigned long lastVelocityUpdate_us = 0;

unsigned long lastCommand_ms = 0;
bool watchdogTripped = false;
unsigned long telemetryPeriod_ms = 100;
unsigned long lastTelemetry_ms = 0;

bool homing = false;
const float homeApproachGain = 1.2f;
const float homeLinearToleranceMM = 0.05f;
const float homeRotationToleranceDeg = 0.5f;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float rampToward(float current, float target, float maxChange) {
  if (current < target) {
    current += maxChange;
    if (current > target) current = target;
  } else if (current > target) {
    current -= maxChange;
    if (current < target) current = target;
  }
  return current;
}

float chooseRamp(float current, float target, float accel, float decel) {
  bool sameDirection = (current >= 0.0f && target >= 0.0f) ||
                       (current <= 0.0f && target <= 0.0f);
  if (sameDirection && fabsf(target) > fabsf(current)) {
    return accel;
  }
  return decel;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------





// Assert the enabled channels in the configured order, with the configured gap.
// "Asserted" respects each channel's polarity, so an inverted channel OPENS
// here while a normal one CLOSES.


// Bench test one fire relay alone. The ready contact is forced OPEN for the
// whole test, so the machine is in standby and nothing can fire whatever these
// contacts do. Listen for the click, meter the output terminals.






void setupSteppers() {
  pinMode(linear_stepPin, OUTPUT);
  pinMode(linear_dirPin, OUTPUT);
  pinMode(linear_enaPin, OUTPUT);
  pinMode(rotation_stepPin, OUTPUT);
  pinMode(rotation_dirPin, OUTPUT);
  pinMode(rotation_enaPin, OUTPUT);

  digitalWrite(linear_stepPin, LOW);
  digitalWrite(rotation_stepPin, LOW);
  digitalWrite(linear_dirPin, linear_dirForward);
  digitalWrite(rotation_dirPin, rotation_dirClockwise);
  digitalWrite(linear_enaPin, enaPin_Enable);
  digitalWrite(rotation_enaPin, enaPin_Enable);
}

void setupFlexion() {
  Serial1.begin(servo_baud);
  servo.pSerial = &Serial1;
  servoConnected = (servo.Ping(servo_ID) != -1);
  if (servoConnected) {
    servo.EnableTorque(servo_ID, 1);
    // One-off move to neutral at startup, so a fixed moderate speed is right
    // here -- the rate matching below only applies to jogging.
    servo.WritePosEx(servo_ID, flexionNeutralStep, 400, servo_acceleration);
  }
}

void setup() {
  setupLaserOutputs();   // relays to a safe state first, always
  setupSteppers();
  Serial.begin(115200);
  Serial.setTimeout(50);
  setupFlexion();

  lastVelocityUpdate_us = micros();
  lastCommand_ms = millis();

  Serial.println();
  Serial.println("=== MOTION VELOCITY SERVER  BUILD VEL-1 ===");
  Serial.print("Flexion servo: ");
  Serial.println(servoConnected ? "connected" : "NOT FOUND");
  Serial.println("L<mm/s> R<deg/s> F<deg> S Z H T<ms> ?   plus laser e r f p s x");
  Serial.println("Carriage start position is treated as 0.00 mm.");
  if (READY_ON_NC) {
    Serial.println("!! pin 8 on NC: de-energised coil = ARMED.");
    Serial.println("!! De-energise the relay supply before flashing.");
  }
}

// ---------------------------------------------------------------------------
// Velocity and pulse generation
// ---------------------------------------------------------------------------

void updateVelocities() {
  unsigned long now_us = micros();
  float dt = (float)(now_us - lastVelocityUpdate_us) / 1000000.0f;
  lastVelocityUpdate_us = now_us;

  if (dt <= 0.0f || dt > 0.25f) {
    return;
  }

  currentLinearSpeed = rampToward(
    currentLinearSpeed, targetLinearSpeed,
    chooseRamp(currentLinearSpeed, targetLinearSpeed,
               linearAccelMMPerSec2, linearDecelMMPerSec2) * dt);

  currentRotationSpeed = rampToward(
    currentRotationSpeed, targetRotationSpeed,
    chooseRamp(currentRotationSpeed, targetRotationSpeed,
               rotationAccelDegPerSec2, rotationDecelDegPerSec2) * dt);

  if (fabsf(currentLinearSpeed) < stoppedLinearTolerance &&
      fabsf(targetLinearSpeed) < stoppedLinearTolerance) {
    currentLinearSpeed = 0.0f;
  }
  if (fabsf(currentRotationSpeed) < stoppedRotationTolerance &&
      fabsf(targetRotationSpeed) < stoppedRotationTolerance) {
    currentRotationSpeed = 0.0f;
  }
}

bool linearWouldExceedLimit(float speed) {
  if (speed < 0.0f && linearPositionMM <= linearMinPositionMM + linearLimitToleranceMM) return true;
  if (speed > 0.0f && linearPositionMM >= linearMaxPositionMM - linearLimitToleranceMM) return true;
  return false;
}

void pulseLinear() {
  unsigned long now_us = micros();

  if (linearPulseHigh && now_us - linearPulseHighStart_us >= stepPulseHighTime_us) {
    digitalWrite(linear_stepPin, LOW);
    linearPulseHigh = false;
  }
  if (linearPulseHigh || fabsf(currentLinearSpeed) < stoppedLinearTolerance) {
    return;
  }
  if (linearWouldExceedLimit(currentLinearSpeed)) {
    currentLinearSpeed = 0.0f;
    targetLinearSpeed = 0.0f;
    return;
  }

  int dir = currentLinearSpeed > 0.0f ? +1 : -1;
  if (dir != linearActiveDir) {
    linearActiveDir = dir;
    digitalWrite(linear_dirPin, dir > 0 ? linear_dirForward : linear_dirBackward);
    linearDirChanged_us = now_us;
    return;
  }
  if (now_us - linearDirChanged_us < directionSetupTime_us) {
    return;
  }

  float stepRate = fabsf(currentLinearSpeed) * linearStepsPerMM;
  if (stepRate <= 0.0f) return;
  unsigned long interval_us = (unsigned long)(1000000.0f / stepRate);
  if (interval_us < stepPulseHighTime_us + 2) interval_us = stepPulseHighTime_us + 2;

  if (linearLastRise_us == 0 || now_us - linearLastRise_us >= interval_us) {
    digitalWrite(linear_stepPin, HIGH);
    linearPulseHigh = true;
    linearPulseHighStart_us = now_us;
    linearLastRise_us = now_us;

    linearStepEstimate += dir;
    linearPositionMM = (float)linearStepEstimate / linearStepsPerMM;
    linearPositionMM = clampf(linearPositionMM, linearMinPositionMM, linearMaxPositionMM);
  }
}

void pulseRotation() {
  unsigned long now_us = micros();

  if (rotationPulseHigh && now_us - rotationPulseHighStart_us >= stepPulseHighTime_us) {
    digitalWrite(rotation_stepPin, LOW);
    rotationPulseHigh = false;
  }
  if (rotationPulseHigh || fabsf(currentRotationSpeed) < stoppedRotationTolerance) {
    return;
  }

  int dir = currentRotationSpeed > 0.0f ? +1 : -1;
  if (dir != rotationActiveDir) {
    rotationActiveDir = dir;
    digitalWrite(rotation_dirPin, dir > 0 ? rotation_dirClockwise : rotation_dirCounter);
    rotationDirChanged_us = now_us;
    return;
  }
  if (now_us - rotationDirChanged_us < directionSetupTime_us) {
    return;
  }

  float stepRate = fabsf(currentRotationSpeed) * rotationStepsPerScopeDeg;
  if (stepRate <= 0.0f) return;
  unsigned long interval_us = (unsigned long)(1000000.0f / stepRate);
  if (interval_us < stepPulseHighTime_us + 2) interval_us = stepPulseHighTime_us + 2;

  if (rotationLastRise_us == 0 || now_us - rotationLastRise_us >= interval_us) {
    digitalWrite(rotation_stepPin, HIGH);
    rotationPulseHigh = true;
    rotationPulseHighStart_us = now_us;
    rotationLastRise_us = now_us;

    rotationStepEstimate += dir;
    rotationPositionDeg = (float)rotationStepEstimate / rotationStepsPerScopeDeg;
  }
}

void stopMotion() {
  targetLinearSpeed = 0.0f;
  targetRotationSpeed = 0.0f;
  currentLinearSpeed = 0.0f;
  currentRotationSpeed = 0.0f;
  digitalWrite(linear_stepPin, LOW);
  digitalWrite(rotation_stepPin, LOW);
  linearPulseHigh = false;
  rotationPulseHigh = false;
  homing = false;
}

// ---------------------------------------------------------------------------
// Flexion
// ---------------------------------------------------------------------------

float flexionTipGain(float tipDeg) {
  float a = fabsf(tipDeg);
  float twist = (a <= 90.0f)  ? flexionTwistFactor0To90
              : (a <= 180.0f) ? flexionTwistFactor90To180
                              : flexionTwistFactor180To270;
  return twist / flexionGearRatio;
}

void applyFlexionTarget() {
  if (!servoConnected) return;

  unsigned long now = millis();
  if (now - flexionLastApply_ms < flexionApplyInterval_ms) return;

  float dt = (float)(now - flexionLastApply_ms) / 1000.0f;
  flexionLastApply_ms = now;
  if (dt <= 0.0f || dt > 0.5f) {
    flexionPrevTargetDeg = flexionTargetTipDeg;
    return;
  }

  // How fast the host is sweeping the target. This is what the servo speed has
  // to match for the motion to come out continuous.
  float rawVelocity = (flexionTargetTipDeg - flexionPrevTargetDeg) / dt;
  flexionPrevTargetDeg = flexionTargetTipDeg;
  flexionTargetVelocityDegPerSec =
    (1.0f - flexionVelocityFilter) * flexionTargetVelocityDegPerSec +
    flexionVelocityFilter * rawVelocity;

  float gain = flexionTipGain(flexionTargetTipDeg);

  // Aim ahead of the target while moving so the servo never converges and
  // stalls between updates. Standing still, lead collapses to zero.
  float commandTipDeg =
    flexionTargetTipDeg + flexionTargetVelocityDegPerSec * flexionLeadSeconds;
  commandTipDeg = clampf(commandTipDeg, flexionMinTipDeg, flexionMaxTipDeg);

  float motorDeg = commandTipDeg / gain;
  if (motorDeg < -180.0f || motorDeg > 180.0f) return;
  int steps = (int)lroundf(((motorDeg + 180.0f) * servoMaximumStep) / 360.0f);

  // Speed matched to the commanded tip rate, in servo steps per second.
  float motorDegPerSec = fabsf(flexionTargetVelocityDegPerSec) / gain;
  int speed = (int)lroundf((motorDegPerSec / 360.0f) * (float)servoMaximumStep);
  speed = (int)clampf((float)speed, (float)flexionMinServoSpeed,
                      (float)flexionMaxServoSpeed);

  // Nothing to say: same step, and not moving.
  bool idle = fabsf(flexionTargetVelocityDegPerSec) < 0.05f;
  if (steps == flexionLastSentStep && idle) return;

  // While idle, respect the deadband so a resting stick does not trickle
  // commands at the servo.
  if (idle && fabsf(flexionTargetTipDeg - flexionLastSentTipDeg) < flexionCommandDeadbandDeg) {
    return;
  }

  servo.WritePosEx(servo_ID, steps, speed, servo_acceleration);
  flexionLastSentStep = steps;
  flexionLastSentTipDeg = flexionTargetTipDeg;
}

// ---------------------------------------------------------------------------
// Homing  (non-blocking: it just steers the velocity targets)
// ---------------------------------------------------------------------------

void updateHoming() {
  if (!homing) return;

  float linearError = -linearPositionMM;
  float rotationError = -rotationPositionDeg;

  bool linearThere = fabsf(linearError) <= homeLinearToleranceMM;
  bool rotationThere = fabsf(rotationError) <= homeRotationToleranceDeg;

  flexionTargetTipDeg = 0.0f;

  if (linearThere && rotationThere) {
    homing = false;
    stopMotion();
    Serial.println("HOME complete");
    return;
  }

  targetLinearSpeed = linearThere ? 0.0f
    : clampf(linearError * homeApproachGain, -linearMaxSpeedMMPerSec, linearMaxSpeedMMPerSec);
  targetRotationSpeed = rotationThere ? 0.0f
    : clampf(rotationError * homeApproachGain * 10.0f,
             -rotationMaxSpeedDegPerSec, rotationMaxSpeedDegPerSec);
}

// ---------------------------------------------------------------------------
// Laser commands
// ---------------------------------------------------------------------------





// ---------------------------------------------------------------------------
// Command parsing
// ---------------------------------------------------------------------------

void emitTelemetry() {
  Serial.print("TEL L");
  Serial.print(linearPositionMM, 3);
  Serial.print(" R");
  Serial.print(rotationPositionDeg, 2);
  Serial.print(" F");
  Serial.print(flexionTargetTipDeg, 2);
  Serial.print(" VL");
  Serial.print(currentLinearSpeed, 3);
  Serial.print(" VR");
  Serial.print(currentRotationSpeed, 2);
  Serial.print(" E");
  Serial.print(laserEnabled ? 1 : 0);
  Serial.print(" Y");
  Serial.print(laserReady ? 1 : 0);
  Serial.print(" G");
  Serial.print(laserFiring ? 1 : 0);
  Serial.print(" H");
  Serial.println(homing ? 1 : 0);
}

void handleLine(char *line) {
  lastCommand_ms = millis();

  // Trim leading space.
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  char c = *line;

  // Length is checked BEFORE dispatching. Written the other way round, the
  // handleLaserCommand() call sits earlier in the && chain and therefore RUNS
  // -- side effects and all -- for any multi-character line that happens to
  // begin with a laser letter. A stray "s..." or "f..." would have fired or
  // stood the machine down as a side effect of parsing.
  if (line[1] == '\0' && c >= 'a' && c <= 'z' && handleLaserCommand(c)) {
    printLaserStatus();
    return;
  }
  if (c >= '1' && c <= '4' && line[1] == '\0') {
    handleLaserCommand(c);
    printLaserStatus();
    return;
  }

  switch (c) {
    case 'L': case 'l':
      homing = false;
      targetLinearSpeed = clampf(atof(line + 1), -linearMaxSpeedMMPerSec, linearMaxSpeedMMPerSec);
      return;

    case 'R':
      homing = false;
      targetRotationSpeed = clampf(atof(line + 1), -rotationMaxSpeedDegPerSec, rotationMaxSpeedDegPerSec);
      return;

    case 'F':
      flexionTargetTipDeg = clampf(atof(line + 1), flexionMinTipDeg, flexionMaxTipDeg);
      return;

    case 'S':
      stopMotion();
      Serial.println("STOPPED");
      return;

    case 'Z':
      linearStepEstimate = 0;
      rotationStepEstimate = 0;
      linearPositionMM = 0.0f;
      rotationPositionDeg = 0.0f;
      Serial.println("ZEROED");
      return;

    case 'H':
      homing = true;
      Serial.println("HOMING");
      return;

    case 'P': {
      // Fire-channel polarity as an absolute mask, bit0 = ch1, bit1 = ch2.
      // The 3/4 toggles are fine by hand but useless from a host: they flip
      // relative to whatever state the board happens to be in, so reconnecting
      // without a reboot inverts the wrong way. This is idempotent.
      int mask = atoi(line + 1);
      fireCh1Invert = (mask & 1) != 0;
      fireCh2Invert = (mask & 2) != 0;
      driveFireChannelsToIdle();
      Serial.print("Fire polarity: ch1 ");
      Serial.print(fireCh1Invert ? "INVERTED" : "normal");
      Serial.print(", ch2 ");
      Serial.println(fireCh2Invert ? "INVERTED" : "normal");
      return;
    }

    case 'A':
      flexionLeadSeconds = clampf(atof(line + 1), 0.0f, 2.0f);
      Serial.print("Flexion lead ");
      Serial.print(flexionLeadSeconds, 3);
      Serial.println(" s");
      return;

    case 'D':
      flexionCommandDeadbandDeg = clampf(atof(line + 1), 0.02f, 10.0f);
      Serial.print("Flexion deadband ");
      Serial.print(flexionCommandDeadbandDeg, 2);
      Serial.println(" deg");
      return;

    case 'T':
      telemetryPeriod_ms = (unsigned long)atol(line + 1);
      Serial.print("Telemetry period ");
      Serial.println(telemetryPeriod_ms);
      return;

    case '?':
      emitTelemetry();
      printLaserStatus();
      return;

    default:
      Serial.print("Unknown command: ");
      Serial.println(line);
      return;
  }
}

void readSerial() {
  static char buffer[48];
  static uint8_t length = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (length > 0) {
        buffer[length] = '\0';
        handleLine(buffer);
        length = 0;
      }
      continue;
    }
    if (length < sizeof(buffer) - 1) {
      buffer[length++] = c;
    }
  }
}

// ---------------------------------------------------------------------------
// Main loop  -  nothing in here blocks
// ---------------------------------------------------------------------------

void loop() {
  readSerial();

  // Host went quiet: ramp to a stop rather than run on at the last velocity.
  // Announced, not silent -- an unexplained stop mid-jog is indistinguishable
  // from a mechanical fault, and a host that stops sending because its command
  // did not change looks exactly like a host that died.
  if (millis() - lastCommand_ms > commandTimeout_ms) {
    if (!watchdogTripped &&
        (fabsf(targetLinearSpeed) > 0.0f || fabsf(targetRotationSpeed) > 0.0f)) {
      Serial.print("WATCHDOG: no command for ");
      Serial.print(millis() - lastCommand_ms);
      Serial.println(" ms, ramping to a stop");
      watchdogTripped = true;
    }
    targetLinearSpeed = 0.0f;
    targetRotationSpeed = 0.0f;
    homing = false;
  } else {
    watchdogTripped = false;
  }

  updateHoming();
  updateVelocities();
  pulseLinear();
  pulseRotation();
  applyFlexionTarget();

  if (telemetryPeriod_ms > 0 && millis() - lastTelemetry_ms >= telemetryPeriod_ms) {
    lastTelemetry_ms = millis();
    emitTelemetry();
  }
}
