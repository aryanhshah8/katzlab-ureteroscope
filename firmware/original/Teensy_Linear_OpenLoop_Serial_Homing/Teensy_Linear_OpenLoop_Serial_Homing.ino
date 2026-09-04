/// Teensy 4.1 Linear Open-Loop Serial Control With Homing
/*
  Purpose:
    Control one linear NEMA 14 stepper motor from Serial Monitor commands.

  Open-loop note:
    This sketch does not use encoder wires. Position is estimated only by
    counting the STEP pulses sent to the motor driver. If the motor skips steps
    or the carriage is moved by hand, the estimate will be wrong.

  Startup:
    Put the carriage at the mechanical home position before turning the system on.
    The startup position is treated as 0.00 mm.

  Hardware:
    Teensy 4.1
    NEMA 14 stepper motor
    TB6600 stepper driver

  Pins:
    Pin 2 -> TB6600 ENA
    Pin 3 -> TB6600 DIR
    Pin 4 -> TB6600 STEP

  Serial Monitor:
    Use 115200 baud.

  Commands:
    f = move forward
    b = move backward
    s = stop
    h = return to software home, 0.00 mm
    ? = print help
*/

#include <math.h>
#include <stdint.h>

///----------------------------------------------- Pins ----------------------------------------------------------

const int linear_enaPin = 2;
const int linear_dirPin = 3;
const int linear_stepPin = 4;

///----------------------------------------- Driver Electrical Setup ---------------------------------------------

const int enaPin_Enable = LOW;
const int enaPin_Disable = HIGH;

const int linear_dirForward = HIGH;
const int linear_dirBackward = LOW;

///------------------------------------------ Linear Calibration -------------------------------------------------

const float linearMMPerRev = 4.0f;
const float linearPulsesPerRev = 1600.0f;
const float linearStepsPerMM = linearPulsesPerRev / linearMMPerRev;

const float linearMinPositionMM = 0.0f;
const float linearMaxPositionMM = 177.0f;
const float linearLimitToleranceMM = 0.01f;

const float commandSpeedMMPerSec = 2.0f;
const float linearAccelerationMMPerSec2 = 6.0f;
const float linearDecelerationMMPerSec2 = 10.0f;
const float stoppedSpeedToleranceMMPerSec = 0.005f;

const float homePositionMM = 0.0f;
const float homeToleranceMM = 0.02f;
const float homePositionControllerGain = 8.0f;

const unsigned int stepPulseHighTime_us = 8;
const unsigned int directionSetupTime_us = 10;
const unsigned long statusPrintInterval_ms = 1000;

///---------------------------------------------- Runtime State --------------------------------------------------

bool homeReturnActive = false;

float targetLinearSpeedMMPerSec = 0.0f;
float currentLinearSpeedMMPerSec = 0.0f;
float currentLinearPositionMM = 0.0f;

long currentLinearStepEstimate = 0;
unsigned long totalStepPulses = 0;

bool stepPulseIsHigh = false;
int activeDirectionSign = 0;
unsigned long stepPulseHighStart_us = 0;
unsigned long lastStepRise_us = 0;
unsigned long directionChanged_us = 0;
unsigned long lastVelocityUpdate_us = 0;
unsigned long lastStatusPrint_ms = 0;

///--------------------------------------------- Function Prototypes ---------------------------------------------

void setupStepperPins();
void printStartupMessage();
void printHelp();
void handleSerialCommands();
void moveForward();
void moveBackward();
void startHomeReturn();
void stopMotion();
void updateHomeReturn();
void updateSmoothVelocity();
float chooseRampRate(float currentValue, float targetValue);
float moveValueToward(float currentValue, float targetValue, float maximumChange);
void updateStepperPulseOutput();
bool commandWouldPushPastLimit(float speedMMPerSec);
void updateEstimatedPositionAfterStep(int directionSign);
void printStatusIfReady();
float clampFloat(float value, float minimumValue, float maximumValue);

///---------------------------------------------------- Setup -----------------------------------------------------

void setup() {
  Serial.begin(115200);

  unsigned long serialWaitStart_ms = millis();
  while (!Serial && millis() - serialWaitStart_ms < 4000) {
  }

  setupStepperPins();

  currentLinearPositionMM = 0.0f;
  currentLinearStepEstimate = 0;
  lastVelocityUpdate_us = micros();

  printStartupMessage();
}

void setupStepperPins() {
  pinMode(linear_stepPin, OUTPUT);
  pinMode(linear_dirPin, OUTPUT);
  pinMode(linear_enaPin, OUTPUT);

  digitalWrite(linear_stepPin, LOW);
  digitalWrite(linear_dirPin, linear_dirForward);
  digitalWrite(linear_enaPin, enaPin_Enable);
}

void printStartupMessage() {
  Serial.println();
  Serial.println("============================================================");
  Serial.println("       LINEAR OPEN-LOOP SERIAL CONTROL WITH HOMING");
  Serial.println("============================================================");
  Serial.println("No encoder wires are used.");
  Serial.println("Position is estimated by counting STEP pulses.");
  Serial.println();
  Serial.println("Place carriage at mechanical home before startup.");
  Serial.println("Startup position is treated as 0.00 mm.");
  Serial.println();
  printHelp();
  Serial.println("============================================================");
}

void printHelp() {
  Serial.println("SERIAL COMMANDS");
  Serial.println("  f = move forward");
  Serial.println("  b = move backward");
  Serial.println("  s = stop");
  Serial.println("  h = return to software home, 0.00 mm");
  Serial.println("  ? = print help");
  Serial.println();
}

///----------------------------------------------------- Loop -----------------------------------------------------

void loop() {
  handleSerialCommands();
  updateHomeReturn();
  updateSmoothVelocity();
  updateStepperPulseOutput();
  printStatusIfReady();
}

///--------------------------------------------- Serial Commands -------------------------------------------------

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char command = (char)Serial.read();

    if (command == '\n' || command == '\r' || command == ' ' || command == '\t') {
      continue;
    }

    if (command >= 'A' && command <= 'Z') {
      command = (char)(command + ('a' - 'A'));
    }

    if (command == 'f') {
      moveForward();
    } else if (command == 'b') {
      moveBackward();
    } else if (command == 's') {
      stopMotion();
      Serial.println("STOP");
    } else if (command == 'h') {
      startHomeReturn();
    } else if (command == '?') {
      printHelp();
    } else {
      Serial.println("Unknown command. Type ? for help.");
    }
  }
}

void moveForward() {
  homeReturnActive = false;
  targetLinearSpeedMMPerSec = commandSpeedMMPerSec;

  if (commandWouldPushPastLimit(targetLinearSpeedMMPerSec)) {
    targetLinearSpeedMMPerSec = 0.0f;
    Serial.println("Forward blocked at software maximum limit.");
    return;
  }

  Serial.println("MOVING FORWARD");
}

void moveBackward() {
  homeReturnActive = false;
  targetLinearSpeedMMPerSec = -commandSpeedMMPerSec;

  if (commandWouldPushPastLimit(targetLinearSpeedMMPerSec)) {
    targetLinearSpeedMMPerSec = 0.0f;
    Serial.println("Backward blocked at software home limit.");
    return;
  }

  Serial.println("MOVING BACKWARD");
}

void startHomeReturn() {
  homeReturnActive = true;

  Serial.println();
  Serial.println("HOME requested.");
  Serial.println("Returning to software home, 0.00 mm.");
  Serial.println();
}

void stopMotion() {
  homeReturnActive = false;
  targetLinearSpeedMMPerSec = 0.0f;
  currentLinearSpeedMMPerSec = 0.0f;
  stepPulseIsHigh = false;
  digitalWrite(linear_stepPin, LOW);
}

///-------------------------------------------- Motion Control ---------------------------------------------------

void updateHomeReturn() {
  if (!homeReturnActive) {
    return;
  }

  float errorMM = homePositionMM - currentLinearPositionMM;

  if (fabsf(errorMM) <= homeToleranceMM) {
    homeReturnActive = false;
    currentLinearPositionMM = homePositionMM;
    currentLinearStepEstimate = 0;
    stopMotion();

    Serial.println();
    Serial.println("HOME complete.");
    Serial.println();
    return;
  }

  targetLinearSpeedMMPerSec =
    clampFloat(
      errorMM * homePositionControllerGain,
      -commandSpeedMMPerSec,
      commandSpeedMMPerSec
    );
}

void updateSmoothVelocity() {
  unsigned long now_us = micros();
  float elapsedSec = (float)(now_us - lastVelocityUpdate_us) / 1000000.0f;
  lastVelocityUpdate_us = now_us;

  if (elapsedSec <= 0.0f || elapsedSec > 0.25f) {
    return;
  }

  float rampRate = chooseRampRate(currentLinearSpeedMMPerSec, targetLinearSpeedMMPerSec);
  currentLinearSpeedMMPerSec =
    moveValueToward(
      currentLinearSpeedMMPerSec,
      targetLinearSpeedMMPerSec,
      rampRate * elapsedSec
    );

  if (fabsf(currentLinearSpeedMMPerSec) < stoppedSpeedToleranceMMPerSec &&
      fabsf(targetLinearSpeedMMPerSec) < stoppedSpeedToleranceMMPerSec) {
    currentLinearSpeedMMPerSec = 0.0f;
  }
}

float chooseRampRate(float currentValue, float targetValue) {
  bool sameDirection =
    (currentValue >= 0.0f && targetValue >= 0.0f) ||
    (currentValue <= 0.0f && targetValue <= 0.0f);

  if (sameDirection && fabsf(targetValue) > fabsf(currentValue)) {
    return linearAccelerationMMPerSec2;
  }

  return linearDecelerationMMPerSec2;
}

float moveValueToward(float currentValue, float targetValue, float maximumChange) {
  if (currentValue < targetValue) {
    currentValue += maximumChange;
    if (currentValue > targetValue) currentValue = targetValue;
  } else if (currentValue > targetValue) {
    currentValue -= maximumChange;
    if (currentValue < targetValue) currentValue = targetValue;
  }

  return currentValue;
}

///---------------------------------------------- Step Pulses ----------------------------------------------------

void updateStepperPulseOutput() {
  unsigned long now_us = micros();

  if (stepPulseIsHigh && now_us - stepPulseHighStart_us >= stepPulseHighTime_us) {
    digitalWrite(linear_stepPin, LOW);
    stepPulseIsHigh = false;
  }

  if (stepPulseIsHigh || fabsf(currentLinearSpeedMMPerSec) < stoppedSpeedToleranceMMPerSec) {
    return;
  }

  if (commandWouldPushPastLimit(currentLinearSpeedMMPerSec)) {
    stopMotion();
    Serial.println("Stopped at software travel limit.");
    return;
  }

  int directionSign = currentLinearSpeedMMPerSec > 0.0f ? +1 : -1;

  if (directionSign != activeDirectionSign) {
    activeDirectionSign = directionSign;
    digitalWrite(linear_dirPin, directionSign > 0 ? linear_dirForward : linear_dirBackward);
    directionChanged_us = now_us;
    return;
  }

  if (now_us - directionChanged_us < directionSetupTime_us) {
    return;
  }

  float stepRate = fabsf(currentLinearSpeedMMPerSec) * linearStepsPerMM;
  unsigned long stepInterval_us = (unsigned long)(1000000.0f / stepRate);

  if (stepInterval_us < stepPulseHighTime_us + 2) {
    stepInterval_us = stepPulseHighTime_us + 2;
  }

  if (lastStepRise_us == 0 || now_us - lastStepRise_us >= stepInterval_us) {
    digitalWrite(linear_stepPin, HIGH);
    stepPulseIsHigh = true;
    stepPulseHighStart_us = now_us;
    lastStepRise_us = now_us;
    updateEstimatedPositionAfterStep(directionSign);
  }
}

bool commandWouldPushPastLimit(float speedMMPerSec) {
  if (speedMMPerSec < 0.0f &&
      currentLinearPositionMM <= linearMinPositionMM + linearLimitToleranceMM) {
    return true;
  }

  if (speedMMPerSec > 0.0f &&
      currentLinearPositionMM >= linearMaxPositionMM - linearLimitToleranceMM) {
    return true;
  }

  return false;
}

void updateEstimatedPositionAfterStep(int directionSign) {
  currentLinearStepEstimate += directionSign;
  totalStepPulses++;
  currentLinearPositionMM = (float)currentLinearStepEstimate / linearStepsPerMM;

  if (currentLinearPositionMM < linearMinPositionMM) {
    currentLinearPositionMM = linearMinPositionMM;
    currentLinearStepEstimate = (long)(linearMinPositionMM * linearStepsPerMM);
  }

  if (currentLinearPositionMM > linearMaxPositionMM) {
    currentLinearPositionMM = linearMaxPositionMM;
    currentLinearStepEstimate = (long)(linearMaxPositionMM * linearStepsPerMM);
  }
}

///------------------------------------------------ Status --------------------------------------------------------

void printStatusIfReady() {
  if (millis() - lastStatusPrint_ms < statusPrintInterval_ms) {
    return;
  }

  lastStatusPrint_ms = millis();

  Serial.print("STATUS: position estimate ");
  Serial.print(currentLinearPositionMM, 2);
  Serial.print(" mm | speed ");
  Serial.print(currentLinearSpeedMMPerSec, 2);
  Serial.print(" mm/s | steps ");
  Serial.println(currentLinearStepEstimate);
}

float clampFloat(float value, float minimumValue, float maximumValue) {
  if (value < minimumValue) return minimumValue;
  if (value > maximumValue) return maximumValue;
  return value;
}
