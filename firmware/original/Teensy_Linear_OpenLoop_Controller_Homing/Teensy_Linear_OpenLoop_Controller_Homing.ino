/// Teensy 4.1 Linear Open-Loop Controller With Joystick Homing
/*
  Purpose:
    Control one linear NEMA 14 stepper motor with the left joystick horizontal
    axis on a USB game controller.

  Open-loop note:
    This sketch does not use encoder wires. Position is estimated only by
    counting the STEP pulses sent to the motor driver. If the motor skips steps
    or the carriage is moved by hand, the estimate will be wrong.

  Startup:
    Put the carriage at the mechanical home position before turning the system on.
    The startup position is treated as 0.00 mm.

  Hardware:
    Teensy 4.1
    USB game controller plugged into the Teensy USB host port
    NEMA 14 stepper motor
    TB6600 stepper driver

  Pins:
    Pin 2 -> TB6600 ENA
    Pin 3 -> TB6600 DIR
    Pin 4 -> TB6600 STEP

  Controls:
    Left joystick horizontal -> move linear axis
    B button                 -> return to software home, 0.00 mm

  Serial Monitor:
    Use 115200 baud.
*/

#include <USBHost_t36.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

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

const float maximumLinearSpeedMMPerSec = 2.0f;
const float linearAccelerationMMPerSec2 = 6.0f;
const float linearDecelerationMMPerSec2 = 10.0f;
const float stoppedSpeedToleranceMMPerSec = 0.005f;

const float homePositionMM = 0.0f;
const float homeToleranceMM = 0.02f;
const float homePositionControllerGain = 8.0f;

const unsigned int stepPulseHighTime_us = 8;
const unsigned int directionSetupTime_us = 10;
const unsigned long statusPrintInterval_ms = 1000;

///--------------------------------------------- USB Controller --------------------------------------------------

USBHost usbHost;
USBHub usbHub(usbHost);
USBHIDParser hidParser(usbHost);
JoystickController joystick(usbHost);

const int maximumJoystickAxes = 64;
const int leftJoystickXAxisIndex = 0;
const int leftJoystickYAxisGuardIndex = 1;
const int linearJoystickDirectionSign = +1;
const int linearGuardJoystickDirectionSign = -1;
const float joystickDeadZoneFraction = 0.12f;
const float leftStickAllowedCrossAxisMotion = 0.12f;
const int32_t minimumLiveAxisHalfRange = 100;

const uint32_t homeButtonMask = 0x00000002;  // B button on the original controller

struct JoystickAxis {
  int axisIndex;
  int directionSign;
  int32_t minimum;
  int32_t center;
  int32_t maximum;
  int32_t raw;
  float normalized;
  bool rangeInitialized;
};

JoystickAxis linearAxis = {
  leftJoystickXAxisIndex,
  linearJoystickDirectionSign,
  0,
  0,
  0,
  0,
  0.0f,
  false
};

JoystickAxis linearGuardAxis = {
  leftJoystickYAxisGuardIndex,
  linearGuardJoystickDirectionSign,
  0,
  0,
  0,
  0,
  0.0f,
  false
};

///---------------------------------------------- Runtime State --------------------------------------------------

bool controllerWasConnected = false;
bool controlsReady = false;
bool homeButtonWasPressed = false;
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
void updateControllerConnection();
void processJoystickPacket();
void updateNormalizedJoystickValue(JoystickAxis *axis);
void updateLiveAxisRange(JoystickAxis *axis);
bool axisIsPresent(int axisIndex);
float filterHorizontalOnly(float horizontalValue, float verticalValue);
float applyJoystickResponseCurve(float value);
void updateHomeButton(uint32_t currentButtons);
void startHomeReturn();
void updateTargetFromJoystick();
void updateHomeReturn();
void updateSmoothVelocity();
float chooseRampRate(float currentValue, float targetValue);
float moveValueToward(float currentValue, float targetValue, float maximumChange);
void updateStepperPulseOutput();
bool commandWouldPushPastLimit(float speedMMPerSec);
void updateEstimatedPositionAfterStep(int directionSign);
void stopMotion();
void printStatusIfReady();
float clampFloat(float value, float minimumValue, float maximumValue);

///---------------------------------------------------- Setup -----------------------------------------------------

void setup() {
  Serial.begin(115200);

  unsigned long serialWaitStart_ms = millis();
  while (!Serial && millis() - serialWaitStart_ms < 4000) {
  }

  setupStepperPins();
  usbHost.begin();

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
  Serial.println("     LINEAR OPEN-LOOP JOYSTICK CONTROL WITH HOMING");
  Serial.println("============================================================");
  Serial.println("Left joystick horizontal moves the linear axis.");
  Serial.println("B button returns to software home, 0.00 mm.");
  Serial.println("No encoder wires are used.");
  Serial.println();
  Serial.println("Place carriage at mechanical home before startup.");
  Serial.println("Waiting for controller...");
  Serial.println("============================================================");
}

///----------------------------------------------------- Loop -----------------------------------------------------

void loop() {
  usbHost.Task();
  updateControllerConnection();

  if (joystick && joystick.available()) {
    processJoystickPacket();
    joystick.joystickDataClear();
  }

  updateTargetFromJoystick();
  updateHomeReturn();
  updateSmoothVelocity();
  updateStepperPulseOutput();
  printStatusIfReady();
}

///------------------------------------------ Controller Input ----------------------------------------------------

void updateControllerConnection() {
  bool controllerIsConnected = (bool)joystick;

  if (controllerIsConnected && !controllerWasConnected) {
    controllerWasConnected = true;
    controlsReady = true;
    linearAxis.rangeInitialized = false;
    linearGuardAxis.rangeInitialized = false;
    homeButtonWasPressed = false;

    Serial.println();
    Serial.println("CONTROLLER CONNECTED");
    Serial.print("VID:PID 0x");
    Serial.print(joystick.idVendor(), HEX);
    Serial.print(":0x");
    Serial.println(joystick.idProduct(), HEX);
    Serial.println("Keep the left joystick centered briefly.");
    Serial.println();
  }

  if (!controllerIsConnected && controllerWasConnected) {
    controllerWasConnected = false;
    controlsReady = false;
    stopMotion();

    Serial.println();
    Serial.println("CONTROLLER DISCONNECTED");
    Serial.println("Motion stopped. Waiting for controller...");
    Serial.println();
  }
}

void processJoystickPacket() {
  updateNormalizedJoystickValue(&linearAxis);
  updateNormalizedJoystickValue(&linearGuardAxis);
  updateHomeButton(joystick.getButtons());
}

void updateNormalizedJoystickValue(JoystickAxis *axis) {
  if (!axisIsPresent(axis->axisIndex)) {
    axis->normalized = 0.0f;
    return;
  }

  axis->raw = joystick.getAxis(axis->axisIndex);
  updateLiveAxisRange(axis);

  int32_t positiveHalfSpan = axis->maximum - axis->center;
  int32_t negativeHalfSpan = axis->center - axis->minimum;
  int32_t halfSpan = positiveHalfSpan > negativeHalfSpan ? positiveHalfSpan : negativeHalfSpan;

  if (halfSpan < minimumLiveAxisHalfRange) {
    halfSpan = minimumLiveAxisHalfRange;
  }

  int32_t deadZone = (int32_t)((float)halfSpan * joystickDeadZoneFraction);
  int32_t delta = axis->raw - axis->center;

  if (labs(delta) <= deadZone) {
    axis->normalized = 0.0f;
    return;
  }

  if (delta > 0) {
    axis->normalized = (float)(delta - deadZone) / (float)(halfSpan - deadZone);
  } else {
    axis->normalized = (float)(delta + deadZone) / (float)(halfSpan - deadZone);
  }

  axis->normalized = clampFloat(axis->normalized, -1.0f, 1.0f);
  axis->normalized *= axis->directionSign;
}

void updateLiveAxisRange(JoystickAxis *axis) {
  if (!axis->rangeInitialized) {
    axis->minimum = axis->raw;
    axis->center = axis->raw;
    axis->maximum = axis->raw;
    axis->rangeInitialized = true;
    return;
  }

  if (axis->raw < axis->minimum) axis->minimum = axis->raw;
  if (axis->raw > axis->maximum) axis->maximum = axis->raw;
}

bool axisIsPresent(int axisIndex) {
  if (axisIndex < 0 || axisIndex >= maximumJoystickAxes) {
    return false;
  }

  return (joystick.axisMask() & ((uint64_t)1 << axisIndex)) != 0;
}

float filterHorizontalOnly(float horizontalValue, float verticalValue) {
  if (fabsf(verticalValue) <= leftStickAllowedCrossAxisMotion) {
    return horizontalValue;
  }

  return 0.0f;
}

float applyJoystickResponseCurve(float value) {
  float cubicValue = value * value * value;
  return 0.35f * value + 0.65f * cubicValue;
}

void updateHomeButton(uint32_t currentButtons) {
  bool homeButtonIsPressed = (currentButtons & homeButtonMask) != 0;

  if (homeButtonIsPressed && !homeButtonWasPressed) {
    startHomeReturn();
  }

  homeButtonWasPressed = homeButtonIsPressed;
}

///-------------------------------------------- Motion Control ---------------------------------------------------

void updateTargetFromJoystick() {
  if (!controlsReady || homeReturnActive) {
    return;
  }

  float joystickCommand =
    filterHorizontalOnly(linearAxis.normalized, linearGuardAxis.normalized);

  joystickCommand = applyJoystickResponseCurve(joystickCommand);
  targetLinearSpeedMMPerSec = joystickCommand * maximumLinearSpeedMMPerSec;

  if (commandWouldPushPastLimit(targetLinearSpeedMMPerSec)) {
    targetLinearSpeedMMPerSec = 0.0f;
  }
}

void startHomeReturn() {
  homeReturnActive = true;

  Serial.println();
  Serial.println("HOME requested.");
  Serial.println("Returning to software home, 0.00 mm.");
  Serial.println();
}

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
      -maximumLinearSpeedMMPerSec,
      maximumLinearSpeedMMPerSec
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

void stopMotion() {
  homeReturnActive = false;
  targetLinearSpeedMMPerSec = 0.0f;
  currentLinearSpeedMMPerSec = 0.0f;
  stepPulseIsHigh = false;
  digitalWrite(linear_stepPin, LOW);
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

  Serial.print("STATUS: ");
  Serial.print(controllerWasConnected ? "controller connected" : "waiting for controller");
  Serial.print(" | position estimate ");
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
