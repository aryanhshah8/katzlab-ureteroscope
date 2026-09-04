/// 3-DOF Movement - Serial Monitor
/*
Overall Purpose:
  Control three ureteroscope motion axes from the Serial Monitor using one Teensy 4.1
  Linear and rotation inputs are relative movement commands from the current tracked position
  Flexion input is an absolute ureteroscope tip-angle target
  The program runs linear, rotation, and flexion during the same movement window

Hardware:
  Teensy 4.1
  NEMA 14 stepper motor for linear carriage movement
  TB6600 driver for the linear NEMA 14 stepper motor
  TR8x4 lead screw for linear travel
  Linear quadrature encoder
  NEMA 17 stepper motor for ureteroscope rotation
  TB6600 driver for the rotation NEMA 17 stepper motor
  Rotation quadrature encoder
  ST3020 serial bus servo for ureteroscope tip flexion
  Waveshare bus servo adapter or equivalent ST3020 serial interface
  External motor and servo power supplies
  Shared ground between the Teensy, motor drivers, encoders, and servo system

Pins:
  Pin 2 connects to the linear TB6600 ENA input
  Pin 3 connects to the linear TB6600 DIR input
  Pin 4 connects to the linear TB6600 PUL or STEP input
  Pin 17 connects to linear encoder channel A
  Pin 16 connects to linear encoder channel B
  Pin 5 connects to the rotation TB6600 ENA input
  Pin 6 connects to the rotation TB6600 DIR input
  Pin 7 connects to the rotation TB6600 PUL or STEP input
  Pin 20 connects to rotation encoder channel A
  Pin 21 connects to rotation encoder channel B
  Pin 0 is Teensy RX1 for Serial1 ST3020 communication
  Pin 1 is Teensy TX1 for Serial1 ST3020 communication

Other:
  Enter three numbers in one line as Linear mm relative, Rotation degrees relative, Flexion degrees absolute
  Example input 4 90 20 moves +4 mm, rotates +90 degrees, and targets +20 degrees flexion
  Type home, HOME, or h to return linear, rotation, and flexion to their starting points
  Linear motion uses the NEMA 14 motor with TB6600 driver and TR8x4 lead screw
  Rotation uses the NEMA 17 motor with TB6600 driver and 2.5 to 1 gear ratio
  Tip flexion uses the ST3020 serial bus servo
*/

///------------------------------------------------ ST3020 Setup ---------------------------------------------------

// Include the library used to communicate with the ST3020
#include <SCServo.h>

// Include math functions such as fabsf, lroundf, and isfinite
#include <math.h>

// Create the ST3020 communication object
SMS_STS servo;

///------------------------------------------------ Pin Assignments ------------------------------------------------

// Linear Axis using NEMA 14 and TB6600
const int linear_enaPin = 2;   // Pin 2 sends enable control to the linear TB6600 driver ENA input
const int linear_dirPin = 3;   // Pin 3 sends direction control to the linear TB6600 driver DIR input
const int linear_stepPin = 4;  // Pin 4 sends step pulses to the linear TB6600 driver PUL or STEP input

// Linear Motor Encoder Channels
const int linear_encA = 17;  // Pin 17 reads linear encoder channel A
const int linear_encB = 16;  // Pin 16 reads linear encoder channel B

// Rotation Axis using NEMA 17 and TB6600
const int rotation_enaPin = 5;   // Pin 5 sends enable control to the rotation TB6600 driver ENA input
const int rotation_dirPin = 6;   // Pin 6 sends direction control to the rotation TB6600 driver DIR input
const int rotation_stepPin = 7;  // Pin 7 sends step pulses to the rotation TB6600 driver PUL or STEP input

// Rotation Motor Encoder Channels
const int rotation_encA = 20;  // Pin 20 reads rotation encoder channel A
const int rotation_encB = 21;  // Pin 21 reads rotation encoder channel B

// ST3020 and Waveshare Bus Servo Adapter Communication Pins Documentation
// Serial1 automatically uses pins 0 and 1 on Teensy 4.1
// const int servo_rxPin = 0; // Pin 0 is Teensy RX1
// const int servo_txPin = 1; // Pin 1 is Teensy TX1

///--------------------------------------------- Electrical Settings ----------------------------------------------

// Enabled / Disabled Settings
// LOW enables the TB6600 driver with this wiring
// HIGH disables the TB6600 driver with this wiring
const int enaPin_Enable = LOW;
const int enaPin_Disable = HIGH;

// Direction Levels for the Linear Axis
const int linear_dirForward = LOW;    // was HIGH
const int linear_dirBackward = HIGH;  // was LOW

// Direction Levels for the Rotation Axis
const int rotation_dirClockwise = HIGH;   // flipped from LOW to reverse rotation direction
const int rotation_dirCounter = LOW;      // flipped from HIGH to reverse rotation direction

// Common conversion numbers written as named constants so the math is easier to read
const int degreesPerRevolution = 360;
const int halfRevolutionDegrees = 180;
const int servoMaximumStep = 4095;

// Status printing interval during movement
const unsigned long printInterval_ms = 40;

///---------------------------------------------- Linear Settings -------------------------------------------------

// TR8x4 Lead Screw moves 4 mm during 1 motor revolution
const float linearMMPerRev = 4.0;

// Number of STEP pulses required for 1 motor revolution
// Change this value to match the TB6600 microstep setting
const float linearPulsesPerRev = 1600.0;

// Encoder counts per motor revolution when using 1x decoding on Channel A
const float linearEncoderCPR = 1000.0;

// Stop when the measured linear motion is this close to the requested motion
// 0.005 mm is approximately 1 encoder count because 4 mm / 1000 counts = 0.004 mm per count
const float linearStopMarginMM = 0.005;

// Absolute TR8x4 lead screw travel range
const float linearMinPositionMM = 0.0;
const float linearMaxPositionMM = 177.0;

// Small tolerance used only to avoid rejecting tiny floating-point roundoff near 0 or 177
// 0.01 mm is approximately 2.5 encoder counts at the boundary because 0.01 / 0.004 = 2.5
const float linearRangeToleranceMM = 0.01;

// Change this value to +1 or -1 if the displayed encoder direction is reversed
// If absolute position decreases during a positive linear move, change this sign
const int linearEncoderSign = -1;   // was +1

// volatile means this variable can change inside an interrupt at any time
volatile long linearEncoderCount = 0;

// This variable stores the tracked absolute linear position of the TR8x4 lead screw
// The code starts this at 0 mm during setup, so the hardware must physically begin at 0 mm
float currentLinearPositionMM = 0.0;

// Estimated linear movement for one pulse = 4 / 1600 = 0.0025 mm

///--------------------------------------------- Rotation Settings ------------------------------------------------

// Number of encoder counts for 1 motor revolution when using 1x decoding
const float rotationEncoderCPR = 1000.0;

// Number of STEP pulses required for 1 rotation motor revolution
// This is used for printing and for a backup stop if encoder feedback fails
const float rotationPulsesPerRev = 1600.0;

// Motor rotates 2.5 degrees for every 1 degree of ureteroscope rotation
const float rotationGearRatio = 2.5;

// Stop when the motor encoder is within this many motor degrees of the target
// 0.18 motor degrees is approximately half an encoder count because 360 / 1000 = 0.36 motor degrees per count
// Through the 2.5 to 1 gear ratio, 0.18 motor degrees is about 0.072 ureteroscope degrees
const float rotationStopMarginMotorDeg = 0.18;

// Change this value to +1 or -1 if the displayed encoder direction is reversed
const int rotationEncoderSign = +1;

// volatile allows the interrupt routine to update this value at any time
volatile long rotationEncoderCount = 0;

// This variable stores the mapped absolute rotation position of the ureteroscope
// The code starts this at 0 degrees during setup, so the ureteroscope must physically begin flat
// The map wraps full revolutions so +360 degrees and -360 degrees become 0 degrees again
float currentRotationMapDeg = 0.0;

// Small tolerance used to clean the rotation map back to 0 near a full revolution
const float rotationMapZeroToleranceDeg = 0.5;

///---------------------------------------------- Flexion Settings ------------------------------------------------

// ST3020 Communication and Movement Settings
const long servo_baud = 1000000;
const int servo_ID = 1;
const int servo_speed = 500;         // Slow steady movement
const int servo_acceleration = 100;  // Soft ramp-up profile

// Flexion Speed Reduction Settings
// The ST3020 uses a larger speed command for faster movement and a smaller speed command for slower movement
// The target tip angle is used as a precautionary speed limit for each flexion command
// Targets from -100 to +100 tip degrees use the normal servo_speed of 500
const int flexionSlowdownStartTipDeg = 100;

// Above 100 tip degrees subtract 2.5 speed units for every additional tip degree
// Examples with a normal speed of 500:
// 150 tip degrees gives 500 - (50 * 2.5) = 375
// 200 tip degrees gives 500 - (100 * 2.5) = 250
// 250 tip degrees gives 500 - (150 * 2.5) = 125
const float flexionSpeedReductionPerTipDeg = 2.5;

// Do not allow the calculated speed command to fall below this value
const int flexionMinimumSpeed = 50;

// Flexion Overstretch Warning Levels use the absolute target angle
// This means the same warning is printed for positive and negative flexion
const int flexionSmallWarningTipDeg = 150;
const int flexionModerateWarningTipDeg = 200;
const int flexionMajorWarningTipDeg = 250;

// ST3020 Center Position used as 0 deg Base Motor Angle and 0 deg Base Tip Flexion
const int flexionNeutralStep = 2048;

// Servo Target Tolerance in encoder steps
// 1 ST3020 step is about 0.088 motor degrees because 360 / 4095 = 0.088
// Tip degrees per ST3020 step depend on the selected calibration range
const int flexionStopMarginSteps = 1;

///---------------------------------------------- FLEXION CALIBRATION --------------------------------------------
// Positive and negative targets use the same ranges:
//   0 through 90 degrees       uses 6.0
//   over 90 through 180        uses 6.5
//   over 180 through 270       uses 7.2
const float flexionTwistFactor0To90 = 6.0;
const float flexionTwistFactor90To180 = 6.5;
const float flexionTwistFactor180To270 = 7.2;

const float flexionGearRatio = 3.5;

// Flexion Max and Min 360 Servo Movement
// Counts = encoder counts
// Steps = physical steps
// Neutral: 0 degrees = 2048 counts/steps
// The ST3020 conversion represents -180 through +180 motor degrees around neutral
const float flexionMaxMotorDeg = 180.0;
const float flexionMinMotorDeg = -180.0;

// The allowed absolute ureteroscope tip range is -270 through +270 degrees
const float flexionMaxTipDeg = 270.0;
const float flexionMinTipDeg = -270.0;

// Flexion range tolerance equivalent to one ST3020 encoder count
// 1 count is about 0.088 motor degrees because 360 / 4095 = 0.088
const float flexionRangeToleranceMotorDeg = degreesPerRevolution / (float)servoMaximumStep;
const float flexionRangeToleranceTipDeg =
  flexionRangeToleranceMotorDeg * (flexionTwistFactor180To270 / flexionGearRatio);

// Stop waiting if the ST3020 encoder has not changed for this long
// This prevents endless printing when the servo reaches a mechanical limit before the exact target
const unsigned long flexionNoChangeTimeout_ms = 500;

///----------------------------------------------- Motion Timing --------------------------------------------------

// STEP signal stays HIGH for 800 us and LOW for 800 us
// One pulse takes 1600 us which is approximately 625 pulses per second
const unsigned int stepDelay_us = 800;

// Short wait after setting direction and before sending the first STEP pulse
const unsigned int directionSetup_us = 10;

///------------------------------------------------ Input Variables ------------------------------------------------

// True once ST3020 responds during startup diagnostics
bool servoConnected = false;

// These three variables store the one-line user command
// Example input "4 90 20" means +4 mm relative linear movement,
// +90 degrees relative rotation, and an absolute +20-degree flexion target
float requestedLinearMM = 0.0;
float requestedRotationDeg = 0.0;
float requestedFlexionDeg = 0.0;

// True only when the user types the explicit home command or h shortcut
// In that case the code calculates the relative movement needed to return to home state
bool homeCommandRequested = false;

///----------------------------------------------- Linear Target State --------------------------------------------

// Linear input is relative movement from the current absolute position
float targetLinearRelativeMM = 0.0;

// This is the absolute linear position the code expects after the command
float targetLinearAbsoluteMM = 0.0;

// This is the positive travel distance used for stopping
float targetLinearMagnitudeMM = 0.0;

// Motor revolutions and pulses needed for the requested linear motion
float targetLinearRevolutions = 0.0;
long targetLinearPulses = 0;

// Direction sign for the requested linear movement
int linearMovementSign = +1;

// Runtime state for the linear axis
bool linearDone = true;
long linearPulsesSent = 0;
long signedLinearCount = 0;
float measuredLinearMotorDeg = 0.0;
float measuredLinearUreteroMM = 0.0;

///---------------------------------------------- Rotation Target State -------------------------------------------
// Rotation input is relative ureteroscope rotation in degrees
float targetUreteroscopeRotationDeg = 0.0;

// Required motor angle for the requested ureteroscope rotation
float targetRotationMotorDeg = 0.0;
float targetRotationMotorMagnitudeDeg = 0.0;

// Estimated motor pulses used for printing and backup stop
long targetRotationPulses = 0;

// Direction sign for the requested rotation movement
int rotationMovementSign = +1;

// Runtime state for the rotation axis
bool rotationDone = true;
long rotationPulsesSent = 0;
long signedRotationCount = 0;
float measuredRotationMotorDeg = 0.0;
float measuredUreteroRotationDeg = 0.0;

///---------------------------------------------- Flexion Target State --------------------------------------------

// Flexion input is the desired absolute tip flexion angle
float targetTipDeg = 0.0;

// These are selected for each command from the absolute target-angle range
float selectedFlexionTwistFactor = flexionTwistFactor0To90;
float selectedFlexionTipGain = flexionTwistFactor0To90 / flexionGearRatio;

// Required ST3020 motor angle for the requested tip angle
float targetFlexionMotorDeg = 0.0;

// Physical ST3020 angle after shifting -180 to +180 onto 0 to 360
float targetFlexionPhysicalDeg = 0.0;

// ST3020 target encoder step
int targetFlexionSteps = flexionNeutralStep;

// Speed selected from the absolute value of the requested tip angle
// A larger flexion target above 100 degrees produces a smaller and slower speed command
int selectedFlexionSpeed = servo_speed;

// Runtime state for the flexion axis
bool flexionDone = true;
int currentFlexionStep = -1;
int lastMovingFlexionStep = -1;
float measuredFlexionMotorDeg = 0.0;
float measuredTipDeg = 0.0;

///---------------------------------------------- Timing State -----------------------------------------------------

// These are reset at the beginning of every command
unsigned long lastPrintTime = 0;
unsigned long lastFlexionReadTime = 0;
unsigned long lastFlexionChangeTime = 0;

///--------------------------------------------- Function Prototypes ----------------------------------------------

// Setup Functions
void linearSetup();
void rotationSetup();
void flexionSetup();

// Startup and Prompt Functions
void diagnostics();
void linearDiagnostics();
void rotationDiagnostics();
void flexionDiagnostics();
void printFlexionTuningSummary();
void printLinearZeroReminder();
void printRotationNeutralReminder();
void printCommandPrompt();

// Serial Input Functions
bool readMovementCommand();
void clearSerialInput();

// Target Calculation Functions
bool calculateAllTargets();
bool calculateLinearTarget();
bool calculateRotationTarget();
float selectFlexionTwistFactor(float tipDeg);
bool calculateFlexionTarget();
bool linearTargetIsInsideRange();
bool flexionTargetIsInsideRange();
int calculateFlexionSpeed();
void printFlexionOverstretchWarning();

// Movement Organization Functions
void moveAllAxesTogether();
void resetMovementMeasurements();
void printRequestedTargets();
void prepareMotorsForMovement();
void startFlexionMovement();
void runSimultaneousMovementLoop();
void pulseActiveSteppers();
void readLinearFeedback();
void readRotationFeedback();
void readFlexionFeedbackIfReady();
void printMovementStatusIfReady();
void updateAbsoluteLinearPosition();
void updateAbsoluteRotationPosition();
void printFinalSummary();

// Encoder Functions
void linearEncoderISR();
void rotationEncoderISR();
long getLinearEncoderCount();
long getRotationEncoderCount();
void resetLinearEncoder();
void resetRotationEncoder();

// General Function
void disableAllMotors();
bool inputLineEqualsWord(const char inputLine[], const char word[]);
void setHomeCommandTargets();
float normalizeRotationMapDeg(float angleDeg);

///-------------------------------------------------- Linear Setup -------------------------------------------------

void linearSetup() {
  // Configure TB6600 movement pins as Teensy outputs
  pinMode(linear_stepPin, OUTPUT);
  pinMode(linear_dirPin, OUTPUT);
  pinMode(linear_enaPin, OUTPUT);

  // Begin with no STEP pulse and with the driver enabled
  digitalWrite(linear_stepPin, LOW);
  digitalWrite(linear_dirPin, linear_dirForward);
  digitalWrite(linear_enaPin, enaPin_Enable);

  // Enable Teensy internal pull-up resistors on both encoder channels
  pinMode(linear_encA, INPUT_PULLUP);
  pinMode(linear_encB, INPUT_PULLUP);

  // Run linearEncoderISR whenever encoder channel A changes from LOW to HIGH
  attachInterrupt(digitalPinToInterrupt(linear_encA), linearEncoderISR, RISING);

  // Start each command from an encoder count of 0
  resetLinearEncoder();
}

///------------------------------------------------- Rotation Setup ------------------------------------------------

void rotationSetup() {
  // Configure TB6600 movement pins as Teensy outputs
  pinMode(rotation_stepPin, OUTPUT);
  pinMode(rotation_dirPin, OUTPUT);
  pinMode(rotation_enaPin, OUTPUT);

  // Begin with no STEP pulse and with the driver enabled
  digitalWrite(rotation_stepPin, LOW);
  digitalWrite(rotation_dirPin, rotation_dirClockwise);
  digitalWrite(rotation_enaPin, enaPin_Enable);

  // Enable Teensy internal pull-up resistors on both encoder channels
  pinMode(rotation_encA, INPUT_PULLUP);
  pinMode(rotation_encB, INPUT_PULLUP);

  // Run rotationEncoderISR whenever encoder channel A changes from LOW to HIGH
  attachInterrupt(digitalPinToInterrupt(rotation_encA), rotationEncoderISR, RISING);

  // Start each command from an encoder count of 0
  resetRotationEncoder();
}

///-------------------------------------------------- Flexion Setup ------------------------------------------------

void flexionSetup() {
  // Start Serial1 at the ST3020 factory-default communication speed
  // Teensy pin 0 is RX1 and Teensy pin 1 is TX1
  Serial1.begin(servo_baud);

  // Tell the SCServo library to communicate through Serial1
  servo.pSerial = &Serial1;

  // Ask the servo to move to neutral during startup
  servo.WritePosEx(servo_ID, flexionNeutralStep, servo_speed, servo_acceleration);
}

///------------------------------------------------ Diagnostics ---------------------------------------------------

void diagnostics() {
  Serial.println("\n-------------------- DIAGNOSTICS --------------------");
  linearDiagnostics();
  rotationDiagnostics();
  flexionDiagnostics();
  Serial.println("-----------------------------------------------------");
}

void linearDiagnostics() {
  Serial.println("\nLinear Axis Diagnostics");

  // Reading an output pin reports the logic level commanded by the Teensy
  // It does not prove that the motor driver received the signal
  Serial.print("ENA Pin (expected 0): ");
  Serial.println(digitalRead(linear_enaPin));
  Serial.print("DIR Pin: ");
  Serial.println(digitalRead(linear_dirPin));
  Serial.print("STEP Pin: ");
  Serial.println(digitalRead(linear_stepPin));
  Serial.print("Pulses per Motor Revolution: ");
  Serial.println(linearPulsesPerRev, 0);

  // Clear the encoder count before printing the diagnostic count
  resetLinearEncoder();

  Serial.print("Encoder Count: ");
  Serial.println(getLinearEncoderCount());

  Serial.print("Tracked Absolute Linear Position: ");
  Serial.print(currentLinearPositionMM, 3);
  Serial.println(" mm");
}

void rotationDiagnostics() {
  Serial.println("\nRotation Axis Diagnostics");

  // Reading an output pin reports the logic level commanded by the Teensy
  // It does not prove that the motor driver received the signal
  Serial.print("ENA Pin (expected 0): ");
  Serial.println(digitalRead(rotation_enaPin));
  Serial.print("DIR Pin: ");
  Serial.println(digitalRead(rotation_dirPin));
  Serial.print("STEP Pin: ");
  Serial.println(digitalRead(rotation_stepPin));
  Serial.print("Pulses per Motor Revolution: ");
  Serial.println(rotationPulsesPerRev, 0);

  // Clear the encoder count before printing the diagnostic count
  resetRotationEncoder();

  Serial.print("Encoder Count: ");
  Serial.println(getRotationEncoderCount());
}

void flexionDiagnostics() {
  Serial.println("\nFlexion Axis Diagnostics");

  // Ping returns the detected servo ID or -1 when no reply is received
  int detectedID = servo.Ping(servo_ID);

  if (detectedID == -1) {
    servoConnected = false;
    Serial.println("Servo Communication Failed");
    return;
  }

  servoConnected = true;
  Serial.print("Servo Connected with ID: ");
  Serial.println(detectedID);

  // Enable Servo Torque so the ST3020 can hold position and move
  int torqueAcknowledged = servo.EnableTorque(servo_ID, 1);
  Serial.print("Torque Enable Acknowledged: ");
  Serial.println(torqueAcknowledged == 1 ? "YES" : "NO");

  // Read servo supply voltage in tenths of a volt
  int rawVoltage = servo.ReadVoltage(servo_ID);
  Serial.print("Servo Supply Voltage: ");

  if (rawVoltage == -1) {
    Serial.println("READ FAILED");
  } else {
    Serial.print(rawVoltage / 10.0, 1);
    Serial.println(" V");
  }

  // Read servo internal temperature in degrees Celsius
  int temperatureC = servo.ReadTemper(servo_ID);
  Serial.print("Servo Temperature: ");

  if (temperatureC == -1) {
    Serial.println("READ FAILED");
  } else {
    Serial.print(temperatureC);
    Serial.println(" C");
  }
}

void printFlexionTuningSummary() {
  Serial.println("\n---------------- FLEXION CALIBRATION ----------------");
  Serial.print("Twist factor for 0 to 90 deg: ");
  Serial.println(flexionTwistFactor0To90, 4);
  Serial.print("Twist factor for over 90 to 180 deg: ");
  Serial.println(flexionTwistFactor90To180, 4);
  Serial.print("Twist factor for over 180 to 270 deg: ");
  Serial.println(flexionTwistFactor180To270, 4);
  Serial.print("Flexion gear ratio: ");
  Serial.println(flexionGearRatio, 4);
  Serial.print("Allowed absolute tip range: ");
  Serial.print(flexionMinTipDeg, 2);
  Serial.print(" deg to ");
  Serial.print(flexionMaxTipDeg, 2);
  Serial.println(" deg");
  Serial.print("Representable ST3020 motor range: ");
  Serial.print(flexionMinMotorDeg, 2);
  Serial.print(" deg to ");
  Serial.print(flexionMaxMotorDeg, 2);
  Serial.println(" deg");
  Serial.print("Speed reduction per tip degree beyond +/-100 deg: ");
  Serial.println(flexionSpeedReductionPerTipDeg, 1);
  Serial.println("Edit the three factors in the FLEXION CALIBRATION block near the top of this file.");
  Serial.println("-----------------------------------------------------");
}

///------------------------------------------------------ Setup ---------------------------------------------------

void setup() {
  // Open USB communication with the Computer Serial Monitor
  Serial.begin(115200);

  // Limit how long readBytesUntil waits for the rest of the input line
  Serial.setTimeout(500);

  // Setup all three motor axes
  linearSetup();
  rotationSetup();
  flexionSetup();

  // The code assumes the TR8x4 axis starts at the physical 0 mm position
  currentLinearPositionMM = linearMinPositionMM;

  // The code assumes the ureteroscope starts physically flat at the rotation neutral position
  currentRotationMapDeg = 0.0;

  // Give the drivers adapter and servo time to start
  delay(1000);

  // Run startup diagnostics
  diagnostics();
  printFlexionTuningSummary();

  // Print the important absolute-position reminder after diagnostics
  printLinearZeroReminder();
  printRotationNeutralReminder();

  Serial.println("\nMotor Setup Complete");
  Serial.println("Enter one command with three numbers separated by spaces");
  Serial.println("Linear is RELATIVE, Rotation is RELATIVE, Flexion is ABSOLUTE");
  Serial.println("Example: 4 90 20");
  Serial.println("Type home or h to return to the home state");
}

void printLinearZeroReminder() {
  Serial.println("\n---------------- LINEAR ZERO REMINDER ----------------");
  Serial.println("Before sending movement commands, make sure the TR8x4 lead screw begins at 0 mm");
  Serial.println("This code will treat the startup position as absolute 0 mm");
  Serial.print("Allowed absolute linear range: ");
  Serial.print(linearMinPositionMM, 1);
  Serial.print(" mm to ");
  Serial.print(linearMaxPositionMM, 1);
  Serial.println(" mm");
  Serial.println("------------------------------------------------------");
}

void printRotationNeutralReminder() {
  Serial.println("\n-------------- ROTATION NEUTRAL REMINDER ------------");
  Serial.println("Before sending movement commands, make sure the ureteroscope is flat at rotation neutral");
  Serial.println("This code does not enforce rotation min/max range");
  Serial.println("This startup flat position becomes 0 degrees on the rotation map");
  Serial.println("Rotation commands are relative, but the code tracks mapped absolute rotation");
  Serial.println("A full revolution back to 0 in either direction resets the map to 0 degrees");
  Serial.println("------------------------------------------------------");
}

///------------------------------------------------------- Loop ----------------------------------------------------

void loop() {
  // Show current absolute position before asking for the next command
  printCommandPrompt();

  // Wait here until the Serial Monitor has at least one character available
  while (Serial.available() == 0) {
  }

  // If the line cannot be converted into exactly three numbers, do not move
  if (!readMovementCommand()) {
    Serial.println("Wrong Format");
    clearSerialInput();
    return;
  }

  // Calculate target values and reject impossible movement before any motor moves
  if (!calculateAllTargets()) {
    clearSerialInput();
    return;
  }

  // Run all axes during the same movement window
  moveAllAxesTogether();
}

void printCommandPrompt() {
  // The first number is RELATIVE linear movement from the current tracked position
  // The second number is RELATIVE rotation in ureteroscope degrees
  // The third number is an ABSOLUTE tip-flexion target in degrees
  // Type home or h to return all three axes to the software home state
  Serial.println("\nPOSITIONING: Linear = RELATIVE | Rotation = RELATIVE | Flexion = ABSOLUTE");
  Serial.println("Enter Linear Additional(mm) Rotation Additional(deg) Flexion Target(deg):");

  Serial.print("Current absolute linear position: ");
  Serial.print(currentLinearPositionMM, 3);
  Serial.println(" mm");

  Serial.print("Current mapped rotation position: ");
  Serial.print(currentRotationMapDeg, 2);
  Serial.println(" deg");

  Serial.print("Linear range allowed: ");
  Serial.print(linearMinPositionMM, 1);
  Serial.print(" mm to ");
  Serial.print(linearMaxPositionMM, 1);
  Serial.println(" mm");

  Serial.println("Return to home state with: home OR h");
}

///------------------------------------------------ Serial Input --------------------------------------------------

bool readMovementCommand() {
  // Read one complete line from the Serial Monitor
  // The character array keeps the input small and avoids dynamic memory use
  char inputLine[64];

  // Reset this flag before reading each new command
  homeCommandRequested = false;

  // readBytesUntil stops at newline or when Serial timeout is reached
  size_t inputLength = Serial.readBytesUntil('\n', inputLine, sizeof(inputLine) - 1);

  // Add a null terminator so sscanf knows where the text ends
  inputLine[inputLength] = '\0';

  // The word home returns all three axes to the software home state
  // This check ignores capitalization, so home, Home, and HOME all work
  // The single letter h also works as a shortcut
  if (inputLineEqualsWord(inputLine, "home") || inputLineEqualsWord(inputLine, "h")) {
    setHomeCommandTargets();
    return true;
  }

  // A fourth value or any extra text makes the command invalid
  // sscanf tries to read Linear, Rotation, Flexion, and then one extra character
  // A correct command reads only the first three numbers
  char extraCharacter;
  int valuesRead = sscanf(inputLine, " %f %f %f %c", &requestedLinearMM, &requestedRotationDeg, &requestedFlexionDeg, &extraCharacter);

  if (valuesRead != 3) {
    // Wrong number of values, text, commas, or extra values will arrive here
    return false;
  }

  if (!isfinite(requestedLinearMM) || !isfinite(requestedRotationDeg) || !isfinite(requestedFlexionDeg)) {
    // Reject unusual values such as NaN (Not a Number) or infinity
    return false;
  }

  return true;
}

void clearSerialInput() {
  // Remove carriage return newline and extra characters after the command
  while (Serial.available() > 0) {
    Serial.read();
  }
}

bool inputLineEqualsWord(const char inputLine[], const char word[]) {
  // Compare a Serial Monitor line with a single word while ignoring capitalization
  // Leading and trailing spaces are ignored

  // Skips Leading Whitespace
  // Start at the very beginning index (0) of the input line
  int inputStart = 0;
  
  // Move the starting index forward as long as the character is a space, tab, or carriage return
  while (inputLine[inputStart] == ' ' || inputLine[inputStart] == '\t' || inputLine[inputStart] == '\r') {
    inputStart++; // Move to the next character
  }

  // Find the End of the String
  // Start looking for the end of the text, beginning from where the actual text starts
  int inputEnd = inputStart;
  
  // Loop until we find the null terminator ('\0'), which marks the absolute end of a C-string
  while (inputLine[inputEnd] != '\0') {
    inputEnd++; // Move forward until we find the end marker
  }

  // Trim Trailing Whitespace
  // Work backward from the end of the string to look for spaces, tabs, or returns
  // Make sure we don't accidentally back up past our starting point (inputEnd > inputStart)
  while (inputEnd > inputStart &&
         (inputLine[inputEnd - 1] == ' ' ||
          inputLine[inputEnd - 1] == '\t' ||
          inputLine[inputEnd - 1] == '\r')) {
    inputEnd--; // Pull the end boundary back by one character
  }

  // Measure the Target Word
  int wordLength = 0;
  // Count the characters in the target word until we hit its null terminator
  while (word[wordLength] != '\0') {
    wordLength++;
  }

  // Quick Length Check
  // (inputEnd - inputStart) calculates the exact length of the cleaned-up input line
  // If the lengths don't match, they can't possibly be the same word
  if (inputEnd - inputStart != wordLength) {
    return false; // Exit early because the lengths are different
  }

  // Character-by-Character Comparison
  // Loop through each character of both strings based on the target word's length
  for (int i = 0; i < wordLength; i++) {
    // Grab the current character from the input line (offset by where the real text started)
    char inputChar = inputLine[inputStart + i];
    // Grab the current character from the target word
    char wordChar = word[i];

    // Convert inputChar to lowercase if it is an uppercase letter ('A' through 'Z')
    // Adding 32 shifts the ASCII value from uppercase to lowercase
    if (inputChar >= 'A' && inputChar <= 'Z') {
      inputChar = inputChar + 32;
    }

    // Convert wordChar to lowercase if it is an uppercase letter
    if (wordChar >= 'A' && wordChar <= 'Z') {
      wordChar = wordChar + 32;
    }

    // Now that both are lowercase, compare them 
    // If they don't match, the words are not the same
    if (inputChar != wordChar) {
      return false; // Mis-match found, exit and return false
    }
  }

  // If the code successfully makes it through the entire loop without returning false,
  // it means the cleaned-up input perfectly matches the target word
  return true;
}

void setHomeCommandTargets() {
  // The home state is linear 0 mm, rotation map 0 degrees, and flexion tip 0 degrees
  // Linear and rotation are controlled by relative moves, so the code calculates the needed relative command
  // Flexion is already an absolute tip target, so its home command is simply 0 degrees

  homeCommandRequested = true;

  requestedLinearMM = linearMinPositionMM - currentLinearPositionMM;
  requestedRotationDeg = -currentRotationMapDeg;
  requestedFlexionDeg = 0.0;

  Serial.println("\nHome command received");
  Serial.println("Target home state: Linear 0 mm, Rotation 0 deg, Flexion 0 deg");
}

///-------------------------------------------- Target Calculations -----------------------------------------------

bool calculateAllTargets() {
  // All targets are calculated before motion begins
  // This lets the code reject the full command before any motor starts moving

  if (!calculateLinearTarget()) {
    return false;
  }

  if (!calculateRotationTarget()) {
    return false;
  }

  if (!calculateFlexionTarget()) {
    return false;
  }

  return true;
}

bool calculateLinearTarget() {
  // Linear input is additional movement relative to the current tracked absolute position
  targetLinearRelativeMM = requestedLinearMM;

  // This is the absolute position the linear axis would reach if the command is allowed
  targetLinearAbsoluteMM = currentLinearPositionMM + targetLinearRelativeMM;

  // Reject the entire 3-DOF command if linear motion would go outside 0 mm to 177 mm
  if (!linearTargetIsInsideRange()) {
    return false;
  }

  // Positive input moves forward and negative input moves backward
  linearMovementSign = (targetLinearRelativeMM > 0.0 ? +1 : -1);

  // Use absolute value because stopping only cares about travel distance
  targetLinearMagnitudeMM = fabsf(targetLinearRelativeMM);

  // Convert requested millimeters into motor revolutions
  targetLinearRevolutions = targetLinearMagnitudeMM / linearMMPerRev;

  // Convert motor revolutions into STEP pulses for the TB6600
  targetLinearPulses = lroundf(targetLinearRevolutions * linearPulsesPerRev);

  // If targetLinearPulses is 0, there is no linear movement to execute
  linearDone = (targetLinearPulses == 0);

  return true;
}

bool linearTargetIsInsideRange() {
  // Tiny tolerance prevents floating-point roundoff from rejecting values at exactly 0 or 177
  bool belowMinimum = targetLinearAbsoluteMM < linearMinPositionMM - linearRangeToleranceMM;
  bool aboveMaximum = targetLinearAbsoluteMM > linearMaxPositionMM + linearRangeToleranceMM;

  if (belowMinimum || aboveMaximum) {
    Serial.println("\nLinear Range Error");
    Serial.println("No motors will move because the linear command would leave the allowed range");

    Serial.print("Current absolute linear position: ");
    Serial.print(currentLinearPositionMM, 3);
    Serial.println(" mm");

    Serial.print("Requested additional linear movement: ");
    Serial.print(targetLinearRelativeMM, 3);
    Serial.println(" mm");

    Serial.print("Requested absolute linear position: ");
    Serial.print(targetLinearAbsoluteMM, 3);
    Serial.println(" mm");

    Serial.print("Allowed absolute linear range: ");
    Serial.print(linearMinPositionMM, 1);
    Serial.print(" mm to ");
    Serial.print(linearMaxPositionMM, 1);
    Serial.println(" mm");

    return false;
  }

  // If the target is almost exactly the minimum, clean it to exactly 0
  if (targetLinearAbsoluteMM < linearMinPositionMM) {
    targetLinearAbsoluteMM = linearMinPositionMM;
  }

  // If the target is almost exactly the maximum, clean it to exactly 177
  if (targetLinearAbsoluteMM > linearMaxPositionMM) {
    targetLinearAbsoluteMM = linearMaxPositionMM;
  }

  return true;
}

bool calculateRotationTarget() {
  // Rotation input is relative ureteroscope rotation in degrees
  targetUreteroscopeRotationDeg = requestedRotationDeg;

  // Positive rotation input and negative rotation input use opposite directions
  rotationMovementSign = (targetUreteroscopeRotationDeg > 0.0 ? +1 : -1);

  // Convert ureteroscope rotation into required motor rotation
  targetRotationMotorDeg = rotationGearRatio * targetUreteroscopeRotationDeg;

  // Use absolute value because stopping only cares about angle traveled
  targetRotationMotorMagnitudeDeg = fabsf(targetRotationMotorDeg);

  // Estimate STEP pulses from motor degrees
  // Encoder feedback is still the main stopping method
  targetRotationPulses = lroundf((targetRotationMotorMagnitudeDeg / degreesPerRevolution) * rotationPulsesPerRev);

  // If targetRotationPulses is 0, there is no rotation movement to execute
  rotationDone = (targetRotationPulses == 0);

  return true;
}

float selectFlexionTwistFactor(float tipDeg) {
  // Use absolute angle so the same calibration ranges apply in both directions
  float absoluteTipDeg = fabsf(tipDeg);

  if (absoluteTipDeg <= 90.0) {
    return flexionTwistFactor0To90;
  }

  if (absoluteTipDeg <= 180.0) {
    return flexionTwistFactor90To180;
  }

  return flexionTwistFactor180To270;
}

bool calculateFlexionTarget() {
  // Flexion input is the desired absolute tip flexion angle
  targetTipDeg = requestedFlexionDeg;

  // Select one twist factor from the absolute target-angle range
  selectedFlexionTwistFactor = selectFlexionTwistFactor(targetTipDeg);
  selectedFlexionTipGain = selectedFlexionTwistFactor / flexionGearRatio;

  // Convert desired tip angle into required ST3020 motor angle
  targetFlexionMotorDeg = targetTipDeg / selectedFlexionTipGain;

  // Reject the entire 3-DOF command if the requested flexion is outside the servo range
  if (!flexionTargetIsInsideRange()) {
    return false;
  }

  // Shift motor angle from -180 to +180 into the servo's 0 to 360 scale
  targetFlexionPhysicalDeg = targetFlexionMotorDeg + halfRevolutionDegrees;

  // Convert physical motor angle into ST3020 encoder steps
  targetFlexionSteps = lroundf((targetFlexionPhysicalDeg * servoMaximumStep) / degreesPerRevolution);

  // Select a slower speed when the requested absolute tip angle is above 100 degrees
  selectedFlexionSpeed = calculateFlexionSpeed();

  // Print one warning that matches the largest warning level reached by this target
  printFlexionOverstretchWarning();

  // If the servo is not connected, do not wait for flexion feedback
  flexionDone = !servoConnected;

  return true;
}

int calculateFlexionSpeed() {
  // Use absolute value so +150 degrees and -150 degrees receive the same speed
  float absoluteTargetTipDeg = fabsf(targetTipDeg);

  // Targets through 100 degrees use the normal slow steady speed
  if (absoluteTargetTipDeg <= flexionSlowdownStartTipDeg) {
    return servo_speed;
  }

  // Find how many tip degrees the target is beyond the 100 degree slowdown point
  float degreesBeyondSlowdown = absoluteTargetTipDeg - flexionSlowdownStartTipDeg;

  // Reduce the speed by 2.5 speed units for each additional tip degree
  int speedReduction = lroundf(degreesBeyondSlowdown * flexionSpeedReductionPerTipDeg);
  int calculatedSpeed = servo_speed - speedReduction;

  // Keep the command from becoming slower than the backup minimum speed
  if (calculatedSpeed < flexionMinimumSpeed) {
    calculatedSpeed = flexionMinimumSpeed;
  }

  return calculatedSpeed;
}

void printFlexionOverstretchWarning() {
  // Warning thresholds are based on absolute tip angle so they apply in both directions
  float absoluteTargetTipDeg = fabsf(targetTipDeg);

  // Print only the highest warning level reached to keep the Serial Monitor clear
  // A target within one encoder count of the allowed maximum receives the maximum warning
  if (absoluteTargetTipDeg >= flexionMaxTipDeg - flexionRangeToleranceTipDeg) {
    Serial.println("WARNING: Reaching maximum allowed flexion");
  } else if (absoluteTargetTipDeg >= flexionMajorWarningTipDeg) {
    Serial.println("MAJOR PRECAUTION: Flexion target is near the maximum and may overstretch the ureteroscope");
  } else if (absoluteTargetTipDeg >= flexionModerateWarningTipDeg) {
    Serial.println("WARNING: High flexion target may overstretch the ureteroscope");
  } else if (absoluteTargetTipDeg >= flexionSmallWarningTipDeg) {
    Serial.println("CAUTION: Flexion target is entering the overstretch precaution range");
  }
}

bool flexionTargetIsInsideRange() {
  // Reject commands outside the exact -270 to +270 degree tip range or the
  // ST3020's representable -180 to +180 degree motor range
  bool belowTipMinimum = targetTipDeg < flexionMinTipDeg;
  bool aboveTipMaximum = targetTipDeg > flexionMaxTipDeg;
  bool belowMotorMinimum = targetFlexionMotorDeg < flexionMinMotorDeg;
  bool aboveMotorMaximum = targetFlexionMotorDeg > flexionMaxMotorDeg;

  if (belowTipMinimum || aboveTipMaximum || belowMotorMinimum || aboveMotorMaximum) {
    Serial.println("\nFlexion Range Error");
    Serial.println("No motors will move because the requested flexion is outside the allowed range");

    Serial.print("Requested tip flexion: ");
    Serial.print(targetTipDeg, 2);
    Serial.println(" deg");

    Serial.print("Required ST3020 motor angle: ");
    Serial.print(targetFlexionMotorDeg, 2);
    Serial.println(" deg");

    Serial.print("Selected flexionTwistFactor: ");
    Serial.println(selectedFlexionTwistFactor, 4);

    Serial.print("Selected flexionTipGain: ");
    Serial.println(selectedFlexionTipGain, 4);

    Serial.print("Representable ST3020 motor range: ");
    Serial.print(flexionMinMotorDeg, 1);
    Serial.print(" deg to ");
    Serial.print(flexionMaxMotorDeg, 1);
    Serial.println(" deg");

    Serial.print("Allowed requested tip range: ");
    Serial.print(flexionMinTipDeg, 1);
    Serial.print(" deg to ");
    Serial.print(flexionMaxTipDeg, 1);
    Serial.println(" deg");

    return false;
  }

  return true;
}

///----------------------------------------------- Encoder Interrupts --------------------------------------------

void linearEncoderISR() {
  // Read channel B every time channel A rises to determine encoder direction
  // 1x decoding means only one edge of channel A is counted
  if (digitalRead(linear_encB) == HIGH) {
    linearEncoderCount++;
  } else {
    linearEncoderCount--;
  }
}

void rotationEncoderISR() {
  // Read channel B every time channel A rises to determine encoder direction
  // 1x decoding means only one edge of channel A is counted
  if (digitalRead(rotation_encB) == HIGH) {
    rotationEncoderCount++;
  } else {
    rotationEncoderCount--;
  }
}

long getLinearEncoderCount() {
  // Pause interrupts so the encoder count cannot change halfway through reading it
  noInterrupts();
  long count = linearEncoderCount;
  interrupts();
  return count;
}

long getRotationEncoderCount() {
  // Pause interrupts so the encoder count cannot change halfway through reading it
  noInterrupts();
  long count = rotationEncoderCount;
  interrupts();
  return count;
}

void resetLinearEncoder() {
  // Clear the linear encoder count as one uninterrupted operation
  noInterrupts();
  linearEncoderCount = 0;
  interrupts();
}

void resetRotationEncoder() {
  // Clear the rotation encoder count as one uninterrupted operation
  noInterrupts();
  rotationEncoderCount = 0;
  interrupts();
}

///--------------------------------------------- Simultaneous Movement --------------------------------------------

void moveAllAxesTogether() {
  // This function is intentionally a short organizer
  // Each major part of the movement has its own smaller function below
  Serial.println("\nStarting 3-DOF Movement");

  resetMovementMeasurements();
  printRequestedTargets();
  prepareMotorsForMovement();
  runSimultaneousMovementLoop();
  updateAbsoluteLinearPosition();
  updateAbsoluteRotationPosition();
  printFinalSummary();
}

void resetMovementMeasurements() {
  // Runtime counters and measured values are cleared at the start of each command
  linearPulsesSent = 0;
  rotationPulsesSent = 0;

  signedLinearCount = 0;
  signedRotationCount = 0;

  measuredLinearMotorDeg = 0.0;
  measuredLinearUreteroMM = 0.0;
  measuredRotationMotorDeg = 0.0;
  measuredUreteroRotationDeg = 0.0;

  currentFlexionStep = -1;
  lastMovingFlexionStep = -1;
  measuredFlexionMotorDeg = 0.0;
  measuredTipDeg = 0.0;

  lastPrintTime = 0;
  lastFlexionReadTime = 0;
  lastFlexionChangeTime = millis();
}

void printRequestedTargets() {
  // Print everything the code calculated before motion begins
  // This helps check whether the math matches the intended movement

  Serial.print("Linear current absolute position: ");
  Serial.print(currentLinearPositionMM, 3);
  Serial.println(" mm");

  Serial.print("Linear requested additional movement: ");
  Serial.print(targetLinearRelativeMM, 3);
  Serial.println(" mm");

  Serial.print("Linear target absolute position: ");
  Serial.print(targetLinearAbsoluteMM, 3);
  Serial.println(" mm");

  Serial.print("Linear Motor Revolutions: ");
  Serial.println(targetLinearRevolutions, 3);

  Serial.print("Linear Estimated STEP Pulses: ");
  Serial.println(targetLinearPulses);

  Serial.print("Rotation Target: ");
  Serial.print(targetUreteroscopeRotationDeg, 2);
  Serial.println(" deg ureteroscope");

  Serial.print("Rotation Map Target: ");
  Serial.print(normalizeRotationMapDeg(currentRotationMapDeg + targetUreteroscopeRotationDeg), 2);
  Serial.println(" deg");

  Serial.print("Rotation Required Motor Angle: ");
  Serial.print(targetRotationMotorDeg, 2);
  Serial.println(" deg motor");

  Serial.print("Rotation Estimated STEP Pulses: ");
  Serial.println(targetRotationPulses);

  Serial.print("Flexion Target: ");
  Serial.print(targetTipDeg, 2);
  Serial.println(" deg tip");

  Serial.print("Flexion Selected Twist Factor: ");
  Serial.println(selectedFlexionTwistFactor, 4);

  Serial.print("Flexion Selected Tip Gain: ");
  Serial.println(selectedFlexionTipGain, 4);

  Serial.print("Flexion Required Motor Angle: ");
  Serial.print(targetFlexionMotorDeg, 2);
  Serial.println(" deg motor");

  Serial.print("Flexion ST3020 Target Step: ");
  Serial.println(targetFlexionSteps);

  Serial.print("Flexion Selected Speed: ");
  Serial.println(selectedFlexionSpeed);
}

void prepareMotorsForMovement() {
  // Set the linear direction only if linear movement is needed
  if (!linearDone) {
    digitalWrite(linear_dirPin, linearMovementSign > 0 ? linear_dirForward : linear_dirBackward);
  }

  // Set the rotation direction only if rotation movement is needed
  if (!rotationDone) {
    digitalWrite(rotation_dirPin, rotationMovementSign > 0 ? rotation_dirClockwise : rotation_dirCounter);
  }

  // Give direction pins a tiny moment to settle before the first pulse
  delayMicroseconds(directionSetup_us);

  // Reset encoders so each command is measured from 0
  resetLinearEncoder();
  resetRotationEncoder();

  // Keep both TB6600 drivers enabled so they can move and hold position
  digitalWrite(linear_enaPin, enaPin_Enable);
  digitalWrite(rotation_enaPin, enaPin_Enable);

  // Start the ST3020 before the stepper pulse loop
  startFlexionMovement();
}

void startFlexionMovement() {
  // The ST3020 receives one command and then moves internally
  // This lets it move while the Teensy is pulsing the stepper drivers

  if (!servoConnected) {
    Serial.println("Flexion Skipped because ST3020 Communication is Unavailable");
    flexionDone = true;
    return;
  }

  int torqueAcknowledged = servo.EnableTorque(servo_ID, 1);

  if (torqueAcknowledged != 1) {
    Serial.println("ST3020 did not Acknowledge Torque Enable");
    flexionDone = true;
    return;
  }

  // Use the target-based speed calculated from the requested absolute tip angle
  // Larger absolute targets above 100 degrees use a smaller and slower speed value
  int commandAcknowledged = servo.WritePosEx(servo_ID, targetFlexionSteps, selectedFlexionSpeed, servo_acceleration);

  if (commandAcknowledged != 1) {
    Serial.println("ST3020 did not Acknowledge the Movement Command");
    flexionDone = true;
    return;
  }

  // Start the no-change timer after the command is accepted
  lastFlexionChangeTime = millis();
}

void runSimultaneousMovementLoop() {
  // Backup pulse limits stop the code from pulsing forever if encoder feedback fails
  // Encoder feedback is still the normal stopping method
  // Allow up to 300 extra pulses before assuming encoder feedback did not reach the target
  // For linear, 300 pulses is about 0.75 mm because 300 * 4 / 1600 = 0.75
  // For rotation, 300 pulses is about 67.5 motor degrees or 27 ureteroscope degrees
  long linearBackupPulseLimit = targetLinearPulses + 300;
  long rotationBackupPulseLimit = targetRotationPulses + 300;

  // Continue until every axis has finished or has been skipped
  while (!linearDone || !rotationDone || !flexionDone) {
    pulseActiveSteppers();

    readLinearFeedback();
    readRotationFeedback();
    readFlexionFeedbackIfReady();

    // Check backup pulse limits inside the loop after feedback has had a chance to update
    if (!linearDone && linearPulsesSent >= linearBackupPulseLimit) {
      Serial.println("Linear stopped because encoder target was not reached");
      linearDone = true;
    }

    if (!rotationDone && rotationPulsesSent >= rotationBackupPulseLimit) {
      Serial.println("Rotation stopped because encoder target was not reached");
      rotationDone = true;
    }

    printMovementStatusIfReady();

    // If only the servo is still moving, pause briefly so the loop does not spin too fast
    if (linearDone && rotationDone && !flexionDone) {
      delay(5);
    }
  }
}

void pulseActiveSteppers() {
  // A stepper is pulsed only if that axis still needs to move
  bool pulseLinear = !linearDone;
  bool pulseRotation = !rotationDone;

  // Set active STEP pins HIGH at the same time
  // This is what makes the two stepper axes move together
  if (pulseLinear) {
    digitalWrite(linear_stepPin, HIGH);
  }

  if (pulseRotation) {
    digitalWrite(rotation_stepPin, HIGH);
  }

  // Hold STEP HIGH long enough for the TB6600 to detect the pulse
  if (pulseLinear || pulseRotation) {
    delayMicroseconds(stepDelay_us);
  }

  // Set active STEP pins LOW at the same time
  if (pulseLinear) {
    digitalWrite(linear_stepPin, LOW);
    linearPulsesSent++;
  }

  if (pulseRotation) {
    digitalWrite(rotation_stepPin, LOW);
    rotationPulsesSent++;
  }

  // Hold STEP LOW before the next pulse
  if (pulseLinear || pulseRotation) {
    delayMicroseconds(stepDelay_us);
  }
}

void readLinearFeedback() {
  // Do not read or update a completed linear axis
  if (linearDone) {
    return;
  }

  // Read raw encoder count from the interrupt-updated variable
  long rawLinearCount = getLinearEncoderCount();

  // Apply sign correction so positive command can print as positive motion
  signedLinearCount = linearEncoderSign * rawLinearCount;

  // Convert encoder counts into measured motor angle
  measuredLinearMotorDeg = (signedLinearCount * degreesPerRevolution) / linearEncoderCPR;

  // Convert encoder counts into measured ureteroscope linear motion
  measuredLinearUreteroMM = (signedLinearCount * linearMMPerRev) / linearEncoderCPR;

  // Stop linear axis when measured travel reaches requested travel
  if (fabsf(measuredLinearUreteroMM) >= targetLinearMagnitudeMM - linearStopMarginMM) {
    linearDone = true;
  }
}

void readRotationFeedback() {
  // Do not read or update a completed rotation axis
  if (rotationDone) {
    return;
  }

  // Read raw encoder count from the interrupt-updated variable
  long rawRotationCount = getRotationEncoderCount();

  // Apply sign correction so positive command can print as positive motion
  signedRotationCount = rotationEncoderSign * rawRotationCount;

  // Convert encoder counts into measured motor angle
  measuredRotationMotorDeg = (signedRotationCount * degreesPerRevolution) / rotationEncoderCPR;

  // Convert measured motor angle into measured ureteroscope rotation
  measuredUreteroRotationDeg = measuredRotationMotorDeg / rotationGearRatio;

  // Stop rotation axis when measured motor angle reaches required motor angle
  if (fabsf(measuredRotationMotorDeg) >= targetRotationMotorMagnitudeDeg - rotationStopMarginMotorDeg) {
    rotationDone = true;
  }
}

void readFlexionFeedbackIfReady() {
  // Do not read or update a completed flexion axis
  if (flexionDone) {
    return;
  }

  // Read the ST3020 only at the print interval so the serial bus is not flooded
  if (millis() - lastFlexionReadTime < printInterval_ms) {
    return;
  }

  // Read the current ST3020 encoder position
  currentFlexionStep = servo.ReadPos(servo_ID);

  if (currentFlexionStep == -1) {
    Serial.println("ST3020 Position Read Failed");
    flexionDone = true;
    return;
  }

  // Track the last time the flexion encoder actually changed
  if (lastMovingFlexionStep == -1 || currentFlexionStep != lastMovingFlexionStep) {
    lastMovingFlexionStep = currentFlexionStep;
    lastFlexionChangeTime = millis();
  }

  // Convert current encoder position into signed motor angle
  measuredFlexionMotorDeg =
    (currentFlexionStep * degreesPerRevolution / (float)servoMaximumStep) - halfRevolutionDegrees;

  // Convert motor angle using the gain selected for this command's target range
  measuredTipDeg = measuredFlexionMotorDeg * selectedFlexionTipGain;

  // Stop flexion axis when servo reaches target step within the margin
  if (abs(currentFlexionStep - targetFlexionSteps) <= flexionStopMarginSteps) {
    flexionDone = true;
  }

  // If the servo has stopped changing position, assume it cannot reach the exact target
  // This is usually caused by a mechanical limit, load, or a target that is too close to the physical end range
  if (!flexionDone && millis() - lastFlexionChangeTime > flexionNoChangeTimeout_ms) {
    Serial.println("Flexion stopped moving before reaching the exact target");
    Serial.println("Ending movement so the Serial Monitor does not print forever");
    flexionDone = true;
  }

  lastFlexionReadTime = millis();
}

void printMovementStatusIfReady() {
  // Printing one combined line keeps the Serial Monitor readable during simultaneous motion
  // Printing takes time, so very frequent printing can slow stepper pulse timing
  if (millis() - lastPrintTime < printInterval_ms) {
    return;
  }

  Serial.print("Linear Abs mm, Rel mm, Count: ");
  Serial.print(currentLinearPositionMM + measuredLinearUreteroMM, 3);
  Serial.print(", ");
  Serial.print(measuredLinearUreteroMM, 3);
  Serial.print(", ");
  Serial.print(signedLinearCount);

  Serial.print(" | Rotation Motor Deg, Ureteroscope Deg, Count: ");
  Serial.print(measuredRotationMotorDeg, 2);
  Serial.print(", ");
  Serial.print(measuredUreteroRotationDeg, 2);
  Serial.print(", ");
  Serial.print(signedRotationCount);

  Serial.print(" | Rotation Map Deg: ");
  Serial.print(normalizeRotationMapDeg(currentRotationMapDeg + measuredUreteroRotationDeg), 2);

  Serial.print(" | Flexion Step, Motor Deg, Tip Deg: ");
  Serial.print(currentFlexionStep);
  Serial.print(", ");
  Serial.print(measuredFlexionMotorDeg, 2);
  Serial.print(", ");
  Serial.println(measuredTipDeg, 2);

  lastPrintTime = millis();
}

void updateAbsoluteLinearPosition() {
  // Absolute position is updated after movement using the measured encoder-based linear travel
  // This means the tracker follows measured motion instead of assuming the motor completed perfectly

  float previousLinearPositionMM = currentLinearPositionMM;
  currentLinearPositionMM = currentLinearPositionMM + measuredLinearUreteroMM;

  // Clean up tiny roundoff near the lower limit
  if (currentLinearPositionMM < linearMinPositionMM && currentLinearPositionMM > linearMinPositionMM - linearRangeToleranceMM) {
    currentLinearPositionMM = linearMinPositionMM;
  }

  // Clean up tiny roundoff near the upper limit
  if (currentLinearPositionMM > linearMaxPositionMM && currentLinearPositionMM < linearMaxPositionMM + linearRangeToleranceMM) {
    currentLinearPositionMM = linearMaxPositionMM;
  }

  Serial.print("Linear absolute position updated from ");
  Serial.print(previousLinearPositionMM, 3);
  Serial.print(" mm to ");
  Serial.print(currentLinearPositionMM, 3);
  Serial.println(" mm");
}

void updateAbsoluteRotationPosition() {
  // Rotation map position is updated after movement using measured encoder-based ureteroscope rotation
  // Startup flat position is 0 degrees
  // Full revolutions wrap back to 0 degrees

  float previousRotationMapDeg = currentRotationMapDeg;
  currentRotationMapDeg = normalizeRotationMapDeg(currentRotationMapDeg + measuredUreteroRotationDeg);

  Serial.print("Rotation map updated from ");
  Serial.print(previousRotationMapDeg, 2);
  Serial.print(" deg to ");
  Serial.print(currentRotationMapDeg, 2);
  Serial.println(" deg");
}

void printFinalSummary() {
  Serial.println("\n3-DOF Movement Complete");

  Serial.print("Final Linear Absolute Position: ");
  Serial.print(currentLinearPositionMM, 3);
  Serial.println(" mm");

  Serial.print("Final Linear Relative Motion, Motor Deg, Count: ");
  Serial.print(measuredLinearUreteroMM, 3);
  Serial.print(", ");
  Serial.print(measuredLinearMotorDeg, 2);
  Serial.print(", ");
  Serial.println(signedLinearCount);

  Serial.print("Final Rotation Motor Deg, Ureteroscope Deg, Count: ");
  Serial.print(measuredRotationMotorDeg, 2);
  Serial.print(", ");
  Serial.print(measuredUreteroRotationDeg, 2);
  Serial.print(", ");
  Serial.println(signedRotationCount);

  Serial.print("Final Rotation Map Position: ");
  Serial.print(currentRotationMapDeg, 2);
  Serial.println(" deg");

  Serial.print("Final Flexion Step, Motor Deg, Tip Deg: ");
  Serial.print(currentFlexionStep);
  Serial.print(", ");
  Serial.print(measuredFlexionMotorDeg, 2);
  Serial.print(", ");
  Serial.println(measuredTipDeg, 2);
}

float normalizeRotationMapDeg(float angleDeg) {
  // Keep the rotation map inside one revolution in either direction
  // +360 degrees means the ureteroscope has returned to flat 0 from clockwise motion
  // -360 degrees means the ureteroscope has returned to flat 0 from counterclockwise motion

  while (angleDeg >= degreesPerRevolution) {
    angleDeg = angleDeg - degreesPerRevolution;
  }

  while (angleDeg <= -degreesPerRevolution) {
    angleDeg = angleDeg + degreesPerRevolution;
  }

  // Clean tiny encoder or floating-point error near 0
  if (fabsf(angleDeg) <= rotationMapZeroToleranceDeg) {
    angleDeg = 0.0;
  }

  return angleDeg;
}

///------------------------------------------------ Disable Motors ------------------------------------------------

void disableAllMotors() {
  // Disable both stepper drivers and force both STEP outputs LOW
  digitalWrite(linear_enaPin, enaPin_Disable);
  digitalWrite(rotation_enaPin, enaPin_Disable);
  digitalWrite(linear_stepPin, LOW);
  digitalWrite(rotation_stepPin, LOW);

  // Disable ST3020 torque if the servo was detected
  if (servoConnected) {
    servo.EnableTorque(servo_ID, 0);
  }
}
