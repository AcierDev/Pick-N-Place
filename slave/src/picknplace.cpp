/**
 * Pick and Place Machine Controller
 *
 * Controls a 2-axis pick and place machine with pneumatic end effector.
 * Features homing sequence, configurable movements, and serial control.
 * Includes return-to-origin sequence after initial placement.
 */

#include <AccelStepper.h>
#include <Bounce2.h>

#include "PatternGenerator.h"

//=====================================
// Configuration
//=====================================

// Conversion Configuration
struct ConversionConfig {
  static constexpr double STEPS_PER_INCH =
      125.0;  // 1000 steps / 8 inches = 125 steps per inch

  static long inchesToSteps(double inches) {
    return lround(inches * STEPS_PER_INCH);
  }

  static double stepsToInches(long steps) {
    return static_cast<double>(steps) / STEPS_PER_INCH;
  }
};

// Create configuration instance
const ConversionConfig CONVERSION;

// Motion Parameters
struct MotionConfig {
  // Default values
  double speedInches = 40.0;  // Default operating speed
  double accelerationInches = 20.0;
  const double homingSpeedInches = 7.5;  // Homing speed
  const double pickDistanceInches = 5.0;

  // Converted values
  int32_t getSpeed() const { return CONVERSION.inchesToSteps(speedInches); }
  int32_t getAcceleration() const {
    return CONVERSION.inchesToSteps(accelerationInches);
  }
  int32_t getHomingSpeed() const {
    return CONVERSION.inchesToSteps(homingSpeedInches);
  }
  int32_t getPickDistance() const {
    return CONVERSION.inchesToSteps(pickDistanceInches);
  }
};

// Pin Assignments
struct PinConfig {
  // X-Axis
  const uint8_t xPulse = 6;
  const uint8_t xDirection = 7;
  const uint8_t xEndstop = 13;

  // Y-Axis
  const uint8_t yPulse = 11;
  const uint8_t yDirection = 12;
  const uint8_t yEndstop = 10;

  // Pneumatics
  const uint8_t extension = 3;  // Cylinder extension solenoid
  const uint8_t suction = 4;    // Vacuum suction solenoid
};

// Timing Parameters (in milliseconds)
struct TimingConfig {
  const uint16_t endstopDebounce = 5;   // Debounce time for endstop switches
  const uint16_t pickDelay = 10;        // Wait before starting pick sequence
  const uint16_t pickDuration = 300;    // Time to hold position during pick
  const uint16_t placementDelay = 100;  // Wait before starting place sequence
  const uint16_t placementDuration =
      100;                            // Time to hold position during placement
  const uint16_t retractDelay = 100;  // Wait time after retracting
};

// Additional timing constants
constexpr unsigned long WAIT_RETRIEVE_DELAY = 200;  // milliseconds

// Create configuration instances
MotionConfig MOTION;
const PinConfig PINS;
const TimingConfig TIMING;

//=====================================
// System State Definition
//=====================================

enum class State {
  IDLE,            // New initial state
  HOME_REQUESTED,  // New state for when homing is commanded
  HOMING_X,
  HOMING_Y,
  AWAITING_START,
  MOVING_TO_PICK,
  MOVING_TO_TARGET,  // Generic movement state
  PICKING,
  PLACING,
  RETRACTING,  // Generic retraction state
  WAITING_TO_RETRIEVE,
  EXECUTING_PATTERN
};

//=====================================
// Function Prototypes
//=====================================

// Setup functions
void setupEndstops();
void setupSteppers();
void setupPneumatics();
void setupCommunication();

// Motion control
void homeXAxis();
void homeYAxis();
/**
 * @brief Moves both X and Y axes to the specified position
 * @param position Target position in steps
 * @return true if both axes have reached their target position
 */
bool moveToXYPosition(const long xPos, const long yPos);

// State machine control
void updateInputs();
void runStateMachine();
/**
 * @brief Transitions to the next state and resets the state timer
 * @param nextState The state to transition to
 */
void startNextState(const State nextState);
void waitForStartCommand();

// Sequence control
bool executePickSequence();
bool executePlaceSequence();

// Pneumatic control
void extendArm();
void retractArm();
void enableSuction();
void disableSuction();

// Utility functions
bool hasTimeElapsed(const unsigned long duration);

// Add new function prototypes in the Function Prototypes section
bool moveToXYPosition(const long xPos, const long yPos);
void parseCommand();
void handleCommand(const char cmd, const String& params);
void printCurrentSettings();

//=====================================
// Global Variables
//=====================================

// Stepper motor objects (DRIVER = 1 for step/dir interface)
AccelStepper stepperX(1, PINS.xPulse, PINS.xDirection);
AccelStepper stepperY(1, PINS.yPulse, PINS.yDirection);

// Endstop switch objects
Bounce xEndstop = Bounce();
Bounce yEndstop = Bounce();

// State tracking
State currentState = State::IDLE;
unsigned long stateStartTime = 0;

// Add new global variables in the Global Variables section
long targetX = 0;
long targetY = 0;

// Add to global variables section
PatternGenerator patternGenerator;
std::vector<Point> currentPattern;
size_t currentPatternIndex = 0;
bool patternInProgress = false;

// Global variables for speed and acceleration

// Add these near other global variables
State nextStateAfterMove = State::IDLE;
State nextStateAfterRetract = State::IDLE;

//=====================================
// Setup & Main Loop
//=====================================

void setup() {
  setupEndstops();
  setupSteppers();
  setupPneumatics();
  setupCommunication();
}

void loop() {
  updateInputs();
  runStateMachine();
}

//=====================================
// Setup Helper Functions
//=====================================

void setupEndstops() {
  xEndstop.attach(PINS.xEndstop, INPUT_PULLUP);
  yEndstop.attach(PINS.yEndstop, INPUT_PULLUP);
  xEndstop.interval(TIMING.endstopDebounce);
  yEndstop.interval(TIMING.endstopDebounce);
}

void setupSteppers() {
  // X-Axis configuration
  stepperX.setMaxSpeed(MOTION.getSpeed());
  stepperX.setAcceleration(MOTION.getAcceleration());

  // Y-Axis configuration
  stepperY.setMaxSpeed(MOTION.getSpeed());
  stepperY.setAcceleration(MOTION.getAcceleration());
}

void setupPneumatics() {
  pinMode(PINS.extension, OUTPUT);
  pinMode(PINS.suction, OUTPUT);
  // Initialize in retracted position with suction off
  digitalWrite(PINS.extension, HIGH);
  digitalWrite(PINS.suction, HIGH);
}

void setupCommunication() {
  Serial.begin(115200);
  Serial.println(F("Pick and Place Controller"));
  Serial.println(F("Available commands:"));
  Serial.println(F("h - Home axes (press enter after)"));
  Serial.println(F("s - Start cycle (press enter after)"));
  Serial.println(F("p X Y - Move to position (e.g., p 1000 2000)"));
  Serial.println(F("v SPEED - Set speed (inches/sec, e.g., v 5)"));
  Serial.println(F("a ACCEL - Set acceleration (inches/sec², e.g., a 10)"));
  Serial.println(F("x 0|1 - Retract/Extend arm"));
  Serial.println(F("u 0|1 - Disable/Enable suction"));
  Serial.println(F("k - Execute pick sequence"));
  Serial.println(F("l - Execute place sequence"));
  Serial.println(F("? - Print current settings"));
}

//=====================================
// Main Loop Helper Functions
//=====================================

void updateInputs() {
  xEndstop.update();
  yEndstop.update();
}

void runStateMachine() {
  switch (currentState) {
    case State::IDLE:
    case State::AWAITING_START:
      waitForStartCommand();
      break;

    case State::HOME_REQUESTED:
      startNextState(State::HOMING_X);
      break;

    case State::HOMING_X:
      homeXAxis();
      break;

    case State::HOMING_Y:
      homeYAxis();
      break;

    case State::MOVING_TO_TARGET:
      if (moveToXYPosition(targetX, targetY)) {
        startNextState(nextStateAfterMove);
      }
      break;

    case State::PICKING:
      if (executePickSequence()) {
        Serial.println(F("Pick sequence complete - retracting"));
        startNextState(State::RETRACTING);
      }
      break;

    case State::PLACING:
      if (executePlaceSequence()) {
        Serial.println(F("Place sequence complete - retracting"));
        startNextState(State::RETRACTING);
      }
      break;

    case State::RETRACTING:
      if (hasTimeElapsed(TIMING.retractDelay)) {
        retractArm();
        Serial.println(F("Arm retracted - continuing pattern"));
        startNextState(nextStateAfterRetract);
      }
      break;

    case State::WAITING_TO_RETRIEVE:
      if (hasTimeElapsed(WAIT_RETRIEVE_DELAY)) {
        startNextState(State::EXECUTING_PATTERN);
      }
      break;

    case State::EXECUTING_PATTERN: {
      if (currentPatternIndex >= currentPattern.size()) {
        patternInProgress = false;
        currentState = State::AWAITING_START;
        Serial.println(F("Pattern complete"));
        break;
      }

      static bool needToPickup = true;  // Track if we need to get a new piece

      if (needToPickup) {
        // Move to pickup location (0,0)
        targetX = 0;
        targetY = 0;

        if (moveToXYPosition(targetX, targetY)) {
          Serial.println(
              F("At pickup location (0,0) - starting pick sequence"));
          nextStateAfterRetract = State::EXECUTING_PATTERN;
          startNextState(State::PICKING);
          needToPickup = false;  // We've picked up a piece
        }
      } else {
        // Move to placement location
        Point target = currentPattern[currentPatternIndex];
        targetX = CONVERSION.inchesToSteps(target.x);
        targetY = CONVERSION.inchesToSteps(target.y);

        static bool positionReported = false;
        if (!positionReported) {
          Serial.print(F("Moving to place at point "));
          Serial.print(currentPatternIndex);
          Serial.print(F(" ("));
          Serial.print(target.x);
          Serial.print(F(", "));
          Serial.print(target.y);
          Serial.println(F(") inches"));
          positionReported = true;
        }

        if (moveToXYPosition(targetX, targetY)) {
          Serial.println(F("At placement location - starting place sequence"));
          nextStateAfterRetract = State::EXECUTING_PATTERN;
          startNextState(State::PLACING);
          positionReported = false;
          needToPickup = true;    // Need to get another piece after this
          currentPatternIndex++;  // Move to next placement position
        }
      }
    } break;
  }
}

//=====================================
// Motion Control Functions
//=====================================

void homeXAxis() {
  stepperX.moveTo(-100000);
  stepperX.setMaxSpeed(MOTION.getHomingSpeed());
  stepperX.run();

  if (xEndstop.rose() || xEndstop.read()) {
    stepperX.stop();
    stepperX.setCurrentPosition(0);
    stepperX.setMaxSpeed(MOTION.getSpeed());
    Serial.println(F("X axis homed"));
    currentState = State::HOMING_Y;
  }
}

void homeYAxis() {
  stepperY.moveTo(-100000);
  stepperY.setMaxSpeed(MOTION.getHomingSpeed());
  stepperY.run();

  if (yEndstop.rose() || yEndstop.read()) {
    stepperY.stop();
    stepperY.setCurrentPosition(0);
    stepperY.setMaxSpeed(MOTION.getSpeed());
    Serial.println(F("Y axis homed"));
    currentState = State::AWAITING_START;
    Serial.println(F("Homing complete. Send 's' to start cycle"));
  }
}

/**
 * @brief Moves both X and Y axes to the specified position
 * @param position Target position in steps
 * @return true if both axes have reached their target position
 */
bool moveToXYPosition(const long xPos, const long yPos) {
  // Set target positions
  stepperX.moveTo(xPos);
  stepperY.moveTo(yPos);

  // Run both steppers
  stepperX.run();
  stepperY.run();

  // Check if both have reached their targets
  bool xComplete = stepperX.distanceToGo() == 0;
  bool yComplete = stepperY.distanceToGo() == 0;

  return (xComplete && yComplete);
}

//=====================================
// Pneumatic Control Functions
//=====================================

void extendArm() { digitalWrite(PINS.extension, LOW); }

void retractArm() { digitalWrite(PINS.extension, HIGH); }

void enableSuction() { digitalWrite(PINS.suction, LOW); }

void disableSuction() { digitalWrite(PINS.suction, HIGH); }

//=====================================
// Sequence Control Functions
//=====================================

bool executePickSequence() {
  unsigned long elapsed = millis() - stateStartTime;

  if (elapsed < TIMING.pickDelay) {
    return false;
  } else if (elapsed < TIMING.pickDelay + TIMING.pickDuration) {
    extendArm();
    enableSuction();
    return false;
  }

  return true;
}

bool executePlaceSequence() {
  unsigned long elapsed = millis() - stateStartTime;

  if (elapsed < TIMING.placementDelay) {
    return false;
  } else if (elapsed < TIMING.placementDelay + TIMING.placementDuration) {
    extendArm();
    return false;
  } else {
    disableSuction();
    return true;
  }
}

//=====================================
// Utility Functions
//=====================================

void waitForStartCommand() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    // Wait briefly for any parameters to arrive
    delay(5);
    String params = Serial.readStringUntil('\n');
    params.trim();  // Remove whitespace

    handleCommand(cmd, params);
  }
}

/**
 * @brief Transitions to the next state and resets the state timer
 * @param nextState The state to transition to
 */
void startNextState(const State nextState) {
  currentState = nextState;
  stateStartTime = millis();
}

bool hasTimeElapsed(const unsigned long duration) {
  return (millis() - stateStartTime >= duration);
}

// Add new function to parse position command
void parseCommand() {
  // Wait for all data to arrive
  delay(50);

  String command = Serial.readStringUntil('\n');
  int commaIndex = command.indexOf(',');

  if (commaIndex != -1) {
    targetX = command.substring(0, commaIndex).toInt();
    targetY = command.substring(commaIndex + 1).toInt();

    Serial.print(F("Moving to X:"));
    Serial.print(targetX);
    Serial.print(F(" Y:"));
    Serial.println(targetY);

    startNextState(State::MOVING_TO_TARGET);
  } else {
    Serial.println(F("Invalid command format. Use: p<X>,<Y>"));
  }
}

// Add new command handling functions
void handleCommand(const char cmd, const String& params) {
  switch (cmd) {
    case 'h':
      if (params.length() == 0) {
        Serial.println(F("Starting homing sequence..."));
        startNextState(State::HOME_REQUESTED);
      }
      break;

    case 's':
      if (params.length() == 0) {
        if (stepperX.currentPosition() == 0 &&
            stepperY.currentPosition() == 0) {
          Serial.println(F("Starting cycle..."));
          startNextState(State::MOVING_TO_PICK);
          stepperX.moveTo(MOTION.getPickDistance());
          stepperY.moveTo(MOTION.getPickDistance());
        } else {
          Serial.println(F("Error: Must home axes first. Send 'h' to home."));
        }
      }
      break;

    case 'p': {
      // Find position of first space
      int firstSpace = params.indexOf(' ');
      if (firstSpace != -1) {
        // Get remainder of string after first space
        String yPart = params.substring(firstSpace + 1);
        // Trim any leading spaces from Y part
        yPart.trim();

        // Convert X and Y values from inches to steps
        double xInches = params.substring(0, firstSpace).toFloat();
        double yInches = yPart.toFloat();

        targetX = CONVERSION.inchesToSteps(xInches);
        targetY = CONVERSION.inchesToSteps(yInches);

        Serial.print(F("Moving to X:"));
        Serial.print(xInches);
        Serial.print(F(" Y:"));
        Serial.print(yInches);
        Serial.println(F(" inches"));

        startNextState(State::MOVING_TO_TARGET);
      } else {
        Serial.println(F("Invalid position format. Use: p X Y (in inches)"));
      }
    } break;

    case 'v': {
      if (params.length() > 0) {
        String speedStr = params;
        speedStr.trim();
        double newSpeed = speedStr.toFloat();
        if (newSpeed > 0 && newSpeed <= 100.0) {  // Max 100 inches/sec
          MOTION.speedInches = newSpeed;
          stepperX.setMaxSpeed(MOTION.getSpeed());
          stepperY.setMaxSpeed(MOTION.getSpeed());
          Serial.print(F("Speed set to: "));
          Serial.print(MOTION.speedInches);
          Serial.println(F(" inches/sec"));
        } else {
          Serial.println(F("Invalid speed. Use value between 1-100"));
        }
      }
    } break;

    case 'a': {
      if (params.length() > 0) {
        String accelStr = params;
        accelStr.trim();
        double newAccel = accelStr.toFloat();
        if (newAccel > 0 && newAccel <= 100.0) {  // Max 100 inches/sec²
          MOTION.accelerationInches = newAccel;
          stepperX.setAcceleration(MOTION.getAcceleration());
          stepperY.setAcceleration(MOTION.getAcceleration());
          Serial.print(F("Acceleration set to: "));
          Serial.print(MOTION.accelerationInches);
          Serial.println(F(" inches/sec²"));
        }
      }
    } break;

    case '?':  // Print current settings
      printCurrentSettings();
      break;

    case 'g': {  // Generate new pattern
      int firstSpace = params.indexOf(' ');
      if (firstSpace != -1) {
        String remainder = params.substring(firstSpace + 1);
        remainder.trim();

        // Parse rows and columns directly from the parameters
        int rows = params.substring(0, firstSpace).toInt();
        int cols = remainder.toInt();  // Changed this line

        Serial.print(F("Attempting to generate pattern with rows="));
        Serial.print(rows);
        Serial.print(F(" cols="));
        Serial.println(cols);

        if (rows > 0 && cols > 0) {
          currentPattern = patternGenerator.generatePattern(rows, cols);
          currentPatternIndex = 0;

          if (currentPattern.empty()) {
            Serial.println(F("Error: Pattern too large for tray"));
          } else {
            Serial.print(F("Generated pattern with "));
            Serial.print(currentPattern.size());
            Serial.println(F(" points"));

            // Debug output for pattern points
            for (size_t i = 0; i < currentPattern.size(); i++) {
              Serial.print(F("Point "));
              Serial.print(i);
              Serial.print(F(": ("));
              Serial.print(currentPattern[i].x);
              Serial.print(F(", "));
              Serial.print(currentPattern[i].y);
              Serial.println(F(")"));
            }

            // Start pattern execution if homed
            if (stepperX.currentPosition() == 0 &&
                stepperY.currentPosition() == 0) {
              patternInProgress = true;
              startNextState(State::EXECUTING_PATTERN);
              Serial.println(F("Starting pattern execution"));
            } else {
              Serial.println(
                  F("Error: Must home axes first. Send 'h' to home."));
            }
          }
        } else {
          Serial.println(F("Invalid rows/columns. Must be > 0"));
        }
      } else {
        Serial.println(F("Invalid pattern format. Use: g ROWS COLS"));
      }
    } break;

    case 'x': {  // Manual extension control
      if (params.length() > 0) {
        if (params[0] == '1') {
          extendArm();
          Serial.println(F("Arm extended"));
        } else if (params[0] == '0') {
          retractArm();
          Serial.println(F("Arm retracted"));
        }
      } else {
        Serial.println(F("Invalid extension command. Use: x 0|1"));
      }
    } break;

    case 'u': {  // Manual suction control
      if (params.length() > 0) {
        if (params[0] == '1') {
          enableSuction();
          Serial.println(F("Suction enabled"));
        } else if (params[0] == '0') {
          disableSuction();
          Serial.println(F("Suction disabled"));
        }
      } else {
        Serial.println(F("Invalid suction command. Use: u 0|1"));
      }
    } break;

    case 'k': {  // Manual pick sequence
      if (params.length() == 0) {
        stateStartTime = millis();  // Reset timer for sequence
        Serial.println(F("Starting pick sequence..."));

        // Execute pick sequence
        bool pickComplete = false;
        while (!pickComplete) {
          pickComplete = executePickSequence();
          delay(10);
        }

        // Wait for retract delay
        delay(TIMING.retractDelay);

        // Retract arm while maintaining suction
        retractArm();
        Serial.println(F("Pick sequence complete"));
      }
    } break;

    case 'l': {  // Manual place sequence
      if (params.length() == 0) {
        stateStartTime = millis();  // Reset timer for sequence
        Serial.println(F("Starting place sequence..."));

        // Execute place sequence
        bool placeComplete = false;
        while (!placeComplete) {
          placeComplete = executePlaceSequence();
          delay(10);
        }

        // Wait for retract delay
        delay(TIMING.retractDelay);

        // Retract arm (suction already disabled by place sequence)
        retractArm();
        Serial.println(F("Place sequence complete"));
      }
    } break;

    default:
      Serial.println(F("Unknown command. Available commands:"));
      Serial.println(F("h - Home axes (press enter after)"));
      Serial.println(F("s - Start cycle (press enter after)"));
      Serial.println(F("p X Y - Move to position (e.g., p 1000 2000)"));
      Serial.println(F("v SPEED - Set speed (inches/sec, e.g., v 5)"));
      Serial.println(F("a ACCEL - Set acceleration (inches/sec², e.g., a 10)"));
      Serial.println(F("x 0|1 - Retract/Extend arm"));
      Serial.println(F("u 0|1 - Disable/Enable suction"));
      Serial.println(F("k - Execute pick sequence"));
      Serial.println(F("l - Execute place sequence"));
      Serial.println(F("? - Print current settings"));
      break;
  }
}

void printCurrentSettings() {
  Serial.println(F("Current Settings:"));
  Serial.print(F("Speed: "));
  Serial.print(MOTION.speedInches);
  Serial.println(F(" inches/sec"));
  Serial.print(F("Acceleration: "));
  Serial.print(MOTION.accelerationInches);
  Serial.println(F(" inches/sec²"));
  Serial.print(F("X Position: "));
  Serial.print(CONVERSION.stepsToInches(stepperX.currentPosition()));
  Serial.println(F(" inches"));
  Serial.print(F("Y Position: "));
  Serial.print(CONVERSION.stepsToInches(stepperY.currentPosition()));
  Serial.println(F(" inches"));
}