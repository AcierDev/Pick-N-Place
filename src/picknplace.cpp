/**
 * Pick and Place Machine Controller
 *
 * Controls a 2-axis pick and place machine with pneumatic end effector.
 * Features homing sequence, configurable movements, and serial control.
 * Includes return-to-origin sequence after initial placement.
 */

#include <AccelStepper.h>
#include <Bounce2.h>

//=====================================
// Configuration
//=====================================

// Motion Parameters
struct MotionConfig {
  const int32_t homingSpeed = 500;  // Steps per second during homing
  const int32_t operatingSpeed =
      15000;  // Steps per second during normal operation
  const int32_t acceleration = 12000;  // Steps per second per second
  const int32_t pickDistance = -1000;  // Steps from home to pick position
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
const MotionConfig MOTION;
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
  PICKING,
  RETRACTING_WITH_PART,
  MOVING_TO_PLACE,
  PLACING,
  RETRACTING_AFTER_PLACE,
  WAITING_TO_RETRIEVE,
  RETRIEVING,
  RETRACTING_WITH_RETRIEVED,
  MOVING_TO_ORIGIN,
  PLACING_AT_ORIGIN,
  RETRACTING_EMPTY,
  RETURNING_HOME
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
bool moveToPosition(const long position);

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
bool executeRetrieveSequence();
bool executePlaceSequence();

// Pneumatic control
void extendArm();
void retractArm();
void enableSuction();
void disableSuction();

// Utility functions
bool hasTimeElapsed(const unsigned long duration);

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
  stepperX.setMaxSpeed(MOTION.operatingSpeed);
  stepperX.setAcceleration(MOTION.acceleration);

  // Y-Axis configuration
  stepperY.setMaxSpeed(MOTION.operatingSpeed);
  stepperY.setAcceleration(MOTION.acceleration);
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
  Serial.println(F("Send 'h' to home axes"));
  Serial.println(F("Send 's' to start cycle (only after homing)"));
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

    case State::MOVING_TO_PICK:
      if (moveToPosition(MOTION.pickDistance)) {
        startNextState(State::PICKING);
      }
      break;

    case State::PICKING:
      if (executePickSequence()) {
        startNextState(State::RETRACTING_WITH_PART);
      }
      break;

    case State::RETRACTING_WITH_PART:
      if (hasTimeElapsed(TIMING.retractDelay)) {
        retractArm();
        startNextState(State::MOVING_TO_PLACE);
      }
      break;

    case State::MOVING_TO_PLACE:
      if (moveToPosition(0)) {  // Return to home
        startNextState(State::PLACING);
      }
      break;

    case State::PLACING:
      if (executePlaceSequence()) {
        startNextState(State::RETRACTING_AFTER_PLACE);
      }
      break;

    case State::RETRACTING_AFTER_PLACE:
      if (hasTimeElapsed(TIMING.retractDelay)) {
        retractArm();
        startNextState(State::WAITING_TO_RETRIEVE);
      }
      break;

    case State::WAITING_TO_RETRIEVE:
      if (hasTimeElapsed(WAIT_RETRIEVE_DELAY)) {
        startNextState(State::RETRIEVING);
      }
      break;

    case State::RETRIEVING:
      if (executeRetrieveSequence()) {
        startNextState(State::RETRACTING_WITH_RETRIEVED);
      }
      break;

    case State::RETRACTING_WITH_RETRIEVED:
      if (hasTimeElapsed(TIMING.retractDelay)) {
        retractArm();
        startNextState(State::MOVING_TO_ORIGIN);
      }
      break;

    case State::MOVING_TO_ORIGIN:
      if (moveToPosition(
              MOTION.pickDistance)) {  // Return to original pick position
        startNextState(State::PLACING_AT_ORIGIN);
      }
      break;

    case State::PLACING_AT_ORIGIN:
      if (executePlaceSequence()) {
        startNextState(State::RETRACTING_EMPTY);
      }
      break;

    case State::RETRACTING_EMPTY:
      if (hasTimeElapsed(TIMING.retractDelay)) {
        retractArm();
        startNextState(State::RETURNING_HOME);
      }
      break;

    case State::RETURNING_HOME:
      if (moveToPosition(0)) {  // Return to home position
        currentState = State::AWAITING_START;
        Serial.println(F("Cycle complete. Send 's' to start next cycle"));
      }
      break;
  }
}

//=====================================
// Motion Control Functions
//=====================================

void homeXAxis() {
  stepperX.setSpeed(MOTION.homingSpeed);
  stepperX.run();

  if (xEndstop.rose() || xEndstop.read()) {
    stepperX.stop();
    stepperX.setCurrentPosition(0);
    Serial.println(F("X axis homed"));
    currentState = State::HOMING_Y;
  }
}

void homeYAxis() {
  stepperY.setSpeed(MOTION.homingSpeed);
  stepperY.run();

  if (yEndstop.rose() || yEndstop.read()) {
    stepperY.stop();
    stepperY.setCurrentPosition(0);
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
bool moveToPosition(const long position) {
  stepperX.moveTo(position);
  stepperY.moveTo(position);

  bool xComplete = !stepperX.isRunning();
  bool yComplete = !stepperY.isRunning();

  if (!xComplete) stepperX.run();
  if (!yComplete) stepperY.run();

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

bool executeRetrieveSequence() {
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
    char input = Serial.read();
    if (input == 'h') {
      Serial.println(F("Starting homing sequence..."));
      startNextState(State::HOME_REQUESTED);
    } else if (input == 's') {
      // Only allow start if we've homed
      if (stepperX.currentPosition() == 0 && stepperY.currentPosition() == 0) {
        Serial.println(F("Starting cycle..."));
        startNextState(State::MOVING_TO_PICK);
        stepperX.moveTo(MOTION.pickDistance);
        stepperY.moveTo(MOTION.pickDistance);
      } else {
        Serial.println(F("Error: Must home axes first. Send 'h' to home."));
      }
    }
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