#include "SlaveController.h"

SlaveController::SlaveController()
    : currentState(State::IDLE),
      nextStateAfterMove(State::IDLE),
      nextStateAfterRetract(State::IDLE),
      stateStartTime(0),
      currentPatternIndex(0),
      patternInProgress(false),
      targetX(0),
      targetY(0) {
  // Initialize steppers
  stepperX = new AccelStepper(1, PinConfig::xPulse, PinConfig::xDirection);
  stepperY = new AccelStepper(1, PinConfig::yPulse, PinConfig::yDirection);
}

SlaveController::~SlaveController() {
  delete stepperX;
  delete stepperY;
}

void SlaveController::setup() {
  setupEndstops();
  setupSteppers();
  setupPneumatics();
  setupCommunication();
}

void SlaveController::setupEndstops() {
  xEndstop.attach(PinConfig::xEndstop, INPUT_PULLUP);
  yEndstop.attach(PinConfig::yEndstop, INPUT_PULLUP);
  xEndstop.interval(TimingConfig::endstopDebounce);
  yEndstop.interval(TimingConfig::endstopDebounce);
}

void SlaveController::setupSteppers() {
  stepperX->setMaxSpeed(motionConfig.getSpeed());
  stepperX->setAcceleration(motionConfig.getAcceleration());
  stepperY->setMaxSpeed(motionConfig.getSpeed());
  stepperY->setAcceleration(motionConfig.getAcceleration());
}

void SlaveController::setupPneumatics() {
  pinMode(PinConfig::extension, OUTPUT);
  pinMode(PinConfig::suction, OUTPUT);
  retractArm();
  disableSuction();
}

void SlaveController::setupCommunication() {
  Serial.begin(115200);
  Serial.println(F("Pick and Place Controller"));
  // Print available commands...
  // [Previous command documentation remains the same]
}

void SlaveController::loop() {
  updateInputs();
  runStateMachine();
}

void SlaveController::updateInputs() {
  xEndstop.update();
  yEndstop.update();
}

void SlaveController::runStateMachine() {
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
      if (hasTimeElapsed(TimingConfig::retractDelay)) {
        retractArm();
        Serial.println(F("Arm retracted - continuing pattern"));
        startNextState(nextStateAfterRetract);
      }
      break;

    case State::WAITING_TO_RETRIEVE:
      if (hasTimeElapsed(TimingConfig::waitRetrieveDelay)) {
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
        targetX = ConversionConfig::inchesToSteps(target.x);
        targetY = ConversionConfig::inchesToSteps(target.y);

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

void SlaveController::homeXAxis() {
  stepperX->moveTo(-100000);
  stepperX->setMaxSpeed(motionConfig.getHomingSpeed());
  stepperX->run();

  if (xEndstop.rose() || xEndstop.read()) {
    stepperX->stop();
    stepperX->setCurrentPosition(0);
    stepperX->setMaxSpeed(motionConfig.getSpeed());
    Serial.println(F("X axis homed"));
    currentState = State::HOMING_Y;
  }
}

void SlaveController::homeYAxis() {
  stepperY->moveTo(-100000);
  stepperY->setMaxSpeed(motionConfig.getHomingSpeed());
  stepperY->run();

  if (yEndstop.rose() || yEndstop.read()) {
    stepperY->stop();
    stepperY->setCurrentPosition(0);
    stepperY->setMaxSpeed(motionConfig.getSpeed());
    Serial.println(F("Y axis homed"));
    currentState = State::AWAITING_START;
    Serial.println(F("Homing complete. Send 's' to start cycle"));
  }
}

bool SlaveController::moveToXYPosition(const long xPos, const long yPos) {
  stepperX->moveTo(xPos);
  stepperY->moveTo(yPos);

  stepperX->run();
  stepperY->run();

  bool xComplete = stepperX->distanceToGo() == 0;
  bool yComplete = stepperY->distanceToGo() == 0;

  return (xComplete && yComplete);
}

void SlaveController::startNextState(const State nextState) {
  currentState = nextState;
  stateStartTime = millis();
}

bool SlaveController::hasTimeElapsed(const unsigned long duration) {
  return (millis() - stateStartTime >= duration);
}

// Pneumatic control methods
void SlaveController::extendArm() { digitalWrite(PinConfig::extension, LOW); }

void SlaveController::retractArm() { digitalWrite(PinConfig::extension, HIGH); }

void SlaveController::enableSuction() { digitalWrite(PinConfig::suction, LOW); }

void SlaveController::disableSuction() {
  digitalWrite(PinConfig::suction, HIGH);
}

// Sequence control methods
bool SlaveController::executePickSequence() {
  unsigned long elapsed = millis() - stateStartTime;

  if (elapsed < TimingConfig::pickDelay) {
    return false;
  } else if (elapsed < TimingConfig::pickDelay + TimingConfig::pickDuration) {
    extendArm();
    enableSuction();
    return false;
  }

  return true;
}

bool SlaveController::executePlaceSequence() {
  unsigned long elapsed = millis() - stateStartTime;

  if (elapsed < TimingConfig::placementDelay) {
    return false;
  } else if (elapsed <
             TimingConfig::placementDelay + TimingConfig::placementDuration) {
    extendArm();
    return false;
  } else {
    disableSuction();
    return true;
  }
}

void SlaveController::waitForStartCommand() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    // Wait briefly for any parameters to arrive
    delay(5);
    String params = Serial.readStringUntil('\n');
    params.trim();  // Remove whitespace

    handleCommand(cmd, params);
  }
}

void SlaveController::handleCommand(const char cmd, const String& params) {
  switch (cmd) {
    case 'h':
      if (params.length() == 0) {
        Serial.println(F("Starting homing sequence..."));
        startNextState(State::HOME_REQUESTED);
      }
      break;

    case 's':
      if (params.length() == 0) {
        if (stepperX->currentPosition() == 0 &&
            stepperY->currentPosition() == 0) {
          Serial.println(F("Starting cycle..."));
          startNextState(State::MOVING_TO_PICK);
          stepperX->moveTo(motionConfig.getPickDistance());
          stepperY->moveTo(motionConfig.getPickDistance());
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
        yPart.trim();

        // Convert X and Y values from inches to steps
        double xInches = params.substring(0, firstSpace).toFloat();
        double yInches = yPart.toFloat();

        targetX = ConversionConfig::inchesToSteps(xInches);
        targetY = ConversionConfig::inchesToSteps(yInches);

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
          motionConfig.speedInches = newSpeed;
          stepperX->setMaxSpeed(motionConfig.getSpeed());
          stepperY->setMaxSpeed(motionConfig.getSpeed());
          Serial.print(F("Speed set to: "));
          Serial.print(motionConfig.speedInches);
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
          motionConfig.accelerationInches = newAccel;
          stepperX->setAcceleration(motionConfig.getAcceleration());
          stepperY->setAcceleration(motionConfig.getAcceleration());
          Serial.print(F("Acceleration set to: "));
          Serial.print(motionConfig.accelerationInches);
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

        int rows = params.substring(0, firstSpace).toInt();
        int cols = remainder.toInt();

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
            if (stepperX->currentPosition() == 0 &&
                stepperY->currentPosition() == 0) {
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

        bool pickComplete = false;
        while (!pickComplete) {
          pickComplete = executePickSequence();
          delay(10);
        }

        delay(TimingConfig::retractDelay);
        retractArm();
        Serial.println(F("Pick sequence complete"));
      }
    } break;

    case 'l': {  // Manual place sequence
      if (params.length() == 0) {
        stateStartTime = millis();  // Reset timer for sequence
        Serial.println(F("Starting place sequence..."));

        bool placeComplete = false;
        while (!placeComplete) {
          placeComplete = executePlaceSequence();
          delay(10);
        }

        delay(TimingConfig::retractDelay);
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

void SlaveController::printCurrentSettings() {
  Serial.println(F("Current Settings:"));
  Serial.print(F("Speed: "));
  Serial.print(motionConfig.speedInches);
  Serial.println(F(" inches/sec"));
  Serial.print(F("Acceleration: "));
  Serial.print(motionConfig.accelerationInches);
  Serial.println(F(" inches/sec²"));
  Serial.print(F("X Position: "));
  Serial.print(ConversionConfig::stepsToInches(stepperX->currentPosition()));
  Serial.println(F(" inches"));
  Serial.print(F("Y Position: "));
  Serial.print(ConversionConfig::stepsToInches(stepperY->currentPosition()));
  Serial.println(F(" inches"));
}