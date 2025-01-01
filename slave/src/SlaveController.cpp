#include "SlaveController.h"

SlaveController::SlaveController()
    : currentState(State::IDLE),
      nextStateAfterMove(State::IDLE),
      nextStateAfterRetract(State::IDLE),
      stateStartTime(0),
      currentPatternIndex(0),
      patternInProgress(false),
      targetX(0),
      targetY(0),
      lastStateChangeTime(0),
      stateStartCount(0) {
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
  // Check for commands before running state machine
  checkForCommands();
  runStateMachine();
}

void SlaveController::updateInputs() {
  xEndstop.update();
  yEndstop.update();
}

void SlaveController::runStateMachine() {
  static unsigned long lastDebugOutput = 0;
  const unsigned long DEBUG_INTERVAL = 1000;  // Print debug every second

  unsigned long currentTime = millis();
  if (currentTime - lastDebugOutput >= DEBUG_INTERVAL) {
    Serial.print(F("Current state: "));
    Serial.print(stateToString(currentState));
    Serial.print(F(" Time in state: "));
    Serial.print(currentTime - stateStartTime);
    Serial.println(F("ms"));
    lastDebugOutput = currentTime;
  }

  // Check for state timeout - exclude IDLE and AWAITING_START states
  if (currentTime - stateStartTime > STATE_TIMEOUT_MS &&
      currentState != State::IDLE && currentState != State::AWAITING_START) {
    Serial.print(F("ERROR: State "));
    Serial.print(stateToString(currentState));
    Serial.println(F(" has timed out. Executing emergency stop."));
    emergencyStop();
    return;
  }

  switch (currentState) {
    case State::IDLE:
    case State::AWAITING_START:
      waitForStartCommand();
      break;

    case State::HOME_REQUESTED:
      homeXAxis();
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
        currentPickupLocation = Point(0, 0);  // Reset pickup location
        Serial.println(F("Pattern complete"));
        break;
      }

      static bool needToPickup = true;
      static bool movingToSafeY = false;

      if (needToPickup) {
        targetX = ConversionConfig::inchesToSteps(currentPickupLocation.x);
        targetY = ConversionConfig::inchesToSteps(currentPickupLocation.y);

        if (moveToXYPosition(targetX, targetY)) {
          currentPickupLocation.x += PIECE_SPACING;
          nextStateAfterRetract = State::EXECUTING_PATTERN;
          startNextState(State::PICKING);
          needToPickup = false;
          movingToSafeY = true;
        }
      } else if (movingToSafeY) {
        Point target = currentPattern[currentPatternIndex];
        targetX = stepperX->currentPosition();
        targetY = ConversionConfig::inchesToSteps(SAFE_TRAVEL_Y);

        if (moveToXYPosition(targetX, targetY)) {
          movingToSafeY = false;
        }
      } else {
        Point target = currentPattern[currentPatternIndex];
        targetX = ConversionConfig::inchesToSteps(target.x);
        targetY = ConversionConfig::inchesToSteps(target.y);

        if (moveToXYPosition(targetX, targetY)) {
          nextStateAfterRetract = State::EXECUTING_PATTERN;
          startNextState(State::PLACING);
          needToPickup = true;
          currentPatternIndex++;
        }
      }
    } break;
  }
}

void SlaveController::homeXAxis() {
  // Cancel any ongoing movements and reset state
  stepperX->stop();
  stepperY->stop();

  // Reset current positions to clear any previous targets
  stepperX->setCurrentPosition(stepperX->currentPosition());
  stepperY->setCurrentPosition(stepperY->currentPosition());

  // Reset pattern state
  patternInProgress = false;
  currentPatternIndex = 0;

  // Set homing speed
  stepperX->setMaxSpeed(motionConfig.getHomingSpeed());
  stepperX->setSpeed(
      -motionConfig.getHomingSpeed());  // Set constant speed for homing

  // Move towards home
  if (xEndstop.rose() || xEndstop.read()) {
    stepperX->stop();
    stepperX->setCurrentPosition(0);
    stepperX->setMaxSpeed(motionConfig.getSpeed());
    stepperX->setAcceleration(motionConfig.getAcceleration());
    Serial.println(F("X axis homed"));
    currentState = State::HOMING_Y;
  } else {
    stepperX->runSpeed();  // Use runSpeed() instead of run()
  }
}

void SlaveController::homeYAxis() {
  // Set homing speed
  stepperY->setMaxSpeed(motionConfig.getHomingSpeed());
  stepperY->setSpeed(
      -motionConfig.getHomingSpeed());  // Set constant speed for homing

  // Move towards home
  if (yEndstop.rose() || yEndstop.read()) {
    stepperY->stop();
    stepperY->setCurrentPosition(0);
    stepperY->setMaxSpeed(motionConfig.getSpeed());
    stepperY->setAcceleration(motionConfig.getAcceleration());
    Serial.println(F("Y axis homed"));
    currentState = State::AWAITING_START;
    Serial.println(F("Homing complete. Send 's' to start cycle"));
  } else {
    stepperY->runSpeed();  // Use runSpeed() instead of run()
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
  // Debug output for state change
  Serial.print(F("State change: "));
  Serial.print(stateToString(currentState));
  Serial.print(F(" -> "));
  Serial.println(stateToString(nextState));

  if (currentState == nextState) {
    stateStartCount++;
    if (stateStartCount >
        1000) {  // Arbitrary threshold for repeated state entries
      Serial.print(F("WARNING: State "));
      Serial.print(stateToString(nextState));
      Serial.println(F(" has been re-entered over 1000 times"));
    }
  } else {
    stateStartCount = 0;
  }

  // Track time in previous state
  unsigned long timeInPreviousState = millis() - lastStateChangeTime;
  if (timeInPreviousState > STATE_TIMEOUT_MS) {
    Serial.print(F("WARNING: State "));
    Serial.print(stateToString(currentState));
    Serial.print(F(" took "));
    Serial.print(timeInPreviousState);
    Serial.println(F("ms to complete"));
  }

  currentState = nextState;
  stateStartTime = millis();
  lastStateChangeTime = millis();
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
    String command = Serial.readStringUntil(' ');
    String params = Serial.readStringUntil('\n');
    command.trim();
    params.trim();
    handleCommand(command, params);
  }
}

void SlaveController::handleCommand(const String& command,
                                    const String& params) {
  if (command == "home") {
    Serial.println(F("Starting homing sequence..."));
    // Cancel any ongoing movements
    stepperX->stop();
    stepperY->stop();
    // Reset pattern state
    patternInProgress = false;
    // Start homing sequence
    startNextState(State::HOME_REQUESTED);
  }

  else if (command == "start") {
    if (stepperX->currentPosition() == 0 && stepperY->currentPosition() == 0) {
      // Parse parameters: rows cols [startX startY gridWidth gridLength]
      int rows = 0, cols = 0;
      double startX = 0.0, startY = 0.0;
      double gridWidth = 26.0, gridLength = 35.0;  // Default values

      String params_str = params;
      int firstSpace = params_str.indexOf(' ');
      if (firstSpace != -1) {
        String remainder = params_str.substring(firstSpace + 1);

        // Parse required parameters (rows and cols)
        rows = params_str.substring(0, firstSpace).toInt();
        cols = remainder.toInt();

        // Parse optional parameters if provided
        int nextSpace = remainder.indexOf(' ');
        if (nextSpace != -1) {
          // Parse startX
          String startXStr = remainder.substring(nextSpace + 1);
          int thirdSpace = startXStr.indexOf(' ');
          if (thirdSpace != -1) {
            startX = startXStr.substring(0, thirdSpace).toFloat();

            // Parse startY
            String startYStr = startXStr.substring(thirdSpace + 1);
            int fourthSpace = startYStr.indexOf(' ');
            if (fourthSpace != -1) {
              startY = startYStr.substring(0, fourthSpace).toFloat();

              // Parse gridWidth
              String gridWidthStr = startYStr.substring(fourthSpace + 1);
              int fifthSpace = gridWidthStr.indexOf(' ');
              if (fifthSpace != -1) {
                double providedWidth =
                    gridWidthStr.substring(0, fifthSpace).toFloat();
                double providedLength = gridWidthStr.toFloat();

                // Only use provided values if they're greater than 0
                if (providedWidth > 0) gridWidth = providedWidth;
                if (providedLength > 0) gridLength = providedLength;
              }
            }
          }
        }

        if (rows > 0 && cols > 0) {
          // Generate the pattern with parameters
          currentPattern = patternGenerator.generatePattern(
              rows, cols, startX, startY, gridWidth, gridLength);

          if (currentPattern.empty()) {
            Serial.println(F("Error: Pattern too large for grid"));
            return;
          }

          Serial.print(F("Starting pattern with "));
          Serial.print(rows);
          Serial.print(F(" rows, "));
          Serial.print(cols);
          Serial.print(F(" columns at ("));
          Serial.print(startX);
          Serial.print(F(", "));
          Serial.print(startY);
          Serial.print(F(") with grid size "));
          Serial.print(gridWidth);
          Serial.print(F("x"));
          Serial.print(gridLength);
          Serial.println(F(" inches"));

          currentPatternIndex = 0;
          patternInProgress = true;
          startNextState(State::EXECUTING_PATTERN);
        } else {
          Serial.println(F("Error: Rows and columns must be > 0"));
        }
      }
      if (!patternInProgress) {
        Serial.println(
            F("Error: Invalid parameters. Use: start ROWS COLS [START_X "
              "START_Y GRID_WIDTH GRID_LENGTH]"));
      }
    } else {
      Serial.println(F("Error: Must home axes first. Send 'home' to home."));
    }

    // Reset pickup location when starting new pattern
    currentPickupLocation = Point(0, 0);
  }

  else if (command == "goto") {
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
      Serial.println(F("Invalid position format. Use: goto X Y (in inches)"));
    }
  }

  else if (command == "speed") {
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

  else if (command == "accel") {
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

  else if (command == "extend") {
    extendArm();
    Serial.println(F("Arm extended"));
  }

  else if (command == "retract") {
    retractArm();
    Serial.println(F("Arm retracted"));
  }

  else if (command == "suction_on") {
    enableSuction();
    Serial.println(F("Suction enabled"));
  }

  else if (command == "suction_off") {
    disableSuction();
    Serial.println(F("Suction disabled"));
  }

  else if (command == "stop") {
    emergencyStop();
  }

  else {
    Serial.println(F("Unknown command. Available commands:"));
    Serial.println(F("home           - Home axes"));
    Serial.println(F("start R C      - Start cycle with R rows and C columns"));
    Serial.println(F("goto X Y       - Move to position (inches)"));
    Serial.println(F("speed VALUE    - Set speed (inches/sec, 1-100)"));
    Serial.println(F("accel VALUE    - Set acceleration (inches/sec², 1-100)"));
    Serial.println(F("extend         - Extend arm"));
    Serial.println(F("retract        - Retract arm"));
    Serial.println(F("suction_on     - Enable suction"));
    Serial.println(F("suction_off    - Disable suction"));
    Serial.println(F("stop           - Emergency stop"));
  }

  // Send current state after handling command
  sendCurrentState();
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

void SlaveController::sendCurrentState() {
  StaticJsonDocument<200> doc;
  doc["status"] = stateToString(currentState);

  // Add any sensor states or other relevant data
  JsonObject sensors = doc.createNestedObject("sensors");
  sensors["x_endstop"] = xEndstop.read();
  sensors["y_endstop"] = yEndstop.read();

  Serial.print(F("STATE "));
  serializeJson(doc, Serial);
  Serial.println();
}

const char* SlaveController::stateToString(State state) {
  switch (state) {
    case State::IDLE:
      return "IDLE";
    case State::HOME_REQUESTED:
      return "HOME_REQUESTED";
    case State::HOMING_X:
      return "HOMING_X";
    case State::HOMING_Y:
      return "HOMING_Y";
    case State::AWAITING_START:
      return "AWAITING_START";
    case State::MOVING_TO_PICK:
      return "MOVING_TO_PICK";
    case State::MOVING_TO_TARGET:
      return "MOVING_TO_TARGET";
    case State::PICKING:
      return "PICKING";
    case State::PLACING:
      return "PLACING";
    case State::RETRACTING:
      return "RETRACTING";
    case State::WAITING_TO_RETRIEVE:
      return "WAITING_TO_RETRIEVE";
    case State::EXECUTING_PATTERN:
      return "EXECUTING_PATTERN";
    default:
      return "UNKNOWN";
  }
}

// New method to check for commands
void SlaveController::checkForCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil(' ');
    String params = Serial.readStringUntil('\n');
    command.trim();
    params.trim();

    // Special handling for emergency stop and home commands
    if (command == "stop" || command == "home") {
      if (command == "stop") {
        emergencyStop();
      } else {  // home command
        Serial.println(F("Starting homing sequence..."));
        // Cancel any ongoing movements
        stepperX->stop();
        stepperY->stop();
        // Reset pattern state
        patternInProgress = false;
        // Start homing sequence
        startNextState(State::HOME_REQUESTED);
      }
      return;
    }

    // Other commands are only processed in IDLE or AWAITING_START states
    if (currentState == State::IDLE || currentState == State::AWAITING_START) {
      handleCommand(command, params);
    }
  }
}

void SlaveController::emergencyStop() {
  // First stop the current movements
  stepperX->stop();
  stepperY->stop();

  // Reset all motion state
  stepperX->setCurrentPosition(
      stepperX->currentPosition());  // Clear target position
  stepperY->setCurrentPosition(
      stepperY->currentPosition());  // Clear target position

  // Reset speeds to default
  stepperX->setMaxSpeed(motionConfig.getSpeed());
  stepperY->setMaxSpeed(motionConfig.getSpeed());

  // Reset pattern and state
  currentPatternIndex = 0;
  patternInProgress = false;
  currentState = State::IDLE;

  // Safety: retract and disable suction
  disableSuction();
  retractArm();

  Serial.println(F("Emergency stop executed"));
}