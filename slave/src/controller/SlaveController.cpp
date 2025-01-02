#include "SlaveController.h"

#include "../communication/Protocol.h"
#include "../config/Config.h"
#include "../motion/MotionController.h"
#include "../state/StateMachine.h"

SlaveController::SlaveController()
    : motionController(stateMachine),
      currentPatternIndex(0),
      patternInProgress(false),
      continuousMovementActive(false),
      continuousMovementIsX(false),
      continuousMovementPositive(false),
      originalXSpeed(0),
      originalYSpeed(0),
      suctionEnabled(false),
      armExtended(false) {}

SlaveController::~SlaveController() = default;

void SlaveController::setup() {
  setupEndstops();
  setupSteppers();
  setupPneumatics();
  setupCommunication();
  setupStateHandlers();
  setupCommandHandlers();
}

void SlaveController::setupEndstops() {
  xEndstop.attach(PinConfig::xEndstop, INPUT_PULLUP);
  yEndstop.attach(PinConfig::yEndstop, INPUT_PULLUP);
  xEndstop.interval(TimingConfig::endstopDebounce);
  yEndstop.interval(TimingConfig::endstopDebounce);
}

void SlaveController::setupSteppers() { motionController.setup(); }

void SlaveController::setupPneumatics() {
  pinMode(PinConfig::extension, OUTPUT);
  pinMode(PinConfig::suction, OUTPUT);
  retractArm();
  disableSuction();
}

void SlaveController::setupCommunication() {
  Serial.begin(115200);
  Serial.println(F("Pick and Place Controller"));
  Protocol::debug("Communication initialized");
}

void SlaveController::setupStateHandlers() {
  // Register state handlers with the state machine
  stateMachine.registerHandler(
      State::HOME_REQUESTED,
      [](void* context) {
        auto self = static_cast<SlaveController*>(context);
        self->motionController.startHomingX();
        self->stateMachine.setState(State::HOMING_X);
      },
      this);

  stateMachine.registerHandler(
      State::HOMING_X,
      [](void* context) {
        auto self = static_cast<SlaveController*>(context);
        if (self->xEndstop.read()) {
          Protocol::debug("X endstop triggered during homing");
          self->motionController.stopX();
          self->motionController.setXHome();
          self->motionController.startHomingY();
          self->stateMachine.setState(State::HOMING_Y);
        }
      },
      this);

  stateMachine.registerHandler(
      State::HOMING_Y,
      [](void* context) {
        auto self = static_cast<SlaveController*>(context);
        if (self->yEndstop.rose() || self->yEndstop.read()) {
          self->motionController.stopY();
          self->motionController.setYHome();
          self->stateMachine.setState(State::IDLE);
          Protocol::info("Homing complete");
        }
      },
      this);
}

void SlaveController::setupCommandHandlers() {
  Protocol::debug("Setting up command handlers");

  // Home command
  commandHandler.registerCommand("home", [this](const JsonVariant& params) {
    Protocol::debug("Home command received");
    Protocol::info("Starting homing sequence...");
    patternInProgress = false;
    stateMachine.setState(State::HOME_REQUESTED);
    Protocol::sendResponse({"ok", "Homing sequence started"});
  });

  // Goto command
  commandHandler.registerCommand("goto", [this](const JsonVariant& params) {
    if (!params.containsKey("x") || !params.containsKey("y")) {
      Protocol::error("Missing x or y coordinates");
      return;
    }

    double x = params["x"].as<double>();
    double y = params["y"].as<double>();

    Point target(x, y);
    motionController.moveTo(target);
    Protocol::sendResponse({"ok", "Moving to target position"});
  });

  // Speed command
  commandHandler.registerCommand("setSpeed", [this](const JsonVariant& params) {
    if (!params.containsKey("speed")) {
      Protocol::error("Missing speed value");
      return;
    }

    double speed = params["speed"].as<double>();
    if (speed > 0 && speed <= 100.0) {  // Keep limit at 100 inches/sec
      auto& config = MachineConfig::getInstance();
      config.motion.setSpeed(speed);  // Store in inches/sec
      motionController.setup();       // MotionController will convert to steps
      Protocol::sendResponse({"ok", "Speed updated to " + String(speed)});
    } else {
      Protocol::error("Invalid speed. Use value between 1-100 inches/sec");
    }
  });

  // Acceleration command
  commandHandler.registerCommand("setAccel", [this](const JsonVariant& params) {
    if (!params.containsKey("accel")) {
      Protocol::error("Missing acceleration value");
      return;
    }

    double accel = params["accel"].as<double>();
    if (accel > 0 && accel <= 100.0) {  // Keep limit at 100 inches/sec²
      auto& config = MachineConfig::getInstance();
      config.motion.setAcceleration(accel);  // Store in inches/sec²
      motionController.setup();  // MotionController will convert to steps
      Protocol::sendResponse(
          {"ok", "Acceleration updated to " + String(accel)});
    } else {
      Protocol::error(
          "Invalid acceleration. Use value between 1-100 inches/sec²");
    }
  });

  // Pneumatic commands
  commandHandler.registerCommand("extend", [this](const String&) {
    extendArm();
    Protocol::sendResponse({"ok", "Arm extended"});
  });

  commandHandler.registerCommand("retract", [this](const String&) {
    retractArm();
    Protocol::sendResponse({"ok", "Arm retracted"});
  });

  // Unified suction command
  commandHandler.registerCommand("suction", [this](const JsonVariant& params) {
    if (!params.containsKey("state")) {
      Protocol::error("Missing state parameter for suction");
      return;
    }

    // Handle both boolean and string parameters
    if (params["state"].is<bool>()) {
      bool state = params["state"].as<bool>();
      if (state) {
        enableSuction();
        Protocol::sendResponse({"ok", "Suction enabled"});
      } else {
        disableSuction();
        Protocol::sendResponse({"ok", "Suction disabled"});
      }
    } else {
      const char* state = params["state"];
      if (strcmp(state, "on") == 0) {
        enableSuction();
        Protocol::sendResponse({"ok", "Suction enabled"});
      } else if (strcmp(state, "off") == 0) {
        disableSuction();
        Protocol::sendResponse({"ok", "Suction disabled"});
      } else {
        Protocol::error("Invalid suction state. Use true/false or 'on'/'off'");
      }
    }
  });

  // Emergency stop command
  commandHandler.registerCommand("stop", [this](const String&) {
    emergencyStop();
    Protocol::sendResponse({"ok", "Emergency stop executed"});
  });

  // Start command
  commandHandler.registerCommand("start", [this](const JsonVariant& params) {
    if (!motionController.isHomed()) {
      Protocol::error("Must home axes first. Send 'home' to home.", 1);
      return;
    }

    // Validate required parameters
    if (!params.containsKey("rows") || !params.containsKey("cols")) {
      Protocol::error("Missing required parameters: rows, cols");
      return;
    }

    int rows = params["rows"].as<int>();
    int cols = params["cols"].as<int>();
    double startX = params["startX"] | 0.0;
    double startY = params["startY"] | 0.0;
    double gridWidth = params["gridWidth"] | 26.0;
    double gridLength = params["gridLength"] | 35.0;
    double pickupX = params["pickupX"] | 0.0;
    double pickupY = params["pickupY"] | 0.0;

    if (rows <= 0 || cols <= 0) {
      Protocol::error("Rows and columns must be positive integers");
      return;
    }

    Protocol::debug("Generating pattern with " + String(rows) + " rows, " +
                    String(cols) + " columns at (" + String(startX) + ", " +
                    String(startY) + ") with grid size " + String(gridWidth) +
                    "x" + String(gridLength) + " inches");

    currentPattern = patternGenerator.generatePattern(
        rows, cols, startX, startY, gridWidth, gridLength);

    if (currentPattern.empty()) {
      Protocol::error("Pattern too large for grid", 2);
      return;
    }

    // Log the generated pattern points
    String patternDebug = "Generated pattern points:";
    for (size_t i = 0; i < currentPattern.size(); i++) {
      patternDebug += String("\nPoint ") + String(i) + ": (" +
                      String(currentPattern[i].x) + ", " +
                      String(currentPattern[i].y) + ")";
    }
    Protocol::debug(patternDebug);

    currentPatternIndex = 0;
    patternInProgress = true;
    currentPickupLocation = Point(pickupX, pickupY);
    stateMachine.setState(State::EXECUTING_PATTERN);

    Protocol::sendResponse({"ok", "Pattern started"});
  });
}

void SlaveController::emergencyStop() {
  motionController.stop();
  patternInProgress = false;
  currentPatternIndex = 0;
  disableSuction();
  retractArm();
  stateMachine.setState(State::IDLE);
  Protocol::info("Emergency stop executed");
}

void SlaveController::loop() {
  updateInputs();
  checkForCommands();
  stateMachine.update();
  motionController.update();
  updateState();
}

void SlaveController::updateInputs() {
  xEndstop.update();
  yEndstop.update();

  static bool lastXEndstop = false;
  static bool lastYEndstop = false;

  bool currentXEndstop = xEndstop.read();
  bool currentYEndstop = yEndstop.read();

  if (currentXEndstop != lastXEndstop) {
    Protocol::debug("X endstop changed: " +
                    String(currentXEndstop ? "TRIGGERED" : "RELEASED"));
    lastXEndstop = currentXEndstop;
  }

  if (currentYEndstop != lastYEndstop) {
    Protocol::debug("Y endstop changed: " +
                    String(currentYEndstop ? "TRIGGERED" : "RELEASED"));
    lastYEndstop = currentYEndstop;
  }
}

void SlaveController::checkForCommands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Parse JSON command
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, input);

    if (error) {
      Protocol::error("Failed to parse command JSON", 1);
      return;
    }

    const char* type = doc["type"];
    if (!type) {
      Protocol::error("Missing command type", 2);
      return;
    }

    try {
      // Debug print parsed command
      Protocol::debug("Executing command: " + String(type));

      // Handle command with JSON parameters
      commandHandler.handleCommand(String(type), doc["params"]);
    } catch (const std::exception& e) {
      Protocol::error("Command execution failed: " + String(e.what()), 3);
    } catch (...) {
      Protocol::error("Unknown error during command execution", 4);
    }
  }
}

// Pneumatic control methods
void SlaveController::extendArm() {
  digitalWrite(PinConfig::extension, LOW);
  armExtended = true;

  // Update state
  MachineState newState;
  newState.sensors.armExtended = true;
  newState.sensorsChanged = true;
  Protocol::sendState(newState);
}

void SlaveController::retractArm() {
  digitalWrite(PinConfig::extension, HIGH);
  armExtended = false;

  // Update state
  MachineState newState;
  newState.sensors.armExtended = false;
  newState.sensorsChanged = true;
  Protocol::sendState(newState);
}

void SlaveController::enableSuction() {
  digitalWrite(PinConfig::suction, LOW);
  suctionEnabled = true;

  // Update state
  MachineState newState;
  newState.sensors.suctionEnabled = true;
  newState.sensorsChanged = true;
  Protocol::sendState(newState);
}

void SlaveController::disableSuction() {
  digitalWrite(PinConfig::suction, HIGH);
  suctionEnabled = false;

  // Update state
  MachineState newState;
  newState.sensors.suctionEnabled = false;
  newState.sensorsChanged = true;
  Protocol::sendState(newState);
}

void SlaveController::handleManualMove(const String& command) {
  // Implementation for manual movement
}

void SlaveController::startContinuousMovement(bool isXAxis, bool isPositive,
                                              float speed, float acceleration) {
  // Implementation for continuous movement
}

void SlaveController::stopManualMovement() {
  // Implementation for stopping manual movement
}

void SlaveController::updateState() {
  const unsigned long now = millis();
  MachineState newState;
  bool stateChanged = false;

  // Check state changes every second
  static unsigned long lastStateCheck = 0;
  if (now - lastStateCheck >= 1000) {
    String currentStateStr =
        stateMachine.stateToString(stateMachine.getCurrentState());

    // Only update if state actually changed
    if (currentStateStr != currentState.status) {
      newState.status = currentStateStr;
      newState.statusChanged = true;
      stateChanged = true;

      // Debug state changes
      Protocol::debug("State changed to: " + currentStateStr);
    }
    lastStateCheck = now;
  }

  // Check position changes during movement
  static unsigned long lastPositionCheck = 0;
  if (motionController.isMoving() &&
      now - lastPositionCheck >=
          50) {  // Increased from 100ms to 50ms for smoother updates
    float newX = motionController.getCurrentX();
    float newY = motionController.getCurrentY();

    // Only update if position changed significantly (0.01" threshold)
    if (abs(newX - currentState.position.x) > 0.01 ||
        abs(newY - currentState.position.y) > 0.01) {
      newState.position.x = newX;
      newState.position.y = newY;
      newState.positionChanged = true;
      stateChanged = true;
    }
    lastPositionCheck = now;
  }

  // Check endstop changes
  bool newXEndstop = xEndstop.read();
  bool newYEndstop = yEndstop.read();

  if (newXEndstop != currentState.sensors.xEndstop ||
      newYEndstop != currentState.sensors.yEndstop) {
    newState.sensors.xEndstop = newXEndstop;
    newState.sensors.yEndstop = newYEndstop;
    newState.sensorsChanged = true;
    stateChanged = true;
  }

  // Check pneumatic states
  if (suctionEnabled != currentState.sensors.suctionEnabled ||
      armExtended != currentState.sensors.armExtended) {
    newState.sensors.suctionEnabled = suctionEnabled;
    newState.sensors.armExtended = armExtended;
    newState.sensorsChanged = true;
    stateChanged = true;
  }

  // Only send updates if something actually changed
  if (stateChanged) {
    Protocol::sendState(newState);

    // Update current state with new values
    if (newState.statusChanged) {
      currentState.status = newState.status;
    }
    if (newState.positionChanged) {
      currentState.position = newState.position;
    }
    if (newState.sensorsChanged) {
      currentState.sensors = newState.sensors;
    }
  }
}

void SlaveController::handleJsonCommand() {
  String input = Serial.readStringUntil('\n');
  input.trim();

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, input);

  if (error) {
    Protocol::error("Failed to parse command JSON", 1);
    return;
  }

  const char* type = doc["type"];
  if (!type) {
    Protocol::error("Missing command type", 2);
    return;
  }

  commandHandler.handleCommand(String(type), doc["params"]);
}