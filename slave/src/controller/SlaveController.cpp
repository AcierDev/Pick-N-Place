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
  Protocol::debug("Setting up endstops...");

  xEndstop.attach(PinConfig::xEndstop, INPUT);
  yEndstop.attach(PinConfig::yEndstop, INPUT);
  xEndstop.interval(TimingConfig::endstopDebounce);
  yEndstop.interval(TimingConfig::endstopDebounce);

  Protocol::debug("Initial X endstop state: " + String(xEndstop.read()));
  Protocol::debug("Initial Y endstop state: " + String(yEndstop.read()));
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
        bool triggered =
            self->xEndstop.read();  // Using pull-down, HIGH means triggered

        if (triggered) {
          self->motionController.stopX();
          self->motionController.setXHome();
          self->motionController.startHomingY();
          self->stateMachine.setState(State::HOMING_Y);

          // Force an immediate state update to reflect the endstop
          MachineState state;
          state.sensors.xEndstop = triggered;
          state.sensorsChanged = true;
          Protocol::sendState(state);
        }
      },
      this);

  stateMachine.registerHandler(
      State::HOMING_Y,
      [](void* context) {
        auto self = static_cast<SlaveController*>(context);
        bool triggered =
            self->yEndstop.read();  // Using pull-down, HIGH means triggered

        if (triggered) {
          self->motionController.stopY();
          self->motionController.setYHome();
          self->stateMachine.setState(State::IDLE);
          Protocol::info("Homing complete");

          // Force an immediate state update to reflect the endstop
          MachineState state;
          state.sensors.yEndstop = triggered;
          state.sensorsChanged = true;
          Protocol::sendState(state);
        }
      },
      this);
}

void SlaveController::setupCommandHandlers() {
  Protocol::debug("Setting up command handlers...");

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
    if (speed > 0 && speed <= 300.0) {  // Keep limit at 300 inches/sec
      auto& config = MachineConfig::getInstance();
      config.motion.setSpeed(speed);  // Store in inches/sec
      motionController.setup();       // Update motion controller with new speed
      Protocol::sendResponse({"ok", "Speed updated to " + String(speed)});
    } else {
      Protocol::error("Invalid speed. Use value between 1-300 inches/sec");
    }
  });

  // Acceleration command
  commandHandler.registerCommand("setAccel", [this](const JsonVariant& params) {
    if (!params.containsKey("accel")) {
      Protocol::error("Missing acceleration value");
      return;
    }

    double accel = params["accel"].as<double>();
    if (accel > 0 && accel <= 300.0) {  // Keep limit at 100 inches/sec²
      auto& config = MachineConfig::getInstance();
      config.motion.setAcceleration(accel);  // Store in inches/sec²
      motionController
          .setup();  // Update motion controller with new acceleration
      Protocol::sendResponse(
          {"ok", "Acceleration updated to " + String(accel)});
    } else {
      Protocol::error(
          "Invalid acceleration. Use value between 1-300 inches/sec²");
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

  // Manual move command
  Protocol::debug("Registering manual_move command");
  commandHandler.registerCommand("manual_move", [this](
                                                    const JsonVariant& params) {
    Protocol::debug("Executing manual_move command");

    if (!params.containsKey("direction") || !params.containsKey("state")) {
      Protocol::error("Missing required parameters: direction, state");
      return;
    }

    const char* direction = params["direction"];
    const char* state = params["state"];
    float speed = params["speed"] | 20.0;  // Default speed if not specified
    float acceleration = params["acceleration"] | 10.0;  // Default acceleration

    speed = constrain(speed, 1.0, 50.0);
    acceleration = constrain(acceleration, 1.0, 50.0);

    if (strcmp(state, "START") == 0) {
      bool isXAxis =
          (strcmp(direction, "left") == 0 || strcmp(direction, "right") == 0);
      bool isPositive = (strcmp(direction, "right") == 0 ||
                         strcmp(direction, "forward") == 0);

      // Debug the direction parsing
      Protocol::debug("Direction: " + String(direction) + " isXAxis: " +
                      String(isXAxis) + " isPositive: " + String(isPositive));

      startContinuousMovement(isXAxis, isPositive, speed, acceleration);
      stateMachine.setState(State::MANUAL_MOVING);
      Protocol::sendResponse({"ok", "Started manual movement"});
    } else if (strcmp(state, "STOP") == 0) {
      stopManualMovement();
      stateMachine.setState(State::IDLE);
      Protocol::sendResponse({"ok", "Stopped manual movement"});
    } else {
      Protocol::error("Invalid state parameter. Use 'START' or 'STOP'");
    }
  });

  // Add settings command handler
  commandHandler.registerCommand("settings", [this](const JsonVariant& params) {
    Protocol::debug("Applying settings...");

    if (params.containsKey("speed")) {
      double speed = params["speed"].as<double>();
      auto& config = MachineConfig::getInstance();
      config.motion.setSpeed(speed);
      motionController.setup();
    }

    if (params.containsKey("acceleration")) {
      double accel = params["acceleration"].as<double>();
      auto& config = MachineConfig::getInstance();
      config.motion.setAcceleration(accel);
      motionController.setup();
    }

    Protocol::sendResponse({"ok", "Settings applied"});
  });

  // Set pickup location command
  commandHandler.registerCommand(
      "setPickupLocation", [this](const JsonVariant& params) {
        Protocol::debug("Setting pickup location");

        if (!params.containsKey("x") || !params.containsKey("y")) {
          Protocol::error("Missing x or y coordinates for pickup location");
          return;
        }

        double x = params["x"].as<double>();
        double y = params["y"].as<double>();

        // Store in settings
        auto& config = MachineConfig::getInstance();
        config.pattern.pickupX = x;
        config.pattern.pickupY = y;

        Protocol::sendResponse({"ok", "Pickup location updated"});
      });

  // Set box corner command
  commandHandler.registerCommand(
      "setBoxCorner", [this](const JsonVariant& params) {
        Protocol::debug("Setting box corner location");

        if (!params.containsKey("x") || !params.containsKey("y")) {
          Protocol::error("Missing x or y coordinates for box corner");
          return;
        }

        double x = params["x"].as<double>();
        double y = params["y"].as<double>();

        // Store in settings
        auto& config = MachineConfig::getInstance();
        config.pattern.boxX = x;
        config.pattern.boxY = y;

        Protocol::sendResponse({"ok", "Box corner location updated"});
      });

  // Set box dimensions command
  commandHandler.registerCommand(
      "setBoxDimensions", [this](const JsonVariant& params) {
        Protocol::debug("Setting box dimensions");

        if (!params.containsKey("length") || !params.containsKey("width")) {
          Protocol::error("Missing length or width for box dimensions");
          return;
        }

        double length = params["length"].as<double>();
        double width = params["width"].as<double>();

        auto& config = MachineConfig::getInstance();
        config.pattern.boxLength = length;
        config.pattern.boxWidth = width;

        Protocol::sendResponse({"ok", "Box dimensions updated"});
      });

  Protocol::debug("Command handlers setup complete");
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

  // Since we're using INPUT_PULLUP, the logic is inverted
  // The endstop is triggered when the pin reads LOW
  bool currentXEndstop = !xEndstop.read();  // Invert the reading
  bool currentYEndstop = !yEndstop.read();  // Invert the reading

  static bool lastXEndstop = false;
  static bool lastYEndstop = false;

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
  auto& config = MachineConfig::getInstance();

  // Store original speeds to restore later
  originalXSpeed = motionController.getXMaxSpeed();
  originalYSpeed = motionController.getYMaxSpeed();

  // Convert speed and acceleration to steps
  int32_t stepSpeed = ConversionConfig::inchesToSteps(speed);
  int32_t stepAccel = ConversionConfig::inchesToSteps(acceleration);

  if (isXAxis) {
    motionController.setManualMoveX(isPositive ? stepSpeed : -stepSpeed,
                                    stepAccel);
  } else {
    motionController.setManualMoveY(isPositive ? stepSpeed : -stepSpeed,
                                    stepAccel);
  }

  continuousMovementActive = true;
  continuousMovementIsX = isXAxis;
  continuousMovementPositive = isPositive;

  stateMachine.setState(State::MANUAL_MOVING);
}

void SlaveController::stopManualMovement() {
  if (!continuousMovementActive) return;

  motionController.stopManualMove();

  // Restore original speeds
  motionController.setXMaxSpeed(originalXSpeed);
  motionController.setYMaxSpeed(originalYSpeed);

  continuousMovementActive = false;
  stateMachine.setState(State::IDLE);
}

void SlaveController::updateState() {
  MachineState newState;
  bool stateChanged = false;

  // Get current state
  String currentStateStr =
      stateMachine.stateToString(stateMachine.getCurrentState());

  // With pull-down resistors:
  // Pin reads LOW (0) when not triggered
  // Pin reads HIGH (1) when triggered
  bool currentXEndstop = xEndstop.read();
  bool currentYEndstop = yEndstop.read();

  // Only update sensors if they've changed
  if (currentXEndstop != currentState.sensors.xEndstop ||
      currentYEndstop != currentState.sensors.yEndstop ||
      armExtended != currentState.sensors.armExtended ||
      suctionEnabled != currentState.sensors.suctionEnabled) {
    newState.sensors.xEndstop = currentXEndstop;
    newState.sensors.yEndstop = currentYEndstop;
    newState.sensors.armExtended = armExtended;
    newState.sensors.suctionEnabled = suctionEnabled;
    newState.sensorsChanged = true;
    stateChanged = true;
  }

  // Check if state has changed
  if (currentStateStr != currentState.status) {
    newState.status = currentStateStr;
    newState.statusChanged = true;
    stateChanged = true;
  }

  if (stateChanged) {
    Protocol::sendState(newState);
    // Update current state
    if (newState.sensorsChanged) {
      currentState.sensors = newState.sensors;
    }
    if (newState.statusChanged) {
      currentState.status = newState.status;
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