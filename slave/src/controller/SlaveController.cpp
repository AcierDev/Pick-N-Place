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
      originalYSpeed(0) {}

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
}

void SlaveController::setupStateHandlers() {
  // Register state handlers with the state machine
  // Implementation will depend on your state handling needs

  stateMachine.registerHandler(State::HOME_REQUESTED, [this]() {
    // Start with X axis
    motionController.startHomingX();
    stateMachine.setState(State::HOMING_X);
  });

  stateMachine.registerHandler(State::HOMING_X, [this]() {
    if (xEndstop.rose() || xEndstop.read()) {
      // X endstop hit
      motionController.stopX();
      motionController.setXHome();
      // Start Y homing
      motionController.startHomingY();
      stateMachine.setState(State::HOMING_Y);
    }
  });

  stateMachine.registerHandler(State::HOMING_Y, [this]() {
    if (yEndstop.rose() || yEndstop.read()) {
      // Y endstop hit
      motionController.stopY();
      motionController.setYHome();
      stateMachine.setState(State::IDLE);
      Protocol::info("Homing complete");
    }
  });
}

void SlaveController::setupCommandHandlers() {
  // Home command
  commandHandler.registerCommand("home", [this](const JsonVariant& params) {
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
  commandHandler.registerCommand("setSpeed", [this](const String& params) {
    double newSpeed = params.toFloat();
    if (newSpeed > 0 && newSpeed <= 100.0) {
      auto& config = MachineConfig::getInstance();
      config.motion.speedInches = newSpeed;
      motionController.setup();  // This will apply the new speed
      Protocol::sendResponse({"ok", "Speed updated"});
    } else {
      Protocol::error("Invalid speed. Use value between 1-100", 5);
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

    const char* state = params["state"];
    if (strcmp(state, "on") == 0) {
      enableSuction();
      Protocol::sendResponse({"ok", "Suction enabled"});
    } else if (strcmp(state, "off") == 0) {
      disableSuction();
      Protocol::sendResponse({"ok", "Suction disabled"});
    } else {
      Protocol::error("Invalid suction state. Use 'on' or 'off'");
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

    // Generate the pattern
    currentPattern = patternGenerator.generatePattern(
        rows, cols, startX, startY, gridWidth, gridLength);

    if (currentPattern.empty()) {
      Protocol::error("Pattern too large for grid", 2);
      return;
    }

    Protocol::debug("Starting pattern with " + String(rows) + " rows, " +
                    String(cols) + " columns at (" + String(startX) + ", " +
                    String(startY) + ") with grid size " + String(gridWidth) +
                    "x" + String(gridLength) + " inches");

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
}

void SlaveController::updateInputs() {
  xEndstop.update();
  yEndstop.update();
}

void SlaveController::checkForCommands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Check if it's a command message
    if (input.startsWith("CMD ")) {
      // Parse JSON command
      StaticJsonDocument<512> doc;
      DeserializationError error = deserializeJson(doc, input.substring(4));

      if (error) {
        Protocol::error("Failed to parse command JSON", 1);
        return;
      }

      const char* type = doc["type"];
      if (!type) {
        Protocol::error("Missing command type", 2);
        return;
      }

      // Handle command with JSON parameters
      commandHandler.handleCommand(String(type), doc["params"]);
    }
  }
}

// Pneumatic control methods
void SlaveController::extendArm() { digitalWrite(PinConfig::extension, LOW); }

void SlaveController::retractArm() { digitalWrite(PinConfig::extension, HIGH); }

void SlaveController::enableSuction() { digitalWrite(PinConfig::suction, LOW); }

void SlaveController::disableSuction() {
  digitalWrite(PinConfig::suction, HIGH);
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