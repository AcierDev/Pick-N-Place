#include "SlaveController.h"

#include "config.h"

// Constructor
SlaveController::SlaveController()
    : currentStatus(Status::IDLE),
      currentPatternIndex(0),
      patternInProgress(false),
      targetX(0),
      targetY(0) {
  // Initialize steppers
  stepperX = new AccelStepper(1, STEPPER_X_STEP, STEPPER_X_DIR);
  stepperY = new AccelStepper(1, STEPPER_Y_STEP, STEPPER_Y_DIR);

  // Initialize endstops
  xEndstop.attach(ENDSTOP_X_PIN, INPUT_PULLUP);
  yEndstop.attach(ENDSTOP_Y_PIN, INPUT_PULLUP);
}

SlaveController::~SlaveController() {
  delete stepperX;
  delete stepperY;
}

void SlaveController::setup() {
  Serial.begin(BAUD_RATE);

  // Configure steppers
  stepperX->setMaxSpeed(STEPS_PER_INCH * settings.speedInches);
  stepperX->setAcceleration(STEPS_PER_INCH * settings.accelerationInches);
  stepperY->setMaxSpeed(STEPS_PER_INCH * settings.speedInches);
  stepperY->setAcceleration(STEPS_PER_INCH * settings.accelerationInches);

  // Configure pneumatics
  pinMode(EXTENSION_PIN, OUTPUT);
  pinMode(SUCTION_PIN, OUTPUT);
  retractArm();
  disableSuction();

  // Configure endstops
  xEndstop.interval(ENDSTOP_DEBOUNCE_MS);
  yEndstop.interval(ENDSTOP_DEBOUNCE_MS);

  Serial.println(F("Pick and Place Controller Ready"));
}

void SlaveController::loop() {
  // Update inputs
  xEndstop.update();
  yEndstop.update();

  // Check for serial commands
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("SETTINGS ")) {
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, input.substring(9));

      if (error) {
        Serial.println("Failed to parse settings");
      } else {
        updateSettings(doc.as<JsonObject>());
      }
    } else {
      processCommand(input);
    }
  }

  // Run state machine
  switch (currentStatus) {
    case Status::HOMING_X:
      homeXAxis();
      break;

    case Status::HOMING_Y:
      homeYAxis();
      break;

    case Status::MOVING:
      if (moveToXYPosition(targetX, targetY)) {
        currentStatus = Status::IDLE;
      }
      break;

      // Add other states as needed
  }

  // Send state updates periodically
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1000) {
    sendState();
    lastUpdate = millis();
  }
}

void SlaveController::processCommand(const String& command) {
  // Process incoming commands
  // Example: if (command == "START") { ... }
}

void SlaveController::updateSettings(const JsonObject& json) {
  // Update settings from JSON
  // Example: if (json.containsKey("motorSpeed")) { settings.motorSpeed =
  // json["motorSpeed"]; }
}

void SlaveController::sendState() {
  StaticJsonDocument<200> doc;
  doc["status"] = stateToString(currentStatus);

  // Add sensor readings or other state information
  // Example: doc["sensors"]["limit1"] = digitalRead(LIMIT_SWITCH_1_PIN);

  String output;
  serializeJson(doc, output);
  Serial.println("STATE " + output);
}

String SlaveController::stateToString(Status state) {
  switch (state) {
    case Status::IDLE:
      return "IDLE";
    case Status::BUSY:
      return "BUSY";
    case Status::ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}